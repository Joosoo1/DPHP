import torch
from torch import nn
import numpy as np
import os
from copy import deepcopy
from utils.tools import extract_apg, extract_occ_grid, matrix_from_angle
from models.lit_autoencoder import LitAutoEncoder


class EthPredictor(torch.nn.Module):
    def __init__(
            self,
            pred_horizon: int = 15,
            input_horizon: int = 1,
            ae_state_dict: str = 'saves/misc/eth_autoencoder.h5',
            target_mode: str = 'velocity',
            rotate_scene: bool = True,
            map_size: int = 8,
    ):
        super(EthPredictor, self).__init__()

        # 超参数->预测时间步长
        self.pred_horizon = pred_horizon
        # 超参数->输入时间步长
        self.input_horizon = input_horizon
        # 自编码器加载路径
        self.ae_state_dict = ae_state_dict
        # 地图大小
        self.map_size = map_size
        # 预测目标->速度
        self.target_mode = target_mode
        self.rotate_scene = rotate_scene

        self._build_model()

    # 定义网络结构
    def _build_model(self):
        """
        Builds the model
        """
        # 预测对象的LSTM层
        self.ego = torch.nn.LSTM(2, 32, batch_first=True)

        # social的全连接层->其他行人
        self.apg_fc = torch.nn.Sequential(
            torch.nn.Linear(72, 128),
            torch.nn.ReLU()
        )
        # social的LSTM层
        self.apg_lstm = torch.nn.LSTM(128, 128, batch_first=True)

        # 自编码器，是预训练好的自编码器
        self.ae = LitAutoEncoder()
        # 加载自编码器的参数
        if self.ae_state_dict is not None and os.path.exists(self.ae_state_dict):
            self.ae.load_state_dict(torch.load(self.ae_state_dict))
        # 冻结自编码器的参数，防止在训练过程中更新
        self.ae.freeze()
        # map的LSTM层->处理经过自编码器的地图信息
        self.map = torch.nn.LSTM(64, 256, batch_first=True)

        # 最后聚合的LSTM层
        self.concat = torch.nn.LSTM(416, 512, batch_first=True)
        # 最后两层全连接层
        self.linear = torch.nn.Sequential(
            torch.nn.Linear(512, 256),
            torch.nn.ReLU(),
            torch.nn.Linear(256, 2 * self.pred_horizon)
        )

        self.lstm_ego_size = 32
        self.lstm_apg_size = 128
        self.lstm_map_size = 256
        self.lstm_concat_size = 512
        self._lstm_map_state = None
        self._lstm_concat_state = None
        self._lstm_ego_state = None
        self._lstm_apg_state = None
        self._state_ids = []

    def _ae_loop_tbptt(self, x):
        """
        Performs the autoencoder loop for the map input
        """
        for i in range(x.shape[1]):
            if i == 0:
                output = self.ae(x[:, i, :, :].unsqueeze(1)).unsqueeze(1)
            else:
                output = torch.cat([output, self.ae(x[:, i, :, :].unsqueeze(1)).unsqueeze(1)], dim=1)
        return output

    # 定义前向传播
    def forward(self, i):
        """
        Forward pass of the model
        """
        # 行人历史状态：键值对
        x = i['ego_input'].float()
        # 其他行人历史状态
        y = i['others_input'].float()
        # 地图信息
        z = i['map_input'].float()

        N, T, _ = x.size()
        if 'ids' not in i.keys():
            # 如果当前输入无键"ids"，则初始化所有的LSTM层的状态为零
            self.zero_states(N, device=x.device)
        else:
            # 如果当前输入有键"ids"，则根据输入的id初始化对应的LSTM层的状态
            self.init_states(i['ids'], device=x.device)
        # 特征提取->行人状态
        x, self._lstm_ego_state = self.ego(x, self._lstm_ego_state)
        # 特征提取->其他行人状态->social
        y = self.apg_fc(y)
        y, self._lstm_apg_state = self.apg_lstm(y, self._lstm_apg_state)
        # 特征提取->自动编码，地图信息
        z = self._ae_loop_tbptt(z)
        z, self._lstm_map_state = self.map(z, self._lstm_map_state)
        # 特征融合
        x = torch.cat((x, y, z), dim=2)
        x, self._lstm_concat_state = self.concat(x, self._lstm_concat_state)
        pred = self.linear(x)[:, -1, :]
        # 返回预测结果
        return pred

    # TODO: add other initialization method instead of zeros (xavier ?)
    def zero_states(self, batch_size, device='cpu'):
        """
        Initializes the states of the model to zero
        """
        self._lstm_map_state = (
            torch.zeros([1, batch_size, self.lstm_map_size], device=device),
            torch.zeros([1, batch_size, self.lstm_map_size], device=device))

        self._lstm_concat_state = (
            torch.zeros([1, batch_size, self.lstm_concat_size], device=device),
            torch.zeros([1, batch_size, self.lstm_concat_size], device=device))

        self._lstm_ego_state = (
            torch.zeros([1, batch_size, self.lstm_ego_size], device=device),
            torch.zeros([1, batch_size, self.lstm_ego_size], device=device))

        self._lstm_apg_state = (
            torch.zeros([1, batch_size, self.lstm_apg_size], device=device),
            torch.zeros([1, batch_size, self.lstm_apg_size], device=device))

    def init_states(self, ids, device='cpu'):
        """
        Initializes the states of the model for a given batch of ids
        """
        ids = list(ids)
        prev_lstm_ego_state = deepcopy(self._lstm_ego_state)
        prev_lstm_apg_state = deepcopy(self._lstm_apg_state)
        prev_lstm_map_state = deepcopy(self._lstm_map_state)
        prev_lstm_concat_state = deepcopy(self._lstm_concat_state)

        self.zero_states(len(ids), device=device)

        for i, current_id in enumerate(ids):
            if current_id in self._state_ids:
                prev_index = self._state_ids.index(current_id)
                self._lstm_ego_state[0][0, i, :] = prev_lstm_ego_state[0][0, prev_index, :]
                self._lstm_ego_state[1][0, i, :] = prev_lstm_ego_state[1][0, prev_index, :]
                self._lstm_apg_state[0][0, i, :] = prev_lstm_apg_state[0][0, prev_index, :]
                self._lstm_apg_state[1][0, i, :] = prev_lstm_apg_state[1][0, prev_index, :]
                self._lstm_map_state[0][0, i, :] = prev_lstm_map_state[0][0, prev_index, :]
                self._lstm_map_state[1][0, i, :] = prev_lstm_map_state[1][0, prev_index, :]
                self._lstm_concat_state[0][0, i, :] = prev_lstm_concat_state[0][0, prev_index, :]
                self._lstm_concat_state[1][0, i, :] = prev_lstm_concat_state[1][0, prev_index, :]
        self._state_ids = ids

    # 从原始数据中网络所需输入
    def _extract_inputs(self, ego_input_horizon, other_states, map_state):
        """
        Extracts the model input from raw data
        """
        ego_state = ego_input_horizon[-1]
        if len(other_states) > 1:
            other_states = np.array([state for state in other_states if (state != ego_state).any()])

        example = {}
        vel = np.array(ego_state[3:])
        pos = np.array(ego_state[:2])
        angle = -np.arctan2(vel[1], vel[0])
        if self.rotate_scene:
            rot = matrix_from_angle(angle)
        else:
            rot = np.eye(2)
        example['map_input'] = extract_occ_grid(pos, -angle, map_state, self.map_size, 60)
        example['ego_input'] = rot.dot(vel)
        example['others_input'] = extract_apg(
            np.array([rot.dot(other_state[:2] - pos) for other_state in other_states]),
            n_bins=72,
            max_range=20,
        )
        return example

    # 从ROS的数据格式中提取模型输入，并返回模型所需的输入格式
    def extract_batch_inputs_from_ros(self, tracked_agents, map_state):
        """
        Extracts the model input from a ROS message
        """
        history = tracked_agents
        tracked_agents = tracked_agents[-1]

        examples = {
            'ego_input': [],
            'others_input': [],
            'map_input': [],
            'ids': []
        }
        for a in tracked_agents:
            other_states = [i['state'] for i in tracked_agents]
            ego_history = [aa['state'] for hist_step in history for aa in hist_step if aa['id'] == a['id']]
            if len(ego_history) < self.input_horizon:
                continue
            example = self._extract_inputs(np.array(ego_history), np.array(other_states), map_state)

            for key, item in example.items():
                examples[key].append(item)
            examples['ids'].append(a['id'])

        # 将所有的输入转换为PyTorch张量
        examples = {key: torch.unsqueeze(torch.tensor(item), 1) for key, item in examples.items()}
        return examples

    def map_predictions_to_world(self, tracked_agents, predictions):
        """
        Maps the predictions to world coordinates
        """
        predictions = np.array(predictions).reshape(len(predictions), -1, 2)
        world_predictions = np.empty([len(predictions), self.pred_horizon, 4])

        for i, (agent, prediction) in enumerate(zip(tracked_agents, predictions)):
            if self.rotate_scene:
                R = matrix_from_angle(tracked_agents[i]['state'][2])
            else:
                R = np.eye(2)

            prediction = R.dot(prediction.T).T

            if self.target_mode == 'velocity':
                world_predictions[i, :, 2:] = prediction
                p = np.array([0, 0])
                for t, vel in enumerate(prediction):
                    p = p + (vel * (1 / 5))
                    world_predictions[i, t, :2] = p
            elif self.target_mode == 'position':
                world_predictions[i, :, :2] = prediction
                world_predictions[i, 1:, 2:] = np.diff(prediction, axis=0) * 5
                world_predictions[i, 0, 2:] = world_predictions[i, 1, 2:]
                world_predictions[i, 0, 2:] = agent['state'][3:]

            world_predictions[i, :, :2] += agent['state'][:2]

        return world_predictions

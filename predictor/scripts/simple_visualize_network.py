import torch
from torch import nn
from torchviz import make_dot
import os
import sys

# 添加项目路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

class EthPredictor(torch.nn.Module):
    def __init__(
            self,
            pred_horizon: int = 15,
            input_horizon: int = 1,
            target_mode: str = 'velocity',
            rotate_scene: bool = True,
            map_size: int = 8,
    ):
        super(EthPredictor, self).__init__()

        # 超参数->预测时间步长
        self.pred_horizon = pred_horizon
        # 超参数->输入时间步长
        self.input_horizon = input_horizon
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

        # 自编码器，是预训练好的自编码器（简化版本）
        self.ae_encoder = nn.Sequential(
            nn.Conv2d(1, 64, kernel_size=5, stride=2, padding=2),
            nn.ReLU(),
            nn.Conv2d(64, 32, kernel_size=3, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(32, 8, kernel_size=3, stride=2, padding=1),
            nn.ReLU(),
        )
        self.ae_fc = nn.Linear(512, 64)
        
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
                z = self.ae_encoder(x[:, i, :, :].unsqueeze(1))
                z = z.view(-1, 512)
                output = self.ae_fc(z).unsqueeze(1)
            else:
                z = self.ae_encoder(x[:, i, :, :].unsqueeze(1))
                z = z.view(-1, 512)
                output = torch.cat([output, self.ae_fc(z).unsqueeze(1)], dim=1)
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
        # 初始化所有的LSTM层的状态为零
        self.zero_states(N, device=x.device)
        
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


def visualize_eth_predictor():
    """可视化EthPredictor网络结构"""
    # 创建模型实例
    model = EthPredictor()
    
    # 创建示例输入数据
    # 根据forward函数中的输入要求构造示例数据
    batch_size = 1
    input_horizon = 1
    pred_horizon = 15
    
    # 模拟输入数据
    dummy_input = {
        'ego_input': torch.randn(batch_size, input_horizon, 2),  # 速度信息
        'others_input': torch.randn(batch_size, 1, 72),  # social信息
        'map_input': torch.randn(batch_size, input_horizon, 60, 60),  # 地图信息
    }
    
    # 执行前向传播以创建计算图
    output = model(dummy_input)
    
    # 创建网络结构图
    dot = make_dot(output, params=dict(model.named_parameters()))
    
    # 保存为图片
    dot.render("eth_predictor_network", format="png")
    print("EthPredictor网络结构图已保存为 eth_predictor_network.png")
    
    return dot

def visualize_autoencoder():
    """可视化自编码器网络结构"""
    # 创建自编码器实例（简化版本）
    class SimpleAutoEncoder(nn.Module):
        def __init__(self):
            super().__init__()
            self.encoder = nn.Sequential(
                nn.Conv2d(1, 64, kernel_size=5, stride=2, padding=2),
                nn.ReLU(),
                nn.Conv2d(64, 32, kernel_size=3, stride=2, padding=1),
                nn.ReLU(),
                nn.Conv2d(32, 8, kernel_size=3, stride=2, padding=1),
                nn.ReLU(),
            )
            self.fc1 = nn.Linear(512, 64)
            self.fc2 = nn.Linear(64, 512)
            self.decoder = nn.Sequential(
                nn.ReLU(),
                nn.ConvTranspose2d(8, 32, kernel_size=3, stride=2, padding=1),
                nn.ReLU(),
                nn.ConvTranspose2d(32, 64, kernel_size=3, stride=2, padding=1, output_padding=1),
                nn.ReLU(),
                nn.ConvTranspose2d(64, 1, kernel_size=5, stride=2, padding=2, output_padding=1)
            )

        def forward(self, x):
            z = self.encoder(x)
            z = z.view(-1, 512)
            z = self.fc1(z)
            return z
    
    # 创建自编码器实例
    model = SimpleAutoEncoder()
    
    # 创建示例输入数据 (batch_size, channels, height, width)
    dummy_input = torch.randn(1, 1, 60, 60)
    
    # 执行前向传播以创建计算图
    output = model(dummy_input)
    
    # 创建网络结构图
    dot = make_dot(output, params=dict(model.named_parameters()))
    
    # 保存为图片
    dot.render("autoencoder_network", format="png")
    print("自编码器网络结构图已保存为 autoencoder_network.png")
    
    return dot

if __name__ == "__main__":
    # 确保保存目录存在
    save_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "network_visualizations")
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    os.chdir(save_dir)
    
    print("开始生成网络结构图...")
    
    # 可视化EthPredictor
    try:
        eth_dot = visualize_eth_predictor()
        print("EthPredictor网络结构图生成成功")
    except Exception as e:
        print(f"EthPredictor网络结构图生成失败: {e}")
    
    # 可视化自编码器
    try:
        ae_dot = visualize_autoencoder()
        print("自编码器网络结构图生成成功")
    except Exception as e:
        print(f"自编码器网络结构图生成失败: {e}")
    
    print(f"网络结构图已保存到: {save_dir}")
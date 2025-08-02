# ！/home/joosoo/Dynamic_explorer/predictor/scl/venv/scl/bin/python3
import rospy
import torch
from argparse import ArgumentParser
import pytorch_lightning as pl
from time import time
from copy import deepcopy
from rosinterfaces.rosbackbone import RosBackbone
from training_routines.ewc import EwcPredictor


# 定义预测节点类，继承自RosBackbone
class PredictorNode(RosBackbone):
    def __init__(self, frequency: float = 20.0, **kwargs):
        # 初始化父类RosBackbone
        RosBackbone.__init__(self)
        self.frequency = frequency  # 设置运行频率
        rospy.sleep(1)  # 等待一秒确保ROS节点初始化完成
        self.history = []  # 初始化历史轨迹列表
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')  # 确定使用的设备

    def run(self, model):
        prev = time()  # 记录上一帧的时间戳
        while not rospy.is_shutdown():
            now = time()
            dura = now - prev
            if (now - prev) < (1 / self.frequency):  # 如果未达到设定的频率间隔
                rospy.sleep((1 / self.frequency) - (now - prev))  # 则让程序休眠至下一帧
            else:
                print(dura)
                print("WARN : Not keeping up to rate")  # 若无法保持设定频率，则输出警告信息
            prev = time()

            # 将当前追踪到的代理状态添加到历史记录中
            self.history.append(deepcopy(self.tracked_agents))
            if len(self.history) < model.predictor.input_horizon:
                continue  # 若历史记录长度不足模型输入时域要求，则跳过本次循环

            # 若历史记录长度超过模型输入时域要求，则移除最早的数据
            while len(self.history) > model.predictor.input_horizon:
                self.history.pop(0)

            if len(self.tracked_agents) == 0:
                continue

            with torch.no_grad():
                # 预处理当前状态并从ROS接口提取输入数据
                inputs = model.predictor.extract_batch_inputs_from_ros(self.history, self.map_state)
                inputs = inputs.to(self.device)

                # 使用模型进行推理
                predictions = model(inputs)

                # 对预测结果进行后处理，并映射到世界坐标系下
                trajectories = model.predictor.map_predictions_to_world(self.history[-1], predictions)

            # 将预测的轨迹可视化并发布到ROS主题上供显示
            self.visualize_trajectories(self.history[-1], trajectories[:, :, :2])

            # 将预测的轨迹消息发布到ROS主题上，供运动规划使用
            self.publish_trajectories(self.history[-1], trajectories)


# 主函数入口
def cli_main():
    # 设置随机种子，确保实验可复现性
    pl.seed_everything(1234)

    # 解析命令行参数
    parser = ArgumentParser()
    parser.add_argument('--model', type=str, default='StateDiffs')
    parser.add_argument('--save_name', type=str, default='coop_dev')
    parser.add_argument('--frequency', type=int, default=20)
    args = parser.parse_args()

    # 加载预训练好的EwcPredictor模型并设置为评估模式
    model = EwcPredictor.load_from_checkpoint(f'saves/{args.model}/{args.save_name}/final.ckpt')
    dvice = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model.to(dvice)
    # 置于评估模式
    model.eval()

    # 初始化ROS节点并启动预测循环
    rospy.init_node('predictor_node')
    ewc_node = PredictorNode(**vars(args))
    ewc_node.run(model)


if __name__ == '__main__':
    cli_main()  # 运行主函数入口

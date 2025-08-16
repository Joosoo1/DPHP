import torch
from torch import nn
from torchviz import make_dot
import os
import sys

# 添加项目路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from models.eth_predictor import EthPredictor
from models.lit_autoencoder import LitAutoEncoder

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
    # 创建自编码器实例
    model = LitAutoEncoder()
    
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
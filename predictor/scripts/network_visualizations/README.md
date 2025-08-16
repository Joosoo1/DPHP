# DPHP Planner Predictor 网络结构说明

## EthPredictor 网络结构

EthPredictor 是一个用于行人轨迹预测的复合神经网络模型，包含以下几个主要组件：

### 1. Ego LSTM (行人状态处理)
- 输入维度: 2 (速度信息)
- 隐藏层维度: 32
- 层类型: LSTM

### 2. Social Processing (其他行人状态处理)
- APG FC (全连接层):
  - 输入维度: 72 (APG特征)
  - 输出维度: 128
  - 激活函数: ReLU
- APG LSTM:
  - 输入维度: 128
  - 隐藏层维度: 128
  - 层类型: LSTM

### 3. Map Processing (地图信息处理)
- 自编码器 (预训练):
  - 编码器: 3层卷积网络
    - Conv2d(1, 64, kernel_size=5, stride=2, padding=2) + ReLU
    - Conv2d(64, 32, kernel_size=3, stride=2, padding=1) + ReLU
    - Conv2d(32, 8, kernel_size=3, stride=2, padding=1) + ReLU
  - 全连接层:
    - Linear(512, 64)
- Map LSTM:
  - 输入维度: 64
  - 隐藏层维度: 256
  - 层类型: LSTM

### 4. 特征融合与预测
- Concat LSTM (特征聚合):
  - 输入维度: 416 (32+128+256)
  - 隐藏层维度: 512
  - 层类型: LSTM
- 线性层:
  - Linear(512, 256) + ReLU
  - Linear(256, 2*pred_horizon)

### 输入输出
- 输入:
  - ego_input: 行人自身的状态信息
  - others_input: 其他行人的状态信息 (APG特征)
  - map_input: 地图信息
- 输出:
  - 预测的轨迹点 (pred_horizon个时间步，每个时间步2个坐标值)

## 自编码器网络结构

自编码器用于处理地图信息，包含编码器和解码器两部分：

### 编码器
1. Conv2d(1, 64, kernel_size=5, stride=2, padding=2) + ReLU
2. Conv2d(64, 32, kernel_size=3, stride=2, padding=1) + ReLU
3. Conv2d(32, 8, kernel_size=3, stride=2, padding=1) + ReLU
4. Linear(512, 64)

### 解码器
1. Linear(64, 512)
2. ConvTranspose2d(8, 32, kernel_size=3, stride=2, padding=1) + ReLU
3. ConvTranspose2d(32, 64, kernel_size=3, stride=2, padding=1, output_padding=1) + ReLU
4. ConvTranspose2d(64, 1, kernel_size=5, stride=2, padding=2, output_padding=1)

网络结构图已保存为PNG格式文件：
- eth_predictor_network.png
- autoencoder_network.png
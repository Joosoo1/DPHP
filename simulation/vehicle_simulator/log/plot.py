import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from pathlib import Path

def configure_matplotlib():
    """专业可视化配置"""
    mpl.rcParams.update({
        'font.size': 12,
        'axes.titlesize': 14,
        'axes.labelsize': 12,
        'xtick.labelsize': 10,
        'ytick.labelsize': 10,
        'legend.fontsize': 10,
        'grid.linestyle': '--',
        'grid.alpha': 0.7,
        'savefig.bbox': 'tight',
        'savefig.transparent': False,
    })

def load_data(file_path):
    """数据加载增强版"""
    try:
        data = np.loadtxt(file_path, delimiter=None)
        return {
            'time': data[:, 3],
            'volume': data[:, 0],
            'distance': data[:, 1],
            'runtime': data[:, 2]
        }
    except Exception as e:
        print(f"加载 {file_path} 失败: {str(e)}")
        return None

def create_visualization(datasets, labels, output_png=None):
    """专业对比可视化"""
    # fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 9), dpi=120)
    fig, (ax_left, ax_right) = plt.subplots(1, 2, figsize=(16, 6), dpi=120)
    
    # 可视化样式配置
    style_presets = [
        {'color': '#1f77b4', 'ls': '-', 'marker': 'o', 'mec': 'none'},
        {'color': '#ff7f0e', 'ls': '--', 'marker': 's', 'mec': 'none'},
        {'color': '#2ca02c', 'ls': '-.', 'marker': '^', 'mec': 'none'}
    ]
    
    lines = []
    for i, (data, label) in enumerate(zip(datasets, labels)):
        style = style_presets[i]  # 提取样式参数
        line = ax_left.plot(
            data['time'], 
            data['volume'],
            label=label,
            markersize=5,
            linewidth=0.8,
            alpha=0.8,
            **style  # 关键字参数展开
        )
        lines.append(line[0])

    ax_left.set_xlabel('Time Duration (s)', fontweight='bold')
    ax_left.set_ylabel('Explored Volume (m³)', fontweight='bold')
    # ax_left.grid(True, which='both', alpha=0.4)
    ax_left.grid(False)

    ax_left.legend(
        loc='lower right',
        labels=labels,
        bbox_to_anchor=(0.98, 0.02),  # 距离右边2%，底边2%
        frameon=True,
        framealpha=0.95,
        borderpad=1,
        ncol=1  # 单列显示
    )

    # 距离-体积曲线绘制修正
    for i, (data, label) in enumerate(zip(datasets, labels)):
        style = style_presets[i]  # 提取样式参数
        ax_right.plot(
            data['distance'], 
            data['volume'],
            markersize=5,
            linewidth=1.8,
            alpha=0.8,
            **style  # 关键字参数展开
        )
    
    ax_right.set_xlabel('Flight Distance (m)', fontweight='bold')
    ax_right.set_ylabel('Explored Volume (m³)', fontweight='bold')
    # ax_right.grid(True, which='both', alpha=0.4)
    ax_right.grid(False)

    # 右图图例配置
    ax_right.legend(
        loc='lower right',
        labels=labels,
        bbox_to_anchor=(0.98, 0.02),
        frameon=True,
        framealpha=0.95,
        borderpad=1,
        ncol=1
    )

    plt.subplots_adjust(wspace=0.25)  # 增加水平间距

    # 统一图例
    # fig.legend(handles=lines,
    #           labels=labels,
    #           loc='upper center',
    #           ncol=3,
    #           bbox_to_anchor=(0.5, 1.02),
    #           frameon=True,
    #           shadow=True,
    #           borderpad=1)
    
    # plt.subplots_adjust(top=0.85)  # 顶部留出15%空间
    # plt.tight_layout(pad=3.0)
    
    if output_png:
        plt.savefig(output_png, dpi=300, bbox_inches='tight')
        print(f"结果已保存至：{Path(output_png).resolve()}")
    plt.show()

if __name__ == "__main__":
    configure_matplotlib()
    
    # 配置数据集 (修改为实际路径)
    data_config = [
        {'path': "/home/joosoo/Dynamic_explorer/LOG/metric_2025-4-6-22-25-22.txt", 'label': "proposed"},
    ]
    
    # 加载数据
    datasets = []
    labels = []
    for conf in data_config:
        data = load_data(conf['path'])
        if data is not None:
            datasets.append(data)
            labels.append(conf['label'])
    
    if len(datasets) >= 1:
        create_visualization(datasets, labels, "comparison_result.png")
    else:
        print("错误：没有有效数据可供可视化")

# import numpy as np
# import matplotlib.pyplot as plt
# import matplotlib as mpl
# from pathlib import Path

# def configure_matplotlib():
#     """专业级Matplotlib配置"""
#     mpl.rcParams.update({
#         'font.size': 12,
#         'axes.titlesize': 14,
#         'axes.labelsize': 12,
#         'xtick.labelsize': 10,
#         'ytick.labelsize': 10,
#         'legend.fontsize': 10,
#         'grid.linestyle': '--',
#         'grid.alpha': 0.7,
#         'savefig.bbox': 'tight',
#         'savefig.transparent': False,
#     })

# def load_data(file_path):
#     """增强型数据加载"""
#     data = np.loadtxt(file_path, delimiter=None)
#     return {
#         'time': data[:, 3],
#         'volume': data[:, 0],
#         'distance': data[:, 1],
#         'runtime': data[:, 2]
#     }

# def create_visualization(data, output_png=None):
#     """创建科研级可视化"""
#     fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), dpi=120)
    
#     # 探索体积曲线
#     ax1.plot(data['time'], data['volume'], 
#             color='#2C5F8B', marker='o', markersize=1,
#             linewidth=1.5, label='探索体积')
#     # ax1.set_title('探索体积时间演化曲线', pad=15)
#     ax1.set_xlabel('Time Duration (s)')
#     ax1.set_ylabel('')
#     ax1.grid(True)
    
#     # 行驶距离曲线
#     ax2.plot(data['distance'], data['volume'],
#             color='#B22222', marker='s', markersize=1,
#             linewidth=1.5, label='移动距离')
#     # ax2.set_title('移动距离时间演化曲线', pad=15)
#     ax2.set_xlabel('Flight Distance (m)')
#     ax2.set_ylabel('Explored Volume (m³)')
#     ax2.grid(True)
    
#     plt.tight_layout(pad=3.0)
    
#     if output_png:
#         plt.savefig(output_png, dpi=300)
#         print(f"可视化结果已保存至：{Path(output_png).resolve()}")
#     plt.show()

# if __name__ == "__main__":
#     # 初始化配置
#     configure_matplotlib()
    
#     # 数据文件路径（根据实际情况修改）
#     data_file = "/home/joosoo/Dynamic_explorer/base_workspace/src/vehicle_simulator/log/tare/scene_1/metrics/metric_2025-4-2-2-56-34.txt"
    
#     try:
#         dataset = load_data(data_file)
#         create_visualization(dataset, "result.png")
#     except FileNotFoundError:
#         print(f"错误：数据文件 {data_file} 未找到")
#     except Exception as e:
#         print(f"数据处理异常：{str(e)}")
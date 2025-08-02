import rospy
import numpy as np
from geometry_msgs.msg import Pose
from tracking_msgs.msg import TrackedPersons, Trajectories, Trajectory
from nav_msgs.msg import OccupancyGrid, Odometry
from visualization_msgs.msg import MarkerArray, Marker
from pyquaternion import Quaternion
from utils.tools import yaw_from_quaternion


class RosBackbone:
    # 初始化对象变量
    def __init__(self):
        # 全局变量
        self.tracked_agents = []
        self.map_state = None

        # ROS subscribers
        self.peds_state_topic = rospy.get_param('/tracked_agents_topic', "/tracked_persons")
        self.grid_topic = rospy.get_param('/grid_topic', "/map")

        rospy.Subscriber(self.peds_state_topic, TrackedPersons, self.peds_state_cb, queue_size=1)
        rospy.Subscriber(self.grid_topic, OccupancyGrid, self.map_state_cb, queue_size=1)

        # ROS Publisher(s)
        self.prediction_viz_topic = rospy.get_param('/prediction_topic', "predictions/visualizations")
        self.prediction_topic = rospy.get_param('/prediction_topic', "~predictions")

        self.pub_viz = rospy.Publisher(self.prediction_viz_topic, MarkerArray, queue_size=10)
        self.pub_trajs = rospy.Publisher(self.prediction_topic, Trajectories, queue_size=10)

        # Colors
        self.colors = []
        self.colors.append([0.8500, 0.3250, 0.0980])  # orange
        self.colors.append([0.0, 0.4470, 0.7410])  # blue
        self.colors.append([0.4660, 0.6740, 0.1880])  # green
        self.colors.append([0.4940, 0.1840, 0.5560])  # purple
        self.colors.append([0.9290, 0.6940, 0.1250])  # yellow
        self.colors.append([0.3010, 0.7450, 0.9330])  # cyan
        self.colors.append([0.6350, 0.0780, 0.1840])  # chocolate
        self.colors.append([1, 0.6, 1])  # pink
        self.colors.append([0.505, 0.505, 0.505])  # grey

    # 订阅追踪到的行人轨迹信息，某一时刻各个行人的平面位置和速度信息，并存储到对象变量中，存储方式字典列表
    def peds_state_cb(self, msg):
        self.tracked_agents = []
        for p in msg.tracks:
            # 字典列表
            self.tracked_agents.append({
                'state': [
                    # 位置
                    p.pose.pose.position.x,
                    p.pose.pose.position.y,
                    # 朝向
                    np.arctan2(
                        p.twist.twist.linear.y,
                        p.twist.twist.linear.x
                    ),
                    # 速度
                    p.twist.twist.linear.x,
                    p.twist.twist.linear.y
                ],
                # 行人编号
                'id': p.track_id
            })

    # 订阅机器人周围的环境信息，环境信息由占据栅格地图表示，包含栅格的占据属性，栅格原点位置，栅格分辨率，储存方式为字典列表
    def map_state_cb(self, msg):
        res = {}
        msg.data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        res['image'] = msg.data
        res['origin'] = [
            np.array(msg.info.origin.position.x),
            np.array(msg.info.origin.position.y),
            0]
        res['resolution'] = msg.info.resolution

        self.map_state = res

    # 可视化预测到轨迹
    def visualize_trajectories(self, tracked_agents, predictions):
        global_trajectory = MarkerArray()
        m_id = 0
        for agent, prediction in zip(tracked_agents, predictions):
            i = agent['id'] % 9
            for t, pos in enumerate(prediction):
                marker = Marker()
                marker.header.frame_id = "map"
                # marker.header.frame_id = "odom"
                marker.header.stamp = rospy.Time(0)
                marker.ns = "goal_marker"
                marker.type = 3
                marker.scale.x = 0.3 * 2.0
                marker.scale.y = 0.3 * 2.0
                marker.scale.z = 0.1
                marker.color.a = 0.0
                marker.id = m_id  # int(str(agent['id'])+str(t))
                marker.lifetime = rospy.Duration(0, int(20 * 10e6))
                pose = Pose()
                pose.position.x = pos[0]
                pose.position.y = pos[1]
                marker.color.a = 0.8 * ((len(prediction) - t) / len(prediction))
                marker.color.r = self.colors[i][0]
                marker.color.g = self.colors[i][1]
                marker.color.b = self.colors[i][2]
                pose.orientation.w = 1.0
                marker.pose = pose
                global_trajectory.markers.append(marker)
                m_id += 1

        self.pub_viz.publish(global_trajectory)

    # 发布预测的轨迹信息
    def publish_trajectories(self, tracked_agents, predictions):
        result = Trajectories()
        # 遍历每个被追踪的智能体及其预测轨迹
        for agent, prediction in zip(tracked_agents, predictions):
            agent_traj = Trajectory()
            # 设置智能体的ID信息
            agent_traj.track_id = agent['id']
            agent_traj.frequency = 5

            # 遍历该智能体的所有预测时间步长
            for t, state in enumerate(prediction):
                odom = Odometry()
                odom.pose.pose.position.x = state[0]
                odom.pose.pose.position.y = state[1]

                # 提取并设置速度分量
                vx, vy = state[2:4]

                # 根据速度计算航向角，并生成对应的四元数
                q = Quaternion(axis=[0, 0, 1], angle=np.arctan2(vy, (vx + 1e-9)))

                # 如果存在线速度，则设置Odometry消息的方位角（orientation）
                if vx != 0:
                    odom.pose.pose.orientation.x = q.x
                    odom.pose.pose.orientation.y = q.y
                    odom.pose.pose.orientation.w = q.w
                    odom.pose.pose.orientation.z = q.z

                # 设置Odometry消息的速度信息
                odom.twist.twist.linear.x = vx
                odom.twist.twist.linear.y = vy

                # 设置Odometry消息的协方差矩阵（这里使用行人运动半径作为单位不确定性）
                odom.pose.covariance = (np.eye(6) * 0.6).flatten()

                # 将当前预测时间步长的Odometry消息添加到智能体的Trajectory对象中
                agent_traj.data.append(odom)

            # 将当前智能体的Trajectory对象添加到Trajectories消息中
            result.trajectories.append(agent_traj)

        self.pub_trajs.publish(result)

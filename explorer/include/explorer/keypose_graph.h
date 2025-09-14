

#ifndef SENSOR_COVERAGE_PLANNER_KEYPOSE_GRAPH_H
#define SENSOR_COVERAGE_PLANNER_KEYPOSE_GRAPH_H

#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

#include <memory>
#include <utility>
#include <vector>

#include "explorer/planning_env.h"

namespace viewpoint_manager_ns {
    class ViewPointManager;
}

namespace keypose_graph_ns {
    struct KeyposeNode;
    class KeyposeGraph;
    constexpr double INF = 9999.0;
    typedef std::pair<int, int> iPair;
}  // namespace keypose_graph_ns

struct keypose_graph_ns::KeyposeNode {
    geometry_msgs::Point position_;
    geometry_msgs::Point offset_to_keypose_;
    int keypose_id_;
    int node_ind_;
    int cell_ind_;
    bool is_keypose_;
    bool is_connected_;

public:
    explicit KeyposeNode(double x = 0, double y = 0, double z = 0, int node_ind = 0,
                         int keypose_id = 0, bool is_keypose = true);
    explicit KeyposeNode(const geometry_msgs::Point& point, int node_ind = 0, int keypose_id = 0,
                         bool is_keypose = true);
    ~KeyposeNode() = default;
    bool IsKeypose() const {
        return is_keypose_;
    }
    bool IsConnected() const {
        return is_connected_;
    }
    void SetOffsetToKeypose(const geometry_msgs::Point& offset_to_keypose) {
        offset_to_keypose_ = offset_to_keypose;
    }
    void SetCurrentKeyposePosition(const geometry_msgs::Point& current_keypose_position) {
        offset_to_keypose_.x = position_.x - current_keypose_position.x;
        offset_to_keypose_.y = position_.y - current_keypose_position.y;
        offset_to_keypose_.z = position_.z - current_keypose_position.z;
    }
};

class keypose_graph_ns::KeyposeGraph {
private:
    bool allow_vertical_edge_;
    int current_keypose_id_;
    geometry_msgs::Point current_keypose_position_;
    std::vector<std::vector<int>> graph_;    // 存储索引
    std::vector<std::vector<double>> dist_;  // 对应距离
    std::vector<bool> in_local_planning_horizon_;
    std::vector<KeyposeNode> nodes_;  // 节点
    std::vector<geometry_msgs::Point> node_positions_;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_connected_nodes_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr connected_nodes_cloud_;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_nodes_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr nodes_cloud_;

    std::vector<int> connected_node_indices_;

    double kAddNodeMinDist;
    double kAddNonKeyposeNodeMinDist;
    double kAddEdgeConnectDistThr;
    double kAddEdgeToLastKeyposeDistThr;
    double kAddEdgeVerticalThreshold;
    double kAddEdgeCollisionCheckResolution;
    double kAddEdgeCollisionCheckRadius;
    int kAddEdgeCollisionCheckPointNumThr;

    static bool ComparePair(const std::pair<int, int>& a, const std::pair<int, int>& b) {
        return (a.first == b.first && a.second == b.second)
            || (a.first == b.second && a.second == b.first);
    }

public:
    explicit KeyposeGraph(ros::NodeHandle& nh);
    ~KeyposeGraph() = default;
    void ReadParameters(ros::NodeHandle& nh);
    void AddNode(const geometry_msgs::Point& position, int node_ind, int keypose_id,
                 bool is_keypose);
    void AddNodeAndEdge(const geometry_msgs::Point& position, int node_ind, int keypose_id,
                        bool is_keypose, int connected_node_ind, double connected_node_dist);
    void AddEdge(int from_node_ind, int to_node_ind, double dist);
    bool HasNode(const Eigen::Vector3d& position);
    bool InBound(const int index) const {
        return index >= 0 && index < graph_.size();
    }
    int GetNodeNum() const {
        return nodes_.size();
    }
    int GetConnectedNodeNum() const;
    void GetMarker(visualization_msgs::Marker& node_marker,
                   visualization_msgs::Marker& edge_marker) const;
    void GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) const;
    std::vector<int> GetConnectedGraphNodeIndices() {
        return connected_node_indices_;
    }
    void GetConnectedNodeIndices(int query_ind, std::vector<int>& connected_node_indices,
                                 std::vector<bool> constraints) const;
    void CheckLocalCollision(
        const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager);
    void UpdateNodes() const;
    void CheckConnectivity(const geometry_msgs::Point& robot_position);
    int AddKeyposeNode(const nav_msgs::Odometry& keypose,
                       const planning_env_ns::PlanningEnv& planning_env);
    bool HasEdgeBetween(int node_ind1, int node_ind2);
    bool IsConnected(const Eigen::Vector3d& from_position, const Eigen::Vector3d& to_position);
    int AddNonKeyposeNode(const geometry_msgs::Point& new_node_position);
    void AddPath(const nav_msgs::Path& path);
    void SetAllowVerticalEdge(const bool allow_vertical_edge) {
        allow_vertical_edge_ = allow_vertical_edge;
    }
    bool IsPositionReachable(const geometry_msgs::Point& point, double dist_threshold) const;
    bool IsPositionReachable(const geometry_msgs::Point& point) const;
    int GetClosestNodeInd(const geometry_msgs::Point& point) const;
    void GetClosestNodeIndAndDistance(const geometry_msgs::Point& point, int& node_ind,
                                      double& dist) const;
    void GetClosestConnectedNodeIndAndDistance(const geometry_msgs::Point& point, int& node_ind,
                                               double& dist) const;
    int GetClosestKeyposeID(const geometry_msgs::Point& point) const;
    geometry_msgs::Point GetClosestNodePosition(const geometry_msgs::Point& point) const;
    bool GetShortestPathWithMaxLength(const geometry_msgs::Point& start_point,
                                      const geometry_msgs::Point& target_point,
                                      double max_path_length, bool get_path,
                                      nav_msgs::Path& path) const;
    double GetShortestPath(const geometry_msgs::Point& start_point,
                           const geometry_msgs::Point& target_point, bool get_path,
                           nav_msgs::Path& path, bool use_connected_nodes = false) const;

    // 设置参数
    double& SetAddNodeMinDist() {
        return kAddNodeMinDist;
    }
    double& SetAddNonKeyposeNodeMinDist() {
        return kAddNonKeyposeNodeMinDist;
    }
    double& SetAddEdgeCollisionCheckResolution() {
        return kAddEdgeCollisionCheckResolution;
    }
    double& SetAddEdgeCollisionCheckRadius() {
        return kAddEdgeCollisionCheckRadius;
    }
    int& SetAddEdgeCollisionCheckPointNumThr() {
        return kAddEdgeCollisionCheckPointNumThr;
    }
    double& SetAddEdgeConnectDistThr() {
        return kAddEdgeConnectDistThr;
    }
    double& SetAddEdgeToLastKeyposeDistThr() {
        return kAddEdgeToLastKeyposeDistThr;
    }
    double& SetAddEdgeVerticalThreshold() {
        return kAddEdgeVerticalThreshold;
    }
    // helper function
    geometry_msgs::Point GetFirstKeyposePosition() const;
    geometry_msgs::Point GetKeyposePosition(int keypose_id) const;
    void GetKeyposePositions(std::vector<Eigen::Vector3d>& positions) const;
    geometry_msgs::Point GetNodePosition(int node_ind) const;
};

#endif  // SENSOR_COVERAGE_PLANNER_KEYPOSE_GRAPH_H

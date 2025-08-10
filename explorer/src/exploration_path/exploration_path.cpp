/**
 * @file exploration_path.cpp
 * @author joosoo (joosoo@buct.edu.cn)
 * @brief Class that implements the exploration path
 * @version 0.1
 * @date 2020-10-22
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "exploration_path/exploration_path.h"

#include <utility>

namespace exploration_path_ns
{
    Node::Node() :
        type_(NodeType::LOCAL_VIA_POINT), position_(Eigen::Vector3d::Zero()), local_viewpoint_ind_(-1),
        keypose_graph_node_ind_(-1), global_subspace_index_(-1), nonstop_(false)
    {
    }

    Node::Node(Eigen::Vector3d position) : Node() { position_ = std::move(position); }

    Node::Node(const geometry_msgs::Point& point, const NodeType type) :
        Node(Eigen::Vector3d(point.x, point.y, point.z))
    {
        type_ = type;
    }

    bool Node::IsLocal()
    {
        const int node_type = static_cast<int>(type_);
        return node_type % 2 == 0;
    }

    bool operator==(const Node& n1, const Node& n2)
    {
        return ((n1.position_ - n2.position_).norm() < 0.2) && (n1.type_ == n2.type_);
    }

    bool operator!=(const Node& n1, const Node& n2) { return !(n1 == n2); }

    double ExplorationPath::GetLength() const
    {
        double length = 0.0;
        if (nodes_.size() < 2)
        {
            return length;
        }
        for (int i = 1; i < nodes_.size(); i++)
        {
            length += (nodes_[i].position_ - nodes_[i - 1].position_).norm();
        }
        return length;
    }

    void ExplorationPath::Append(const Node& node)
    {
        if (nodes_.empty() || nodes_.back() != node)
        {
            nodes_.push_back(node);
        }
    }

    void ExplorationPath::Append(const ExplorationPath& path)
    {
        for (const auto& node : path.nodes_)
        {
            Append(node);
        }
    }

    void ExplorationPath::Reverse() { std::reverse(nodes_.begin(), nodes_.end()); }

    nav_msgs::Path ExplorationPath::GetPath() const
    {
        nav_msgs::Path path;
        for (const auto& node : nodes_)
        {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = node.position_.x();
            pose.pose.position.y = node.position_.y();
            pose.pose.position.z = node.position_.z();
            // pose.pose.orientation.w = static_cast<int>(nodes_[i].type_);
            path.poses.push_back(pose);
        }
        return path;
    }

    void ExplorationPath::FromPath(const nav_msgs::Path& path)
    {
        nodes_.clear();
        for (const auto& pose : path.poses)
        {
            Node node;
            node.position_.x() = pose.pose.position.x;
            node.position_.y() = pose.pose.position.y;
            node.position_.z() = pose.pose.position.z;
            node.type_ = static_cast<NodeType>(pose.pose.orientation.w);
            nodes_.push_back(node);
        }
    }

    void ExplorationPath::GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr vis_cloud) const
    {
        vis_cloud->clear();
        for (const auto& node : nodes_)
        {
            pcl::PointXYZI point;
            point.x = node.position_.x();
            point.y = node.position_.y();
            point.z = node.position_.z();
            point.intensity = static_cast<int>(node.type_);
            vis_cloud->points.push_back(point);
        }
    }

    void ExplorationPath::GetKeyPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr vis_cloud) const
    {
        vis_cloud->clear();
        for (const auto& node : nodes_)
        {
            if (node.type_ == NodeType::ROBOT || node.type_ == NodeType::LOOKAHEAD_POINT ||
                node.type_ == NodeType::LOCAL_VIEWPOINT || node.type_ == NodeType::LOCAL_PATH_START ||
                node.type_ == NodeType::LOCAL_PATH_END || node.type_ == NodeType::GLOBAL_VIEWPOINT)
            {
                pcl::PointXYZI point;
                point.x = node.position_.x();
                point.y = node.position_.y();
                point.z = node.position_.z();
                point.intensity = static_cast<int>(node.type_);
                vis_cloud->points.push_back(point);
            }
        }
    }

    void ExplorationPath::GetNodePositions(std::vector<Eigen::Vector3d>& positions) const
    {
        positions.clear();
        for (const auto& node : nodes_)
        {
            positions.push_back(node.position_);
        }
    }

    void ExplorationPath::Reset() { nodes_.clear(); }
} // namespace exploration_path_ns

/**
 * @file grid_world_comprehensive_test.cpp
 * @brief Comprehensive unit tests for GridWorld class
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <grid_world/grid_world.h>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class GridWorldComprehensiveTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize ROS node for testing
        if (!ros::isInitialized()) {
            ros::init(ros::M_string(), "grid_world_comprehensive_test");
        }
        nh_ = std::make_unique<ros::NodeHandle>();
        
        // Create grid world instance with test parameters
        grid_world_ = std::make_unique<grid_world_ns::GridWorld>(5, 5, 3, 6.0, 6.0, 2);
        
        // Set test robot position
        robot_position_.x = 10.0;
        robot_position_.y = 10.0;
        robot_position_.z = 0.0;
    }

    void TearDown() override {
        grid_world_.reset();
        nh_.reset();
    }

    std::unique_ptr<ros::NodeHandle> nh_;
    std::unique_ptr<grid_world_ns::GridWorld> grid_world_;
    geometry_msgs::Point robot_position_;
};

// Test constructor and initialization
TEST_F(GridWorldComprehensiveTest, ConstructorTest) {
    EXPECT_NE(grid_world_, nullptr);
    EXPECT_TRUE(grid_world_->Initialized());
}

// Test grid coordinate conversion
TEST_F(GridWorldComprehensiveTest, CoordinateConversionTest) {
    // Test coordinate conversion functions
    int cell_ind = grid_world_->GetCellInd(robot_position_.x, robot_position_.y, robot_position_.z);
    EXPECT_GE(cell_ind, 0);
    
    // Test index to coordinate conversion
    geometry_msgs::Point retrieved_position = grid_world_->GetCellPosition(cell_ind);
    EXPECT_GE(retrieved_position.x, 0.0);
    EXPECT_GE(retrieved_position.y, 0.0);
    EXPECT_GE(retrieved_position.z, 0.0);
}

// Test grid subdivision conversion
TEST_F(GridWorldComprehensiveTest, SubdivisionConversionTest) {
    // Test subdivision index conversion
    Eigen::Vector3i sub = grid_world_->GetCellSub(Eigen::Vector3d(robot_position_.x, robot_position_.y, robot_position_.z));
    EXPECT_GE(sub.x(), 0);
    EXPECT_GE(sub.y(), 0);
    EXPECT_GE(sub.z(), 0);
    
    // Test subdivision to index conversion
    int ind_from_sub = grid_world_->sub2ind(sub);
    EXPECT_GE(ind_from_sub, 0);
    
    // Test index to subdivision conversion
    Eigen::Vector3i sub_from_ind = grid_world_->ind2sub(ind_from_sub);
    EXPECT_EQ(sub_from_ind.x(), sub.x());
    EXPECT_EQ(sub_from_ind.y(), sub.y());
    EXPECT_EQ(sub_from_ind.z(), sub.z());
}

// Test boundary checking
TEST_F(GridWorldComprehensiveTest, BoundaryCheckingTest) {
    // Test subdivision boundary checking
    Eigen::Vector3i valid_sub(2, 2, 1);
    Eigen::Vector3i invalid_sub(10, 10, 10);
    
    EXPECT_TRUE(grid_world_->SubInBound(valid_sub));
    EXPECT_FALSE(grid_world_->SubInBound(invalid_sub));
    
    // Test index boundary checking
    int valid_ind = grid_world_->sub2ind(valid_sub);
    int invalid_ind = 1000;
    
    EXPECT_TRUE(grid_world_->IndInBound(valid_ind));
    EXPECT_FALSE(grid_world_->IndInBound(invalid_ind));
}

// Test robot position update
TEST_F(GridWorldComprehensiveTest, RobotPositionUpdateTest) {
    // Test robot position update
    grid_world_->UpdateRobotPosition(robot_position_);
    
    // Test neighbor cell update
    EXPECT_NO_THROW(grid_world_->UpdateNeighborCells(robot_position_));
}

// Test cell status management
TEST_F(GridWorldComprehensiveTest, CellStatusTest) {
    int cell_ind = grid_world_->GetCellInd(robot_position_.x, robot_position_.y, robot_position_.z);
    
    // Test initial status
    grid_world_ns::CellStatus initial_status = grid_world_->GetCellStatus(cell_ind);
    EXPECT_EQ(initial_status, grid_world_ns::CellStatus::UNSEEN);
    
    // Test status setting
    grid_world_->SetCellStatus(cell_ind, grid_world_ns::CellStatus::EXPLORING);
    grid_world_ns::CellStatus updated_status = grid_world_->GetCellStatus(cell_ind);
    EXPECT_EQ(updated_status, grid_world_ns::CellStatus::EXPLORING);
    
    // Test status counting
    int exploring_count = grid_world_->GetCellStatusCount(grid_world_ns::CellStatus::EXPLORING);
    EXPECT_GE(exploring_count, 1);
}

// Test viewpoint management
TEST_F(GridWorldComprehensiveTest, ViewPointManagementTest) {
    int cell_ind = grid_world_->GetCellInd(robot_position_.x, robot_position_.y, robot_position_.z);
    
    // Test viewpoint addition
    grid_world_->AddViewPointToCell(cell_ind, 0);
    grid_world_->AddViewPointToCell(cell_ind, 1);
    
    std::vector<int> viewpoint_indices = grid_world_->GetCellViewPointIndices(cell_ind);
    EXPECT_EQ(viewpoint_indices.size(), 2);
    
    // Test viewpoint clearing
    grid_world_->ClearCellViewPointIndices(cell_ind);
    viewpoint_indices = grid_world_->GetCellViewPointIndices(cell_ind);
    EXPECT_EQ(viewpoint_indices.size(), 0);
}

// Test graph node management
TEST_F(GridWorldComprehensiveTest, GraphNodeManagementTest) {
    int cell_ind = grid_world_->GetCellInd(robot_position_.x, robot_position_.y, robot_position_.z);
    
    // Test graph node addition
    grid_world_->AddGraphNodeToCell(cell_ind, 0);
    grid_world_->AddGraphNodeToCell(cell_ind, 1);
    
    // Note: We can't directly access graph node indices, but we can test the function doesn't crash
    EXPECT_NO_THROW(grid_world_->ClearCellViewPointIndices(cell_ind));
}

// Test robot position in cell
TEST_F(GridWorldComprehensiveTest, RobotPositionInCellTest) {
    int cell_ind = grid_world_->GetCellInd(robot_position_.x, robot_position_.y, robot_position_.z);
    
    // Test robot position setting
    grid_world_->SetCellRobotPosition(cell_ind, robot_position_);
    geometry_msgs::Point retrieved_position = grid_world_->GetCellRobotPosition(cell_ind);
    
    EXPECT_EQ(retrieved_position.x, robot_position_.x);
    EXPECT_EQ(retrieved_position.y, robot_position_.y);
    EXPECT_EQ(retrieved_position.z, robot_position_.z);
    
    // Test robot position set flag
    EXPECT_TRUE(grid_world_->IsRobotPositionSet(cell_ind));
}

// Test visit count management
TEST_F(GridWorldComprehensiveTest, VisitCountTest) {
    int cell_ind = grid_world_->GetCellInd(robot_position_.x, robot_position_.y, robot_position_.z);
    
    // Test visit count
    int initial_count = grid_world_->GetCellVisitCount(cell_ind);
    EXPECT_EQ(initial_count, 0);
    
    grid_world_->CellAddVisitCount(cell_ind);
    grid_world_->CellAddVisitCount(cell_ind);
    
    int updated_count = grid_world_->GetCellVisitCount(cell_ind);
    EXPECT_EQ(updated_count, 2);
}

// Test neighbor cell management
TEST_F(GridWorldComprehensiveTest, NeighborCellTest) {
    // Test neighbor cell retrieval
    std::vector<int> neighbor_indices = grid_world_->GetNeighborCellIndices();
    EXPECT_GT(neighbor_indices.size(), 0);
    
    // Test range-based neighbor retrieval
    Eigen::Vector3i center_sub = grid_world_->GetCellSub(Eigen::Vector3d(robot_position_.x, robot_position_.y, robot_position_.z));
    Eigen::Vector3i neighbor_range(1, 1, 1);
    std::vector<int> range_neighbor_indices;
    
    grid_world_->GetNeighborCellIndices(center_sub, neighbor_range, range_neighbor_indices);
    EXPECT_GT(range_neighbor_indices.size(), 0);
}

// Test exploring cell management
TEST_F(GridWorldComprehensiveTest, ExploringCellTest) {
    // Set some cells to exploring state
    int cell_ind1 = grid_world_->GetCellInd(robot_position_.x, robot_position_.y, robot_position_.z);
    int cell_ind2 = grid_world_->GetCellInd(robot_position_.x + 6.0, robot_position_.y, robot_position_.z);
    
    grid_world_->SetCellStatus(cell_ind1, grid_world_ns::CellStatus::EXPLORING);
    grid_world_->SetCellStatus(cell_ind2, grid_world_ns::CellStatus::EXPLORING);
    
    // Test exploring cell retrieval
    std::vector<int> exploring_cells;
    grid_world_->GetExploringCellIndices(exploring_cells);
    EXPECT_GE(exploring_cells.size(), 2);
}

// Test keypose ID management
TEST_F(GridWorldComprehensiveTest, KeyposeIDTest) {
    int cell_ind = grid_world_->GetCellInd(robot_position_.x, robot_position_.y, robot_position_.z);
    
    // Test keypose ID setting
    grid_world_->SetCellRobotPosition(cell_ind, robot_position_);
    grid_world_->SetCurKeyposeID(5);
    
    int keypose_id = grid_world_->GetCellKeyposeID(cell_ind);
    EXPECT_EQ(keypose_id, 5);
}

// Test keypose graph settings
TEST_F(GridWorldComprehensiveTest, KeyposeGraphTest) {
    // Test keypose graph usage settings
    grid_world_->SetUseKeyposeGraph(true);
    EXPECT_TRUE(grid_world_->UseKeyposeGraph());
    
    grid_world_->SetUseKeyposeGraph(false);
    EXPECT_FALSE(grid_world_->UseKeyposeGraph());
}

// Test keypose graph node management
TEST_F(GridWorldComprehensiveTest, KeyposeGraphNodeTest) {
    // Test keypose graph node index setting
    grid_world_->SetCurKeyposeGraphNodeInd(10);
    EXPECT_NO_THROW(grid_world_->UpdateCellKeyposeGraphNodes(nullptr));
}

// Test keypose graph node position
TEST_F(GridWorldComprehensiveTest, KeyposeGraphNodePositionTest) {
    // Test keypose graph node position setting
    geometry_msgs::Point node_position;
    node_position.x = 5.0;
    node_position.y = 5.0;
    node_position.z = 0.0;
    
    grid_world_->SetCurKeyposeGraphNodePosition(node_position);
    EXPECT_NO_THROW(grid_world_->UpdateCellKeyposeGraphNodes(nullptr));
}

// Test home position management
TEST_F(GridWorldComprehensiveTest, HomePositionTest) {
    // Test home position setting
    Eigen::Vector3d home_position(0.0, 0.0, 0.0);
    grid_world_->SetHomePosition(home_position);
    
    EXPECT_TRUE(grid_world_->HomeSet());
    EXPECT_FALSE(grid_world_->IsReturningHome());
}

// Test minimum add point numbers
TEST_F(GridWorldComprehensiveTest, MinimumAddPointTest) {
    // Test minimum add point numbers
    int min_point_num = grid_world_->GetMinAddPointNum();
    int min_frontier_point_num = grid_world_->GetMinAddFrontierPointNum();
    
    EXPECT_GT(min_point_num, 0);
    EXPECT_GT(min_frontier_point_num, 0);
}

// Test cell neighbor checking
TEST_F(GridWorldComprehensiveTest, CellNeighborTest) {
    int cell_ind1 = grid_world_->GetCellInd(robot_position_.x, robot_position_.y, robot_position_.z);
    int cell_ind2 = grid_world_->GetCellInd(robot_position_.x + 6.0, robot_position_.y, robot_position_.z);
    
    // Test neighbor checking
    bool are_neighbors = grid_world_->AreNeighbors(cell_ind1, cell_ind2);
    EXPECT_TRUE(are_neighbors);
}

// Test grid world reset
TEST_F(GridWorldComprehensiveTest, GridWorldResetTest) {
    // Set some state
    int cell_ind = grid_world_->GetCellInd(robot_position_.x, robot_position_.y, robot_position_.z);
    grid_world_->SetCellStatus(cell_ind, grid_world_ns::CellStatus::EXPLORING);
    grid_world_->CellAddVisitCount(cell_ind);
    grid_world_->SetCellRobotPosition(cell_ind, robot_position_);
    
    // Reset grid world
    grid_world_->Reset();
    
    // Check reset state
    grid_world_ns::CellStatus reset_status = grid_world_->GetCellStatus(cell_ind);
    int reset_count = grid_world_->GetCellVisitCount(cell_ind);
    
    EXPECT_EQ(reset_status, grid_world_ns::CellStatus::UNSEEN);
    EXPECT_EQ(reset_count, 0);
}

// Test origin retrieval
TEST_F(GridWorldComprehensiveTest, OriginRetrievalTest) {
    // Test origin retrieval
    geometry_msgs::Point origin = grid_world_->GetOrigin();
    EXPECT_EQ(origin.x, 0.0);
    EXPECT_EQ(origin.y, 0.0);
    EXPECT_EQ(origin.z, 0.0);
}

// Test current keypose setting
TEST_F(GridWorldComprehensiveTest, CurrentKeyposeTest) {
    // Test current keypose setting
    Eigen::Vector3d cur_keypose(1.0, 2.0, 0.0);
    grid_world_->SetCurKeypose(cur_keypose);
    
    // This is a smoke test - just ensure it doesn't crash
    EXPECT_TRUE(true);
}

// Test viewpoint position retrieval
TEST_F(GridWorldComprehensiveTest, ViewPointPositionTest) {
    // Test viewpoint position retrieval
    std::vector<Eigen::Vector3d> viewpoint_positions;
    grid_world_->GetCellViewPointPositions(viewpoint_positions);
    
    // Should return empty vector if no viewpoints set
    EXPECT_EQ(viewpoint_positions.size(), 0);
}

// Test cell status update with viewpoint manager
TEST_F(GridWorldComprehensiveTest, CellStatusUpdateTest) {
    // Test cell status update with viewpoint manager
    // This is a smoke test since we can't easily create a viewpoint manager
    std::shared_ptr<viewpoint_manager_ns::ViewPointManager> viewpoint_manager = nullptr;
    
    EXPECT_NO_THROW(grid_world_->UpdateCellStatus(viewpoint_manager));
}

// Test global TSP solving
TEST_F(GridWorldComprehensiveTest, GlobalTSPTest) {
    // Test global TSP solving
    std::shared_ptr<viewpoint_manager_ns::ViewPointManager> viewpoint_manager = nullptr;
    std::vector<int> ordered_cell_indices;
    
    exploration_path_ns::ExplorationPath path = grid_world_->SolveGlobalTSP(viewpoint_manager, ordered_cell_indices);
    
    // Should return empty path if no viewpoint manager
    EXPECT_EQ(path.nodes_.size(), 0);
}

// Test visualization functions
TEST_F(GridWorldComprehensiveTest, VisualizationTest) {
    // Test visualization marker
    visualization_msgs::Marker marker;
    EXPECT_NO_THROW(grid_world_->GetMarker(marker));
    
    // Test visualization cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr vis_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    EXPECT_NO_THROW(grid_world_->GetVisualizationCloud(vis_cloud));
}

// Test semi-explored cell functions
TEST_F(GridWorldComprehensiveTest, SemiExploredCellTest) {
    // Test semi-explored cell transition probability
    int cell_ind = grid_world_->GetCellInd(robot_position_.x, robot_position_.y, robot_position_.z);
    
    double transition_prob = grid_world_->GetSemiCellTransitionProbability(cell_ind);
    EXPECT_GE(transition_prob, 0.0);
    EXPECT_LE(transition_prob, 1.0);
    
    // Test semi-explored cell indices
    std::vector<Eigen::Vector3d> semi_dynamic_frontier_positions;
    semi_dynamic_frontier_positions.push_back(Eigen::Vector3d(1.0, 1.0, 0.0));
    
    EXPECT_NO_THROW(grid_world_->getSemiExploredCellindices(semi_dynamic_frontier_positions));
}

// Test path validity checking
TEST_F(GridWorldComprehensiveTest, PathValidityTest) {
    int from_cell_ind = grid_world_->GetCellInd(robot_position_.x, robot_position_.y, robot_position_.z);
    int to_cell_ind = grid_world_->GetCellInd(robot_position_.x + 6.0, robot_position_.y, robot_position_.z);
    
    nav_msgs::Path test_path;
    test_path.header.frame_id = "map";
    
    geometry_msgs::PoseStamped pose;
    pose.pose.position = robot_position_;
    test_path.poses.push_back(pose);
    
    pose.pose.position.x += 6.0;
    test_path.poses.push_back(pose);
    
    bool path_valid = grid_world_->PathValid(test_path, from_cell_ind, to_cell_ind);
    EXPECT_TRUE(path_valid);
}

// Test direct keypose graph connection
TEST_F(GridWorldComprehensiveTest, DirectKeyposeGraphConnectionTest) {
    // Test direct keypose graph connection
    std::unique_ptr<keypose_graph_ns::KeyposeGraph> keypose_graph = nullptr;
    Eigen::Vector3d start_position(robot_position_.x, robot_position_.y, robot_position_.z);
    Eigen::Vector3d goal_position(robot_position_.x + 6.0, robot_position_.y, robot_position_.z);
    
    bool has_connection = grid_world_->HasDirectKeyposeGraphConnection(keypose_graph, start_position, goal_position);
    EXPECT_FALSE(has_connection); // Should be false with null keypose graph
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
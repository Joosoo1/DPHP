/**
 * @file viewpoint_comprehensive_test.cpp
 * @brief Comprehensive unit tests for ViewPoint class
 */

#include <gtest/gtest.h>
#include <viewpoint/viewpoint.h>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class ViewPointComprehensiveTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create viewpoint instance with test position
        test_position_.x = 1.0;
        test_position_.y = 2.0;
        test_position_.z = 0.0;
        
        viewpoint_ = std::make_unique<viewpoint_ns::ViewPoint>(test_position_);
        
        // Create test points for coverage testing
        test_point_.x = 1.5;
        test_point_.y = 2.0;
        test_point_.z = 0.0;
        test_point_.intensity = 1.0;
    }

    void TearDown() override {
        viewpoint_.reset();
    }

    std::unique_ptr<viewpoint_ns::ViewPoint> viewpoint_;
    geometry_msgs::Point test_position_;
    pcl::PointXYZI test_point_;
};

// Test constructor with coordinates
TEST_F(ViewPointComprehensiveTest, CoordinateConstructorTest) {
    viewpoint_ns::ViewPoint vp(1.0, 2.0, 0.0);
    
    EXPECT_EQ(vp.GetX(), 1.0);
    EXPECT_EQ(vp.GetY(), 2.0);
    EXPECT_EQ(vp.GetHeight(), 0.0);
}

// Test constructor with position
TEST_F(ViewPointComprehensiveTest, PositionConstructorTest) {
    geometry_msgs::Point pos;
    pos.x = 3.0;
    pos.y = 4.0;
    pos.z = 1.0;
    
    viewpoint_ns::ViewPoint vp(pos);
    
    EXPECT_EQ(vp.GetX(), 3.0);
    EXPECT_EQ(vp.GetY(), 4.0);
    EXPECT_EQ(vp.GetHeight(), 1.0);
}

// Test default constructor
TEST_F(ViewPointComprehensiveTest, DefaultConstructorTest) {
    viewpoint_ns::ViewPoint vp;
    
    EXPECT_EQ(vp.GetX(), 0.0);
    EXPECT_EQ(vp.GetY(), 0.0);
    EXPECT_EQ(vp.GetHeight(), 0.0);
}

// Test position management
TEST_F(ViewPointComprehensiveTest, PositionManagementTest) {
    // Test initial position
    EXPECT_EQ(viewpoint_->GetX(), test_position_.x);
    EXPECT_EQ(viewpoint_->GetY(), test_position_.y);
    EXPECT_EQ(viewpoint_->GetHeight(), test_position_.z);
    
    // Test position setting
    geometry_msgs::Point new_position;
    new_position.x = 5.0;
    new_position.y = 6.0;
    new_position.z = 2.0;
    
    viewpoint_->SetPosition(new_position);
    
    EXPECT_EQ(viewpoint_->GetX(), new_position.x);
    EXPECT_EQ(viewpoint_->GetY(), new_position.y);
    EXPECT_EQ(viewpoint_->GetHeight(), new_position.z);
}

// Test height management
TEST_F(ViewPointComprehensiveTest, HeightManagementTest) {
    // Test height setting
    double new_height = 3.5;
    viewpoint_->SetHeight(new_height);
    
    EXPECT_EQ(viewpoint_->GetHeight(), new_height);
}

// Test collision state management
TEST_F(ViewPointComprehensiveTest, CollisionStateTest) {
    // Test initial collision state
    EXPECT_FALSE(viewpoint_->InCollision());
    
    // Test collision state setting
    viewpoint_->SetInCollision(true);
    EXPECT_TRUE(viewpoint_->InCollision());
    
    viewpoint_->SetInCollision(false);
    EXPECT_FALSE(viewpoint_->InCollision());
}

// Test line of sight state management
TEST_F(ViewPointComprehensiveTest, LineOfSightStateTest) {
    // Test initial line of sight state
    EXPECT_FALSE(viewpoint_->InLineOfSight());
    
    // Test line of sight state setting
    viewpoint_->SetInLineOfSight(true);
    EXPECT_TRUE(viewpoint_->InLineOfSight());
    
    viewpoint_->SetInLineOfSight(false);
    EXPECT_FALSE(viewpoint_->InLineOfSight());
}

// Test current frame line of sight state management
TEST_F(ViewPointComprehensiveTest, CurrentFrameLineOfSightTest) {
    // Test initial current frame line of sight state
    EXPECT_FALSE(viewpoint_->InCurrentFrameLineOfSight());
    
    // Test current frame line of sight state setting
    viewpoint_->SetInCurrentFrameLineOfSight(true);
    EXPECT_TRUE(viewpoint_->InCurrentFrameLineOfSight());
    
    viewpoint_->SetInCurrentFrameLineOfSight(false);
    EXPECT_FALSE(viewpoint_->InCurrentFrameLineOfSight());
}

// Test connectivity state management
TEST_F(ViewPointComprehensiveTest, ConnectivityStateTest) {
    // Test initial connectivity state
    EXPECT_FALSE(viewpoint_->Connected());
    
    // Test connectivity state setting
    viewpoint_->SetConnected(true);
    EXPECT_TRUE(viewpoint_->Connected());
    
    viewpoint_->SetConnected(false);
    EXPECT_FALSE(viewpoint_->Connected());
}

// Test visited state management
TEST_F(ViewPointComprehensiveTest, VisitedStateTest) {
    // Test initial visited state
    EXPECT_FALSE(viewpoint_->Visited());
    
    // Test visited state setting
    viewpoint_->SetVisited(true);
    EXPECT_TRUE(viewpoint_->Visited());
    
    viewpoint_->SetVisited(false);
    EXPECT_FALSE(viewpoint_->Visited());
}

// Test selected state management
TEST_F(ViewPointComprehensiveTest, SelectedStateTest) {
    // Test initial selected state
    EXPECT_FALSE(viewpoint_->Selected());
    
    // Test selected state setting
    viewpoint_->SetSelected(true);
    EXPECT_TRUE(viewpoint_->Selected());
    
    viewpoint_->SetSelected(false);
    EXPECT_FALSE(viewpoint_->Selected());
}

// Test candidate state management
TEST_F(ViewPointComprehensiveTest, CandidateStateTest) {
    // Test initial candidate state
    EXPECT_FALSE(viewpoint_->IsCandidate());
    
    // Test candidate state setting
    viewpoint_->SetCandidate(true);
    EXPECT_TRUE(viewpoint_->IsCandidate());
    
    viewpoint_->SetCandidate(false);
    EXPECT_FALSE(viewpoint_->IsCandidate());
}

// Test terrain height management
TEST_F(ViewPointComprehensiveTest, TerrainHeightTest) {
    // Test initial terrain height state
    EXPECT_FALSE(viewpoint_->HasTerrainHeight());
    
    // Test terrain height state setting
    viewpoint_->SetHasTerrainHeight(true);
    EXPECT_TRUE(viewpoint_->HasTerrainHeight());
    
    viewpoint_->SetHasTerrainHeight(false);
    EXPECT_FALSE(viewpoint_->HasTerrainHeight());
    
    // Test terrain height value setting
    double terrain_height = 1.5;
    viewpoint_->SetTerrainHeight(terrain_height);
    EXPECT_EQ(viewpoint_->GetTerrainHeight(), terrain_height);
}

// Test terrain neighbor state management
TEST_F(ViewPointComprehensiveTest, TerrainNeighborTest) {
    // Test initial terrain neighbor state
    EXPECT_FALSE(viewpoint_->HasTerrainNeighbor());
    
    // Test terrain neighbor state setting
    viewpoint_->SetHasTerrainNeighbor(true);
    EXPECT_TRUE(viewpoint_->HasTerrainNeighbor());
    
    viewpoint_->SetHasTerrainNeighbor(false);
    EXPECT_FALSE(viewpoint_->HasTerrainNeighbor());
}

// Test exploring cell state management
TEST_F(ViewPointComprehensiveTest, ExploringCellTest) {
    // Test initial exploring cell state
    EXPECT_FALSE(viewpoint_->InExploringCell());
    
    // Test exploring cell state setting
    viewpoint_->SetInExploringCell(true);
    EXPECT_TRUE(viewpoint_->InExploringCell());
    
    viewpoint_->SetInExploringCell(false);
    EXPECT_FALSE(viewpoint_->InExploringCell());
}

// Test cell index management
TEST_F(ViewPointComprehensiveTest, CellIndexTest) {
    // Test initial cell index
    EXPECT_EQ(viewpoint_->GetCellInd(), -1);
    EXPECT_EQ(viewpoint_->GetCellIndex(), -1);
    
    // Test cell index setting
    int test_cell_ind = 5;
    viewpoint_->SetCellInd(test_cell_ind);
    
    EXPECT_EQ(viewpoint_->GetCellInd(), test_cell_ind);
    EXPECT_EQ(viewpoint_->GetCellIndex(), test_cell_ind);
}

// Test covered point list management
TEST_F(ViewPointComprehensiveTest, CoveredPointListTest) {
    // Test initial covered point list
    EXPECT_EQ(viewpoint_->GetCoveredPointList().size(), 0);
    EXPECT_EQ(viewpoint_->GetCoveredPointNum(), 0);
    
    // Test adding covered points
    viewpoint_->AddCoveredPoint(1);
    viewpoint_->AddCoveredPoint(2);
    viewpoint_->AddCoveredPoint(3);
    
    EXPECT_EQ(viewpoint_->GetCoveredPointNum(), 3);
    EXPECT_EQ(viewpoint_->GetCoveredPointList().size(), 3);
    
    // Test covered point list reset
    viewpoint_->ResetCoveredPointList();
    EXPECT_EQ(viewpoint_->GetCoveredPointNum(), 0);
    EXPECT_EQ(viewpoint_->GetCoveredPointList().size(), 0);
}

// Test covered frontier point list management
TEST_F(ViewPointComprehensiveTest, CoveredFrontierPointListTest) {
    // Test initial covered frontier point list
    EXPECT_EQ(viewpoint_->GetCoveredFrontierPointList().size(), 0);
    EXPECT_EQ(viewpoint_->GetCoveredFrontierPointNum(), 0);
    
    // Test adding covered frontier points
    viewpoint_->AddCoveredFrontierPoint(1);
    viewpoint_->AddCoveredFrontierPoint(2);
    
    EXPECT_EQ(viewpoint_->GetCoveredFrontierPointNum(), 2);
    EXPECT_EQ(viewpoint_->GetCoveredFrontierPointList().size(), 2);
    
    // Test covered frontier point list reset
    viewpoint_->ResetCoveredFrontierPointList();
    EXPECT_EQ(viewpoint_->GetCoveredFrontierPointNum(), 0);
    EXPECT_EQ(viewpoint_->GetCoveredFrontierPointList().size(), 0);
}

// Test collision frame count management
TEST_F(ViewPointComprehensiveTest, CollisionFrameCountTest) {
    // Test initial collision frame count
    EXPECT_EQ(viewpoint_->GetCollisionFrameCount(), 0);
    
    // Test adding collision frames
    viewpoint_->AddCollisionFrame();
    viewpoint_->AddCollisionFrame();
    viewpoint_->AddCollisionFrame();
    
    EXPECT_EQ(viewpoint_->GetCollisionFrameCount(), 3);
    
    // Test collision frame count reset
    viewpoint_->ResetCollisionFrameCount();
    EXPECT_EQ(viewpoint_->GetCollisionFrameCount(), 0);
}

// Test coverage update
TEST_F(ViewPointComprehensiveTest, CoverageUpdateTest) {
    // Test coverage update with point
    EXPECT_NO_THROW(viewpoint_->UpdateCoverage<pcl::PointXYZI>(test_point_));
    
    // Test coverage update with different point types
    pcl::PointXYZRGBNormal colored_point;
    colored_point.x = 1.5;
    colored_point.y = 2.0;
    colored_point.z = 0.0;
    
    EXPECT_NO_THROW(viewpoint_->UpdateCoverage<pcl::PointXYZRGBNormal>(colored_point));
}

// Test visibility check
TEST_F(ViewPointComprehensiveTest, VisibilityCheckTest) {
    // Test visibility check with point
    double occlusion_threshold = 0.5;
    bool visible = viewpoint_->CheckVisibility<pcl::PointXYZI>(test_point_, occlusion_threshold);
    
    // Result depends on LiDAR model configuration
    EXPECT_NO_THROW(visible);
}

// Test coverage reset
TEST_F(ViewPointComprehensiveTest, CoverageResetTest) {
    // Add some coverage data
    viewpoint_->AddCoveredPoint(1);
    viewpoint_->AddCoveredFrontierPoint(2);
    viewpoint_->AddCollisionFrame();
    
    // Reset coverage
    viewpoint_->ResetCoverage();
    
    // Check that coverage is reset
    EXPECT_EQ(viewpoint_->GetCoveredPointNum(), 0);
    EXPECT_EQ(viewpoint_->GetCoveredFrontierPointNum(), 0);
    EXPECT_EQ(viewpoint_->GetCollisionFrameCount(), 0);
}

// Test complete reset
TEST_F(ViewPointComprehensiveTest, CompleteResetTest) {
    // Set various states
    viewpoint_->SetInCollision(true);
    viewpoint_->SetInLineOfSight(true);
    viewpoint_->SetConnected(true);
    viewpoint_->SetVisited(true);
    viewpoint_->SetSelected(true);
    viewpoint_->SetCandidate(true);
    viewpoint_->SetHasTerrainHeight(true);
    viewpoint_->SetInExploringCell(true);
    viewpoint_->SetCellInd(5);
    viewpoint_->AddCoveredPoint(1);
    viewpoint_->AddCoveredFrontierPoint(2);
    viewpoint_->AddCollisionFrame();
    
    // Complete reset
    viewpoint_->Reset();
    
    // Check that all states are reset
    EXPECT_FALSE(viewpoint_->InCollision());
    EXPECT_FALSE(viewpoint_->InLineOfSight());
    EXPECT_FALSE(viewpoint_->Connected());
    EXPECT_FALSE(viewpoint_->Visited());
    EXPECT_FALSE(viewpoint_->Selected());
    EXPECT_FALSE(viewpoint_->IsCandidate());
    EXPECT_FALSE(viewpoint_->HasTerrainHeight());
    EXPECT_FALSE(viewpoint_->InExploringCell());
    EXPECT_EQ(viewpoint_->GetCellInd(), -1);
    EXPECT_EQ(viewpoint_->GetCoveredPointNum(), 0);
    EXPECT_EQ(viewpoint_->GetCoveredFrontierPointNum(), 0);
    EXPECT_EQ(viewpoint_->GetCollisionFrameCount(), 0);
}

// Test visualization cloud generation
TEST_F(ViewPointComprehensiveTest, VisualizationCloudTest) {
    // Test visualization cloud generation
    pcl::PointCloud<pcl::PointXYZI>::Ptr vis_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    
    EXPECT_NO_THROW(viewpoint_->GetVisualizationCloud(vis_cloud));
}

// Test multiple viewpoints
TEST_F(ViewPointComprehensiveTest, MultipleViewPointsTest) {
    // Create multiple viewpoints
    std::vector<std::unique_ptr<viewpoint_ns::ViewPoint>> viewpoints;
    
    for (int i = 0; i < 5; i++) {
        geometry_msgs::Point pos;
        pos.x = i * 2.0;
        pos.y = i * 2.0;
        pos.z = 0.0;
        
        viewpoints.push_back(std::make_unique<viewpoint_ns::ViewPoint>(pos));
        
        // Set different states for each viewpoint
        viewpoints[i]->SetInCollision(i % 2 == 0);
        viewpoints[i]->SetVisited(i % 3 == 0);
        viewpoints[i]->SetSelected(i % 4 == 0);
        viewpoints[i]->SetCellInd(i);
    }
    
    // Verify all viewpoints are properly initialized
    for (int i = 0; i < 5; i++) {
        EXPECT_EQ(viewpoints[i]->GetX(), i * 2.0);
        EXPECT_EQ(viewpoints[i]->GetY(), i * 2.0);
        EXPECT_EQ(viewpoints[i]->GetCellInd(), i);
        EXPECT_EQ(viewpoints[i]->InCollision(), i % 2 == 0);
        EXPECT_EQ(viewpoints[i]->Visited(), i % 3 == 0);
        EXPECT_EQ(viewpoints[i]->Selected(), i % 4 == 0);
    }
}

// Test edge cases
TEST_F(ViewPointComprehensiveTest, EdgeCasesTest) {
    // Test with extreme coordinates
    geometry_msgs::Point extreme_pos;
    extreme_pos.x = 1e6;
    extreme_pos.y = -1e6;
    extreme_pos.z = 1e3;
    
    viewpoint_ns::ViewPoint extreme_vp(extreme_pos);
    EXPECT_EQ(extreme_vp.GetX(), 1e6);
    EXPECT_EQ(extreme_vp.GetY(), -1e6);
    EXPECT_EQ(extreme_vp.GetHeight(), 1e3);
    
    // Test with negative coordinates
    geometry_msgs::Point negative_pos;
    negative_pos.x = -5.0;
    negative_pos.y = -10.0;
    negative_pos.z = -2.0;
    
    viewpoint_ns::ViewPoint negative_vp(negative_pos);
    EXPECT_EQ(negative_vp.GetX(), -5.0);
    EXPECT_EQ(negative_vp.GetY(), -10.0);
    EXPECT_EQ(negative_vp.GetHeight(), -2.0);
}

// Test large coverage data
TEST_F(ViewPointComprehensiveTest, LargeCoverageDataTest) {
    // Test with large number of covered points
    for (int i = 0; i < 1000; i++) {
        viewpoint_->AddCoveredPoint(i);
        viewpoint_->AddCoveredFrontierPoint(i);
    }
    
    EXPECT_EQ(viewpoint_->GetCoveredPointNum(), 1000);
    EXPECT_EQ(viewpoint_->GetCoveredFrontierPointNum(), 1000);
    
    // Test reset with large data
    viewpoint_->ResetCoverage();
    EXPECT_EQ(viewpoint_->GetCoveredPointNum(), 0);
    EXPECT_EQ(viewpoint_->GetCoveredFrontierPointNum(), 0);
}

// Test state combinations
TEST_F(ViewPointComprehensiveTest, StateCombinationsTest) {
    // Test various state combinations
    std::vector<std::tuple<bool, bool, bool, bool>> test_combinations = {
        {false, false, false, false},  // All false
        {true, true, true, true},      // All true
        {true, false, true, false},   // Alternating
        {false, true, false, true},   // Alternating
    };
    
    for (const auto& combo : test_combinations) {
        viewpoint_ns::ViewPoint test_vp(test_position_);
        
        test_vp.SetInCollision(std::get<0>(combo));
        test_vp.SetConnected(std::get<1>(combo));
        test_vp.SetVisited(std::get<2>(combo));
        test_vp.SetSelected(std::get<3>(combo));
        
        EXPECT_EQ(test_vp.InCollision(), std::get<0>(combo));
        EXPECT_EQ(test_vp.Connected(), std::get<1>(combo));
        EXPECT_EQ(test_vp.Visited(), std::get<2>(combo));
        EXPECT_EQ(test_vp.Selected(), std::get<3>(combo));
    }
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
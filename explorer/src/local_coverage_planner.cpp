/**
 * @file local_coverage_planner.cpp
 * @author joosoo (joosoo@buct.edu.cn)
 * @brief Class that ensures coverage in the surroundings of the robot
 * @version 0.1
 * @date 2021-05-31
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "explorer/local_coverage_planner.h"
#include <random>
#include <utility>
#include "explorer/tsp_solver.h"

namespace local_coverage_planner_ns
{
    const std::string LocalCoveragePlanner::kRuntimeUnit = "us";

    bool LocalCoveragePlannerParameter::ReadParameters(ros::NodeHandle& nh)
    {
        kMinAddPointNum = misc_utils_ns::getParam<int>(nh, "kMinAddPointNumSmall", 60);
        kMinAddFrontierPointNum = misc_utils_ns::getParam<int>(nh, "kMinAddFrontierPointNum", 30);
        kGreedyViewPointSampleRange = misc_utils_ns::getParam<int>(nh, "kGreedyViewPointSampleRange", 5);
        kLocalPathOptimizationItrMax = misc_utils_ns::getParam<int>(nh, "kLocalPathOptimizationItrMax", 10);
        return true;
    }

    LocalCoveragePlanner::LocalCoveragePlanner(ros::NodeHandle& nh) :
        lookahead_point_update_(false), use_frontier_(true), local_coverage_complete_(false)
    {
        parameters_.ReadParameters(nh);
    }

    // 获取边界视点索引
    int LocalCoveragePlanner::GetBoundaryViewpointIndex(const exploration_path_ns::ExplorationPath& global_path) const
    {
        int boundary_viewpoint_index = robot_viewpoint_ind_; // 初始化为机器人当前位置最接近的视点索引
        if (!global_path.nodes_.empty()) // 如果全局路径不为空
        {
            if (viewpoint_manager_->InLocalPlanningHorizon(
                    global_path.nodes_.front().position_)) // 如果全局路径的第一个节点在局部规划窗口内
            {
                // 遍历全局路径，寻找到不是GLOBAL_VIEWPOINT、HOME、在局部规划窗口内的路径点，以此点最近的候选视点作为边界视点
                // 即找到全局路径与局部规划窗口的边界路径点(GLOBAL_VIA_POINT)，求取该点最近的候选视点作为边界视点，将全局路径反向即可找到另一边界视点
                for (int i = 0; i < global_path.nodes_.size(); i++)
                {
                    if (global_path.nodes_[i].type_ == exploration_path_ns::NodeType::GLOBAL_VIEWPOINT ||
                        global_path.nodes_[i].type_ == exploration_path_ns::NodeType::HOME ||
                        !viewpoint_manager_->InLocalPlanningHorizon(global_path.nodes_[i].position_))
                    {
                        break;
                    }
                    boundary_viewpoint_index = viewpoint_manager_->GetNearestCandidateViewPointInd(
                        global_path.nodes_[i].position_); // 以离该点最近的候选视点作为边界视点
                }
            }
        }
        return boundary_viewpoint_index;
    }

    // 获取两个边界视点索引
    void LocalCoveragePlanner::GetBoundaryViewpointIndices(exploration_path_ns::ExplorationPath global_path)
    {
        start_viewpoint_ind_ = GetBoundaryViewpointIndex(global_path); // 顺向与局部规划范围相交为起始视点
        global_path.Reverse();
        end_viewpoint_ind_ = GetBoundaryViewpointIndex(global_path); // 逆向与局部规划范围相交为结束视点
    }

    void LocalCoveragePlanner::GetNavigationViewPointIndices(exploration_path_ns::ExplorationPath global_path,
                                                             std::vector<int>& navigation_viewpoint_indices)
    {
        // Get start and end point
        robot_viewpoint_ind_ = viewpoint_manager_->GetNearestCandidateViewPointInd(
            robot_position_); // 获取离机器人当前位置最近的候选视点的索引作为机器人视点
        lookahead_viewpoint_ind_ = viewpoint_manager_->GetNearestCandidateViewPointInd(
            lookahead_point_); // 同上，获取离lookahead_point_最近的候选视点视点的索引

        if (!lookahead_point_update_ || !viewpoint_manager_->InRange(lookahead_viewpoint_ind_))
        {
            lookahead_viewpoint_ind_ =
                robot_viewpoint_ind_; // 如果lookahead_point_没有更新或者不在local planning
                                      // range内，则lookahead_viewpoint_ind_ = robot_viewpoint_ind_
        }
        // Get connecting viewpoints to the global path
        GetBoundaryViewpointIndices(std::move(global_path)); // 获取边界viewpoint索引，两个边界视点

        // Update the coverage with viewpoints that must visit，更新视点的覆盖范围这些点必须访问
        navigation_viewpoint_indices.push_back(start_viewpoint_ind_);
        navigation_viewpoint_indices.push_back(end_viewpoint_ind_);
        navigation_viewpoint_indices.push_back(robot_viewpoint_ind_);
        navigation_viewpoint_indices.push_back(lookahead_viewpoint_ind_);
    }

    // 更新视点的覆盖point
    void LocalCoveragePlanner::UpdateViewPointCoveredPoint(std::vector<bool>& point_list, int viewpoint_index,
                                                           bool use_array_ind) const
    {
        for (const auto& point_ind : viewpoint_manager_->GetViewPointCoveredPointList(viewpoint_index, use_array_ind))
        {
            MY_ASSERT(misc_utils_ns::InRange<bool>(point_list, point_ind));
            point_list[point_ind] = true;
        }
    }

    // 更新视点的覆盖frontier point
    void LocalCoveragePlanner::UpdateViewPointCoveredFrontierPoint(std::vector<bool>& frontier_point_list,
                                                                   int viewpoint_index, bool use_array_ind) const
    {
        for (const auto& point_ind :
             viewpoint_manager_->GetViewPointCoveredFrontierPointList(viewpoint_index, use_array_ind))
        {
            MY_ASSERT(misc_utils_ns::InRange<bool>(frontier_point_list, point_ind));
            frontier_point_list[point_ind] = true;
        }
    }

    // 根据每个视点covered的point或者frontier point(reward),对视点分别进行排序cover_point_queue&frontier_queue
    void LocalCoveragePlanner::EnqueueViewpointCandidates(std::vector<std::pair<int, int>>& cover_point_queue,
                                                          std::vector<std::pair<int, int>>& frontier_queue,
                                                          const std::vector<bool>& covered_point_list,
                                                          const std::vector<bool>& covered_frontier_point_list,
                                                          const std::vector<int>& selected_viewpoint_array_indices) const
    {
        // 遍历所有候选视点
        for (const auto& viewpoint_index : viewpoint_manager_->GetViewPointCandidateIndices())
        {
            // 如果该视点已经被访问过或者不在Exploring区域中，则跳过
            if (viewpoint_manager_->ViewPointVisited(viewpoint_index) ||
                !viewpoint_manager_->ViewPointInExploringCell(viewpoint_index))
            {
                continue;
            }
            // 获取该视点的数组索引
            int viewpoint_array_index = viewpoint_manager_->GetViewPointArrayInd(viewpoint_index);
            // 如果该视点已经被选中过，则跳过
            if (std::find(selected_viewpoint_array_indices.begin(), selected_viewpoint_array_indices.end(),
                          viewpoint_array_index) != selected_viewpoint_array_indices.end())
            {
                continue;
            }
            // 获取该视点的覆盖点数
            int covered_point_num =
                viewpoint_manager_->GetViewPointCoveredPointNum(covered_point_list, viewpoint_array_index, true);
            // 如果该视点的覆盖点数大于等于最小添加点数，则将其加入到候选队列中
            if (covered_point_num >= parameters_.kMinAddPointNum)
            {
                cover_point_queue.emplace_back(
                    covered_point_num,
                    viewpoint_index); // 在队列末尾插入一个pair，第一个元素是覆盖点数，第二个元素是对应的视点索引
            }
            else if (use_frontier_) // 如果使用frontier
            {
                // 获取该视点的覆盖frontier点数
                int covered_frontier_point_num = viewpoint_manager_->GetViewPointCoveredFrontierPointNum(
                    covered_frontier_point_list, viewpoint_array_index, true);

                if (covered_frontier_point_num >= parameters_.kMinAddFrontierPointNum)
                {
                    frontier_queue.emplace_back(
                        covered_frontier_point_num,
                        viewpoint_index); // 在队列末尾插入一个pair，第一个元素是覆盖frontier点数，第二个元素是对应的视点索引
                }
            }
        }

        // Sort the queue-->对队列进行排序，依据覆盖点数进行降序排序
        std::sort(cover_point_queue.begin(), cover_point_queue.end(), SortPairInRev);
        if (use_frontier_)
        {
            //  Sort the queue-->对队列进行排序，依据覆盖frontier点数进行降序排序
            std::sort(frontier_queue.begin(), frontier_queue.end(), SortPairInRev);
        }
    }

    // 从候选视点中根据动态更新的reward(cover point)选取视点,且cover的point必须大于kMinAddPointNum才会被选做视点
    void LocalCoveragePlanner::SelectViewPoint(const std::vector<std::pair<int, int>>& queue,
                                               const std::vector<bool>& covered,
                                               std::vector<int>& selected_viewpoint_indices, bool use_frontier) const
    {
        if (use_frontier) // false
        {
            if (queue.empty() || queue[0].first < parameters_.kMinAddFrontierPointNum)
            {
                return;
            }
        }
        else
        {
            // 保证队列不为空且队列中第一个元素的覆盖点数大于等于最小添加点数
            if (queue.empty() || queue[0].first < parameters_.kMinAddPointNum)
            {
                return;
            }
        }

        // 拷贝队列和覆盖点列表
        std::vector<bool> covered_copy;
        for (const bool i : covered)
        {
            covered_copy.push_back(i);
        }
        std::vector<std::pair<int, int>> queue_copy;
        for (const auto & i : queue)
        {
            queue_copy.push_back(i);
        }

        int sample_range = 0;
        // 遍历排序队列，获取满足最小添加点数的范围
        for (auto & i : queue_copy)
        {
            if (use_frontier)
            {
                if (i.first >= parameters_.kMinAddFrontierPointNum)
                {
                    sample_range++;
                }
            }
            else
            {
                if (i.first >= parameters_.kMinAddPointNum)
                {
                    sample_range++;
                }
            }
        }
        // 为sample_range参数比较的较小值 <= kGreedyViewPointSampleRange
        sample_range = std::min(parameters_.kGreedyViewPointSampleRange, sample_range);
        // 用于真正获取随机数的设备
        std::random_device rd;
        // 初始化随机数引擎
        std::mt19937 gen(rd());
        // [0, sample_range - 1]之间的均匀整数分布
        std::uniform_int_distribution<> gen_next_queue_idx(0, sample_range - 1);
        int queue_idx = gen_next_queue_idx(gen); // 通过随机数引擎生成一个随机数，该随机数表示队列中的索引
        int cur_ind = queue_copy[queue_idx].second; // 该随机队列索引对应的视点索引

        // 先在smaple_range范围内随机选一个视点，去更新covered
        // 无限循环
        while (true)
        {
            // 获取当前视点的数组索引
            const int cur_array_ind = viewpoint_manager_->GetViewPointArrayInd(cur_ind);
            if (use_frontier) // false
            {
                for (const auto& point_ind :
                     viewpoint_manager_->GetViewPointCoveredFrontierPointList(cur_array_ind, true))

                {
                    MY_ASSERT(misc_utils_ns::InRange<bool>(covered_copy, point_ind));
                    if (!covered_copy[point_ind])
                    {
                        covered_copy[point_ind] = true;
                    }
                }
            }
            else
            {
                // 遍历当前视点的覆盖点列表，将covered_copy中相应的未覆盖的点标记为已覆盖
                for (const auto& point_ind : viewpoint_manager_->GetViewPointCoveredPointList(cur_array_ind, true))
                {
                    MY_ASSERT(misc_utils_ns::InRange<bool>(covered_copy, point_ind));
                    if (!covered_copy[point_ind])
                    {
                        covered_copy[point_ind] = true;
                    }
                }
            } // 将该点加入已选中的视点索引列表中
            selected_viewpoint_indices.push_back(cur_ind);
            queue_copy.erase(queue_copy.begin() + queue_idx); // 再从队列中删除该元素

            // Update the queue 更新队列
            for (auto & i : queue_copy)
            {
                int add_point_num = 0;
                int ind = i.second; // 按顺序
                int array_ind = viewpoint_manager_->GetViewPointArrayInd(ind); // 获取当前视点的数组索引

                if (use_frontier) // false
                {
                    for (const auto& point_ind :
                         viewpoint_manager_->GetViewPointCoveredFrontierPointList(array_ind, true))
                    {
                        MY_ASSERT(misc_utils_ns::InRange<bool>(covered_copy, point_ind));
                        if (!covered_copy[point_ind])
                        {
                            add_point_num++;
                        }
                    }
                }
                else
                {
                    // 同样，获取当前视点的覆盖点列表，更与covered_copy列表相比较，更新该视点的covered reward
                    for (const auto& point_ind : viewpoint_manager_->GetViewPointCoveredPointList(array_ind, true))
                    {
                        MY_ASSERT(misc_utils_ns::InRange<bool>(covered_copy, point_ind));
                        if (!covered_copy[point_ind])
                        {
                            add_point_num++;
                        }
                    }
                }

                i.first = add_point_num;
            }

            // 根据最新的reward值对队列进行重排序
            std::sort(queue_copy.begin(), queue_copy.end(), SortPairInRev);

            // 同上-->loop，在范围内随机选取一个视点，并更新其他viewpoint的reward值
            if (queue_copy.empty() || queue_copy[0].first < parameters_.kMinAddPointNum)
            {
                break;
            }
            if (use_frontier)
            {
                if (queue_copy.empty() || queue_copy[0].first < parameters_.kMinAddFrontierPointNum)
                {
                    break;
                }
            }

            // Randomly select the next point
            int sample_range = 0;
            for (const auto & i : queue)
            {
                if (use_frontier)
                {
                    if (i.first >= parameters_.kMinAddFrontierPointNum)
                    {
                        sample_range++;
                    }
                }
                else
                {
                    if (i.first >= parameters_.kMinAddPointNum)
                    {
                        sample_range++;
                    }
                }
            }
            // 同上
            sample_range = std::min(parameters_.kGreedyViewPointSampleRange, sample_range);
            std::uniform_int_distribution<int> gen_next_queue_idx(0, sample_range - 1);
            queue_idx = gen_next_queue_idx(gen);
            cur_ind = queue_copy[queue_idx].second;
        }
    }

    void LocalCoveragePlanner::SelectViewPointFromFrontierQueue(std::vector<std::pair<int, int>>& frontier_queue,
                                                                std::vector<bool>& frontier_covered,
                                                                std::vector<int>& selected_viewpoint_indices) const
    {
        if (use_frontier_ && !frontier_queue.empty() && frontier_queue[0].first > parameters_.kMinAddFrontierPointNum)
        {
            // Update the frontier queue，遍历已选择的视点，更新已经被covered的frontier数量
            for (const auto& ind : selected_viewpoint_indices)
            {
                UpdateViewPointCoveredFrontierPoint(frontier_covered, ind);
            }
            // 遍历frontier队列，更新队列frontier reward
            for (auto & i : frontier_queue)
            {
                const int ind = i.second;
                const int covered_frontier_point_num =
                    viewpoint_manager_->GetViewPointCoveredFrontierPointNum(frontier_covered, ind);
                i.first = covered_frontier_point_num;
            }
            // 根据reward进行降序排序
            std::sort(frontier_queue.begin(), frontier_queue.end(), SortPairInRev);
            // 同样的，选取关联frontier的视点
            SelectViewPoint(frontier_queue, frontier_covered, selected_viewpoint_indices, true);
        }
    }

    exploration_path_ns::ExplorationPath
    LocalCoveragePlanner::SolveTSP(const std::vector<int>& selected_viewpoint_indices,
                                   std::vector<int>& ordered_viewpoint_indices)

    {
        // nav_msgs::Path tsp_path;
        exploration_path_ns::ExplorationPath tsp_path;

        // 如果被选中的视点为空，则直接返回空路径
        if (selected_viewpoint_indices.empty())
        {
            return tsp_path;
        }

        // Get start and end index
        int start_ind = selected_viewpoint_indices.size() - 1;
        int end_ind = selected_viewpoint_indices.size() - 1; // loop
        // 初始化robot_ind和lookahead_ind
        int robot_ind = 0;
        int lookahead_ind = 0;

        // 遍历所有被选中的视点，找到导航视点在列表中的索引
        for (int i = 0; i < selected_viewpoint_indices.size(); i++)
        {
            if (selected_viewpoint_indices[i] == start_viewpoint_ind_)
            {
                start_ind = i;
            }
            if (selected_viewpoint_indices[i] == end_viewpoint_ind_)
            {
                end_ind = i;
            }
            if (selected_viewpoint_indices[i] == robot_viewpoint_ind_)
            {
                robot_ind = i;
            }
            if (selected_viewpoint_indices[i] == lookahead_viewpoint_ind_)
            {
                lookahead_ind = i;
            }
        }

        bool has_start_end_dummy = start_ind != end_ind; // 判断起点终点是否(不同)为dummy点
        bool has_robot_lookahead_dummy = robot_ind != lookahead_ind; // 判断机器人和lookahead point是否(不同)为dummy点

        // Get distance matrix
        int node_size;
        // 根据导航视点的情况，调整整体的节点数量
        if (has_start_end_dummy && has_robot_lookahead_dummy)
        {
            node_size = selected_viewpoint_indices.size() + 2;
        }
        else if (has_start_end_dummy || has_robot_lookahead_dummy)
        {
            node_size = selected_viewpoint_indices.size() + 1;
        }
        else
        {
            node_size = selected_viewpoint_indices.size();
        }

        misc_utils_ns::Timer find_path_timer("find path");
        find_path_timer.Start();
        // 初始化距离矩阵
        std::vector<std::vector<int>> distance_matrix(node_size, std::vector<int>(node_size, 0));
        std::vector<int> tmp;
        // 遍历所有视点，填充对称的距离矩阵
        for (int i = 0; i < selected_viewpoint_indices.size(); i++)
        {
            //
            int from_ind = selected_viewpoint_indices[i];

            for (int j = 0; j < i; j++)
            {
                int to_ind = selected_viewpoint_indices[j];
                // 寻找并返回两个该视点之间的最短路径
                nav_msgs::Path path = viewpoint_manager_->GetViewPointShortestPath(from_ind, to_ind);
                // 计算路径长度
                double path_length = misc_utils_ns::GetPathLength(path);
                // 填入距离矩阵
                distance_matrix[i][j] = static_cast<int>(10 * path_length);
            }
        }
        // 完成整个距离矩阵的填充
        for (int i = 0; i < selected_viewpoint_indices.size(); i++)
        {
            for (int j = i + 1; j < selected_viewpoint_indices.size(); j++)
            {
                distance_matrix[i][j] = distance_matrix[j][i];
            }
        }

        // 根据dummy情况，扩展距离矩阵
        // Add a fake node to connect the start and end nodes
        if (has_start_end_dummy && has_robot_lookahead_dummy)
        {
            int start_end_dummy_node_ind = node_size - 1;
            int robot_lookahead_dummy_node_ind = node_size - 2; // 最后两个节点为虚拟节点
            for (int i = 0; i < selected_viewpoint_indices.size(); i++)
            {
                // start_ind != end_ind
                if (i == start_ind || i == end_ind)
                {
                    distance_matrix[i][start_end_dummy_node_ind] = 0;
                    distance_matrix[start_end_dummy_node_ind][i] = 0; // 表示dummy与start_end之间的距离为0
                }
                else
                {
                    distance_matrix[i][start_end_dummy_node_ind] = 9999;
                    distance_matrix[start_end_dummy_node_ind][i] = 9999; // 表示dummy与start_end之间的距离为9999
                }
                // robot_ind != lookahead_ind
                if (i == robot_ind || i == lookahead_ind)
                {
                    distance_matrix[i][robot_lookahead_dummy_node_ind] = 0;
                    distance_matrix[robot_lookahead_dummy_node_ind][i] = 0;
                }
                else
                {
                    distance_matrix[i][robot_lookahead_dummy_node_ind] = 9999;
                    distance_matrix[robot_lookahead_dummy_node_ind][i] = 9999;
                }
            }

            distance_matrix[start_end_dummy_node_ind][robot_lookahead_dummy_node_ind] = 9999;
            distance_matrix[robot_lookahead_dummy_node_ind][start_end_dummy_node_ind] = 9999;
        }
        else if (has_start_end_dummy)
        {
            int end_node_ind = node_size - 1;
            for (int i = 0; i < selected_viewpoint_indices.size(); i++)
            {
                if (i == start_ind || i == end_ind)
                {
                    distance_matrix[i][end_node_ind] = 0;
                    distance_matrix[end_node_ind][i] = 0;
                }
                else
                {
                    distance_matrix[i][end_node_ind] = 9999;
                    distance_matrix[end_node_ind][i] = 9999;
                }
            }
        }
        else if (has_robot_lookahead_dummy)
        {
            int end_node_ind = node_size - 1;
            for (int i = 0; i < selected_viewpoint_indices.size(); i++)
            {
                if (i == robot_ind || i == lookahead_ind)
                {
                    distance_matrix[i][end_node_ind] = 0;
                    distance_matrix[end_node_ind][i] = 0;
                }
                else
                {
                    distance_matrix[i][end_node_ind] = 9999;
                    distance_matrix[end_node_ind][i] = 9999;
                }
            }
        }

        find_path_timer.Stop(false);
        find_path_runtime_ += find_path_timer.GetDuration(kRuntimeUnit);

        misc_utils_ns::Timer tsp_timer("tsp");
        tsp_timer.Start();

        // 构建tsp问题
        tsp_solver_ns::DataModel data;
        data.distance_matrix = distance_matrix;
        data.depot = start_ind; // 断点数量？

        std::cout << "size of distance_matrix: " << distance_matrix.size() << std::endl;

        // 求解tsp问题
        tsp_solver_ns::TSPSolver tsp_solver(data);
        tsp_solver.Solve();

        // 获取路径
        std::vector<int> path_index;
        if (has_start_end_dummy) //
        {
            tsp_solver.getSolutionNodeIndex(path_index, true);
        }
        else
        {
            tsp_solver.getSolutionNodeIndex(path_index, false);
        }

        // Get rid of the fake node connecting the robot and lookahead point
        // 为了移除dummy节点，需要确保机器人视点和lookahead视点不再通过这个虚拟节点连接
        for (int i = 0; i < path_index.size(); i++)
        {
            if (path_index[i] >= selected_viewpoint_indices.size() || path_index[i] < 0)
            {
                path_index.erase(path_index.begin() + i);
                i--;
            }
        }

        // 将排序后的视点索引填充到ordered_viewpoint_indices
        ordered_viewpoint_indices.clear();
        for (int i : path_index)
        {
            ordered_viewpoint_indices.push_back(selected_viewpoint_indices[i]);
        }

        // Add the end node index
        if (start_ind == end_ind && !path_index.empty())
        {
            path_index.push_back(path_index[0]);
        }

        tsp_timer.Stop(false);
        tsp_runtime_ += tsp_timer.GetDuration(kRuntimeUnit);

        if (path_index.size() > 1)
        {
            int cur_ind;
            int next_ind;

            // 遍历路径点，分配路径点属性
            for (int i = 0; i < path_index.size() - 1; i++)
            {
                cur_ind = selected_viewpoint_indices[path_index[i]];
                next_ind = selected_viewpoint_indices[path_index[i + 1]];

                geometry_msgs::Point cur_node_position = viewpoint_manager_->GetViewPointPosition(cur_ind); //
                exploration_path_ns::Node cur_node(cur_node_position, exploration_path_ns::NodeType::LOCAL_VIEWPOINT);
                cur_node.local_viewpoint_ind_ = cur_ind;
                // 根据情况，赋予节点类型属性
                if (cur_ind == robot_viewpoint_ind_)
                {
                    cur_node.type_ = exploration_path_ns::NodeType::ROBOT;
                }
                else if (cur_ind == lookahead_viewpoint_ind_)
                {
                    int covered_point_num = viewpoint_manager_->GetViewPointCoveredPointNum(cur_ind);
                    int covered_frontier_num = viewpoint_manager_->GetViewPointCoveredFrontierPointNum(cur_ind);

                    if (covered_point_num > parameters_.kMinAddPointNum ||
                        covered_frontier_num > parameters_.kMinAddFrontierPointNum)
                    {
                        cur_node.type_ = exploration_path_ns::NodeType::LOCAL_VIEWPOINT;
                    }
                    else
                    {
                        cur_node.type_ = exploration_path_ns::NodeType::LOOKAHEAD_POINT;
                    }
                }
                else if (cur_ind == start_viewpoint_ind_)
                {
                    cur_node.type_ = exploration_path_ns::NodeType::LOCAL_PATH_START;
                }
                else if (cur_ind == end_viewpoint_ind_)
                {
                    cur_node.type_ = exploration_path_ns::NodeType::LOCAL_PATH_END;
                }
                tsp_path.Append(cur_node);

                // 获取两视点之间的路径
                nav_msgs::Path path_between_viewpoints =
                    viewpoint_manager_->GetViewPointShortestPath(cur_ind, next_ind);

                // 将两视点之间的路径添加到tsp路径中,其属性为LOCAL_VIA_POINT
                if (path_between_viewpoints.poses.size() > 2)
                {
                    for (int j = 1; j < path_between_viewpoints.poses.size() - 1; j++)
                    {
                        exploration_path_ns::Node node;
                        node.type_ = exploration_path_ns::NodeType::LOCAL_VIA_POINT;
                        node.local_viewpoint_ind_ = -1;

                        node.position_.x() = path_between_viewpoints.poses[j].pose.position.x;
                        node.position_.y() = path_between_viewpoints.poses[j].pose.position.y;
                        node.position_.z() = path_between_viewpoints.poses[j].pose.position.z;
                        tsp_path.Append(node);
                    }
                }

                // 这段存在不存似乎无伤大雅？
                geometry_msgs::Point next_node_position = viewpoint_manager_->GetViewPointPosition(next_ind);
                exploration_path_ns::Node next_node(next_node_position, exploration_path_ns::NodeType::LOCAL_VIEWPOINT);
                next_node.local_viewpoint_ind_ = next_ind;

                if (next_ind == robot_viewpoint_ind_)
                {
                    next_node.type_ = exploration_path_ns::NodeType::ROBOT;
                }
                else if (next_ind == lookahead_viewpoint_ind_)
                {
                    next_node.type_ = exploration_path_ns::NodeType::LOOKAHEAD_POINT;
                }
                else if (next_ind == start_viewpoint_ind_)
                {
                    next_node.type_ = exploration_path_ns::NodeType::LOCAL_PATH_START;
                }
                else if (next_ind == end_viewpoint_ind_)
                {
                    next_node.type_ = exploration_path_ns::NodeType::LOCAL_PATH_END;
                }
                tsp_path.Append(next_node);
            }
        }
        return tsp_path;
    }

    exploration_path_ns::ExplorationPath
    LocalCoveragePlanner::SolveLocalCoverageProblem(const exploration_path_ns::ExplorationPath& global_path,
                                                    int uncovered_point_num, int uncovered_frontier_point_num)
    {
        exploration_path_ns::ExplorationPath local_path; // 局部路径

        find_path_runtime_ = 0;
        viewpoint_sampling_runtime_ = 0;
        tsp_runtime_ = 0;

        local_coverage_complete_ = false; //

        misc_utils_ns::Timer find_path_timer("find path");
        find_path_timer.Start();

        std::vector<int> navigation_viewpoint_indices; // 导航视点索引
        GetNavigationViewPointIndices(
            global_path,
            navigation_viewpoint_indices); // 返回四个viewpoint(两个边界点、lookahead point、robot_viewpoint)

        find_path_timer.Stop(false);
        find_path_runtime_ += find_path_timer.GetDuration(kRuntimeUnit);

        // Sampling viewpoints，视点采样
        misc_utils_ns::Timer viewpoint_sampling_timer("viewpoint sampling");
        viewpoint_sampling_timer.Start();

        std::vector<bool> covered(uncovered_point_num, false); // 初始化covered向量，全为零
        std::vector<bool> frontier_covered(uncovered_frontier_point_num, false); // 初始化frontier_covered向量，全为零

        std::vector<int> pre_selected_viewpoint_array_indices; // 预选的viewpoint索引队列
        std::vector<int> reused_viewpoint_indices; // 重用viewpoint索引集合、

        // 遍历上次选中的viewpoint索引队列
        for (auto& viewpoint_array_ind : last_selected_viewpoint_array_indices_)
        {
            // 如果该viewpoint已经被访问过或者该点不是形成路径的候选点则直接跳过
            if (viewpoint_manager_->ViewPointVisited(viewpoint_array_ind, true) ||
                !viewpoint_manager_->IsViewPointCandidate(viewpoint_array_ind, true))
            {
                continue;
            }
            // 获取该viewpoint覆盖的点数，未对covered进行改变
            int covered_point_num = viewpoint_manager_->GetViewPointCoveredPointNum(covered, viewpoint_array_ind, true);
            // 如果该viewpoint覆盖的点数大于等于最小添加点数则将其加入到复用的viewpoint索引队列中
            if (covered_point_num >= parameters_.kMinAddPointNum)
            {
                reused_viewpoint_indices.push_back(viewpoint_manager_->GetViewPointInd(viewpoint_array_ind));
            }
            else if (use_frontier_) // 使用frontier
            {
                // 获取该viewpoint覆盖的frontier点数，未对frontier_covered进行改变
                int covered_frontier_point_num = viewpoint_manager_->GetViewPointCoveredFrontierPointNum(
                    frontier_covered, viewpoint_array_ind, true);
                // 如果该viewpoint覆盖的frontier点数大于等于最小添加frontier点数则将其加入到复用的viewpoint索引队列中
                if (covered_frontier_point_num >= parameters_.kMinAddFrontierPointNum)
                {
                    reused_viewpoint_indices.push_back(viewpoint_manager_->GetViewPointInd(viewpoint_array_ind));
                }
            }
        }

        for (const auto& ind : reused_viewpoint_indices)
        {
            // 获取该viewpoint对应的array索引-->?当前数组排列？
            int viewpoint_array_ind = viewpoint_manager_->GetViewPointArrayInd(ind);
            pre_selected_viewpoint_array_indices.push_back(
                viewpoint_array_ind); // 将该viewpoint对应的索引加入到预选的viewpoint索引队列中
        }

        // 遍历所有的导航视点索引(机器人视点、lookahead point、边界视点)
        for (const auto& ind : navigation_viewpoint_indices)
        {
            int array_ind = viewpoint_manager_->GetViewPointArrayInd(ind);
            pre_selected_viewpoint_array_indices.push_back(
                array_ind); // 将该导航视点的索引加入到预选的viewpoint索引队列中
        }

        // Update coverage 遍历所有预选视点viewpoint(复用(reused)点和导航(nav)点)，更新covered和frontier_covered
        for (auto& viewpoint_array_ind : pre_selected_viewpoint_array_indices)
        {
            // Update covered points and frontiers
            UpdateViewPointCoveredPoint(covered, viewpoint_array_ind, true);
            if (use_frontier_)
            {
                UpdateViewPointCoveredFrontierPoint(frontier_covered, viewpoint_array_ind, true);
            }
        }

        // Enqueue candidate viewpoints 候选视点排序
        std::vector<std::pair<int, int>> queue;
        std::vector<std::pair<int, int>> frontier_queue;
        // 对所有的候选视点进行排序(降序排序)，根据covered的point和frontier进行，其中去除访问过的候选视点、预选的视点、不在Exploring
        // cell中的候选视点 未对covered和frontier_covered进行改变
        EnqueueViewpointCandidates(queue, frontier_queue, covered, frontier_covered,
                                   pre_selected_viewpoint_array_indices);

        viewpoint_sampling_timer.Stop(false, kRuntimeUnit);
        viewpoint_sampling_runtime_ += viewpoint_sampling_timer.GetDuration(kRuntimeUnit);

        std::vector<int> ordered_viewpoint_indices;
        // 候选视点不为空且排在第一位的候选视点覆盖的点数大于等于最小添加点数
        if (!queue.empty() && queue[0].first > parameters_.kMinAddPointNum)
        {
            auto min_path_length = DBL_MAX; //
            // 迭代优化局部路径---->由于视点选取存在一定随机性，所以路径并不一定最优(最短？)
            for (int itr = 0; itr < parameters_.kLocalPathOptimizationItrMax; itr++)
            {
                std::vector<int> selected_viewpoint_indices_itr;

                // Select from the queue
                misc_utils_ns::Timer select_viewpoint_timer("select viewpoints");
                select_viewpoint_timer.Start();

                // 选取关于障碍物表面的视点
                SelectViewPoint(queue, covered, selected_viewpoint_indices_itr, false);
                // 选取关于frontier的视点
                SelectViewPointFromFrontierQueue(frontier_queue, frontier_covered, selected_viewpoint_indices_itr);

                // 此时从候选视点中选中的视点为selected_viewpoint_indices_itr

                // pre_selected_viewpoint_array_indices 由reused_viewpoint_indices和navigation_viewpoint_indices组成

                // Add viewpoints from last planning cycle
                for (const auto& ind : reused_viewpoint_indices)
                {
                    selected_viewpoint_indices_itr.push_back(ind);
                }

                // Add viewpoints for navigation
                for (const auto& ind : navigation_viewpoint_indices)
                {
                    selected_viewpoint_indices_itr.push_back(ind);
                } // 同时将导航视点和复用视点加入到视点选中集合之中

                misc_utils_ns::UniquifyIntVector(selected_viewpoint_indices_itr); // 对被选视点集合进行去重、排序处理

                select_viewpoint_timer.Stop(false, kRuntimeUnit);
                viewpoint_sampling_runtime_ += select_viewpoint_timer.GetDuration(kRuntimeUnit);

                // Solve the TSP problem
                exploration_path_ns::ExplorationPath local_path_itr;

                // 获取局部路径，并返回求解之后的视点遍历顺序
                local_path_itr = SolveTSP(selected_viewpoint_indices_itr, ordered_viewpoint_indices);

                double path_length = local_path_itr.GetLength();
                // 找到迭代数次中最短的路径，作为局部路径
                if (!local_path_itr.nodes_.empty() && path_length < min_path_length)
                {
                    min_path_length = path_length;
                    local_path = local_path_itr;
                    last_selected_viewpoint_indices_ = ordered_viewpoint_indices;
                }
            }
        }
        else // 如果该区域内的无障碍物surface点云或者点云较少，故而queue为空或者最大的covered reward小于阈值
        {
            misc_utils_ns::Timer select_viewpoint_timer("viewpoint sampling");
            select_viewpoint_timer.Start();

            // std::cout << "entering tsp routine" << std::endl;
            std::vector<int> selected_viewpoint_indices_itr; //

            // Add viewpoints from last planning cycle 继承上一次的选中的视点
            for (const auto& ind : reused_viewpoint_indices)
            {
                selected_viewpoint_indices_itr.push_back(ind);
            }

            // 从frontier队列中选取视点
            SelectViewPointFromFrontierQueue(frontier_queue, frontier_covered, selected_viewpoint_indices_itr);

            // 如果此时选取的视点为空，说明当前局部区域没有需要访问的viewpoint
            if (selected_viewpoint_indices_itr.empty())
            {
                local_coverage_complete_ = true;
            }

            // Add viewpoints for navigation 将导航视点加入到选中的视点集合中
            for (const auto& ind : navigation_viewpoint_indices)
            {
                selected_viewpoint_indices_itr.push_back(ind);
            }

            // 去重、排序所选择的视点
            misc_utils_ns::UniquifyIntVector(selected_viewpoint_indices_itr);

            select_viewpoint_timer.Stop(false, kRuntimeUnit);
            viewpoint_sampling_runtime_ += select_viewpoint_timer.GetDuration(kRuntimeUnit);

            // 返回局部路径
            local_path = SolveTSP(selected_viewpoint_indices_itr, ordered_viewpoint_indices);

            last_selected_viewpoint_indices_ = ordered_viewpoint_indices;
        }

        last_selected_viewpoint_array_indices_.clear();
        for (const auto& ind : last_selected_viewpoint_indices_)
        {
            int array_ind = viewpoint_manager_->GetViewPointArrayInd(ind);
            last_selected_viewpoint_array_indices_.push_back(array_ind);
        } // 保存所选的视点数组索引

        int viewpoint_num = viewpoint_manager_->GetViewPointNum(); // 获取视点数量
        for (int i = 0; i < viewpoint_num; i++)
        {
            viewpoint_manager_->SetViewPointSelected(i, false, true); // 初始化所有视点为未选中状态
        }

        // 根据选中的视点，在viewpoint_manager中将对应的视点设置为选中状态
        for (const auto& viewpoint_index : last_selected_viewpoint_indices_)
        {
            if (viewpoint_index != robot_viewpoint_ind_ && viewpoint_index != start_viewpoint_ind_ &&
                viewpoint_index != end_viewpoint_ind_ && viewpoint_index != lookahead_viewpoint_ind_)
            {
                viewpoint_manager_->SetViewPointSelected(viewpoint_index, true);
            }
        }
        // 最后返回得到的局部路径
        return local_path;
    }

    // 获取选中的视点可视化点云
    void LocalCoveragePlanner::GetSelectedViewPointVisCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) const
    {
        cloud->clear();
        for (const auto& viewpoint_index : last_selected_viewpoint_indices_)
        {
            geometry_msgs::Point position = viewpoint_manager_->GetViewPointPosition(viewpoint_index);
            pcl::PointXYZI point;
            point.x = position.x;
            point.y = position.y;
            point.z = position.z;
            if (viewpoint_index == robot_viewpoint_ind_)
            {
                point.intensity = 0.0;
            }
            else if (viewpoint_index == start_viewpoint_ind_)
            {
                point.intensity = 1.0;
            }
            else if (viewpoint_index == end_viewpoint_ind_)
            {
                point.intensity = 2.0;
            }
            else if (viewpoint_index == lookahead_viewpoint_ind_)
            {
                point.intensity = 0.0;
            }
            else
            {
                point.intensity = 4.0;
            }
            cloud->points.push_back(point);
        }
    }
} // namespace local_coverage_planner_ns

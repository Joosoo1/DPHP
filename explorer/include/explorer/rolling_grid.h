
#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <vector>

#include "explorer/grid.h"
#include "explorer/misc_utils.h"

namespace rolling_grid_ns {
    class RollingGrid {
    public:
        explicit RollingGrid(Eigen::Vector3i size);
        ~RollingGrid() = default;

        bool InRange(const Eigen::Vector3i& sub) const {
            return grid0_->InRange(sub);
        }

        bool InRange(const int ind) const {
            return grid0_->InRange(ind);
        }

        Eigen::Vector3i Ind2Sub(const int ind) const {
            return grid0_->Ind2Sub(ind);
        }

        int Sub2Ind(const Eigen::Vector3i& sub) const {
            return grid0_->Sub2Ind(sub);
        }

        int GetArrayInd(const Eigen::Vector3i& sub) const {
            MY_ASSERT(InRange(sub));
            if (which_grid_) { return grid1_->GetCellValue(sub); }
            return grid0_->GetCellValue(sub);
        }

        int GetArrayInd(const int ind) const {
            MY_ASSERT(InRange(ind));
            const Eigen::Vector3i sub = grid0_->Ind2Sub(ind);
            return GetArrayInd(sub);
        }

        int GetInd(const int array_ind) const {
            MY_ASSERT(InRange(array_ind));
            return array_ind_to_ind_[array_ind];
        }

        void Roll(const Eigen::Vector3i& roll_dir);
        void GetUpdatedIndices(std::vector<int>& updated_indices) const;
        void GetRolledOutIndices(const Eigen::Vector3i& roll_dir,
                                 std::vector<int>& rolled_out_indices);
        void GetUpdatedArrayIndices(std::vector<int>& updated_array_indices) const;

    private:
        Eigen::Vector3i size_;
        std::unique_ptr<grid_ns::Grid<int>> grid0_;
        std::unique_ptr<grid_ns::Grid<int>> grid1_;
        std::vector<int> updated_indices_;
        std::vector<int> array_ind_to_ind_;
        bool which_grid_;

        static int GetFromIdx(const int cur_idx, const int roll_step, const int max_idx) {
            return cur_idx <= roll_step - 1 ? max_idx - roll_step + cur_idx : cur_idx - roll_step;
        }

        void RollHelper(const std::unique_ptr<grid_ns::Grid<int>>& grid_in,
                        const std::unique_ptr<grid_ns::Grid<int>>& grid_out,
                        Eigen::Vector3i roll_dir);

        void GetRolledInIndices(const Eigen::Vector3i& roll_dir);
        void GetIndices(std::vector<int>& indices, Eigen::Vector3i start_idx,
                        Eigen::Vector3i end_idx) const;
    };
}  // namespace rolling_grid_ns

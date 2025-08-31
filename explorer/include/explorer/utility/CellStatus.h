/**
 * @file CellStatus.h
 * @author your name (you@domain.com)
 * @brief Cell status enumeration for GridWorld
 * @version 0.1
 * @date 2025-08-31
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once

namespace grid_world_ns
{
    enum class CellStatus
    {
        UNSEEN = 0,
        EXPLORING = 1,
        COVERED = 2,
        COVERED_BY_OTHERS = 3,
        NOGO = 4,
        SEMI_EXPLORED = 5
    };
}
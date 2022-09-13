#ifndef __GRID_SUBMAP_H__
#define __GRID_SUBMAP_H__
#include <iostream>
#include <Eigen/Core>
#include <memory>
#include <vector>
#include "grid_map_base.h"
#include "key_frame.h"

struct GridSubMap
{
    uint id;
    Eigen::Vector3f pose;
    std::shared_ptr<GridMapBase> grid_map;
    std::shared_ptr<KeyFrames> key_frames; // id pose scan scan_context
};


#endif // __GRID_SUBMAP_H__

#ifndef __GRID_SUBMAP_H__
#define __GRID_SUBMAP_H__
#include <iostream>
#include <Eigen/Core>
#include <memory>
#include <vector>
#include "grid_map_base.h"
#include "scan_context.h"

struct GridSubMap
{
    Eigen::Vector3f pose;
    std::shared_ptr<GridMapBase> grid_map;
    std::vector<std::shared_ptr<ScanContext::KeyFrame>> key_frames;
};


class GridSubMaps
{
public:
    struct Options
    {
        uint max_dist = 20.0;
    };
public:
    GridSubMaps(const Options& options)
    :m_options(options)
    {}

    void AddSubMap(const uint& id, std::shared_ptr<GridSubMaps> grid_map)
    {
        m_submaps[id] = grid_map;
    }

private:
    Options m_options;
    std::map<uint, std::shared_ptr<GridSubMaps>> m_submaps;
};


#endif // __GRID_SUBMAP_H__

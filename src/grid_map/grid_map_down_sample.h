#ifndef __GRID_MAP_DOWN_SAMPLE_H__
#define __GRID_MAP_DOWN_SAMPLE_H__
#include <iostream>
#include <Eigen/Core>
#include "grid_map_base.h"
#include<opencv2/opencv.hpp>

class DownSampleMap: public GridMapBase
{
public:
    DownSampleMap(const Params& p);

    DownSampleMap(const GridMapBase& grid_map);

    DownSampleMap(const DownSampleMap& down_map);
};

#endif // __GRID_MAP_DOWN_SAMPLE_H__

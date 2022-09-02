#ifndef __GRID_MAP_MANAGE_H__
#define __GRID_MAP_MANAGE_H__
#include<iostream>
#include<fstream>
#include<sstream>
#include<memory>
#include<Eigen/Core>
#include<opencv2/opencv.hpp>
#include"grid_map_base.h"


class GridMapManage
{
public:
	GridMapManage() = default;

	void SaveGridMap(const std::string& datafile, std::shared_ptr<GridMapBase> grid_map);

	void SaveProbMap(const std::string& datafile, std::shared_ptr<GridMapBase> grid_map);

	void LoadGridMap(const std::string& datafile, std::shared_ptr<GridMapBase>& grid_map);

	void DisplayGridMap(std::shared_ptr<GridMapBase> grid_map, const uint& type);

	void SaveMapPng(std::shared_ptr<GridMapBase> grid_map, const std::string& datafile);
};

#endif // __GRID_MAP_MANAGE_H__


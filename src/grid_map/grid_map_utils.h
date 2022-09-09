#ifndef __GRID_MAP_UTILS_H__
#define __GRID_MAP_UTILS_H__

#include<vector>
#include<Eigen/Core>
#include "grid_map_base.h"
#include "grid_map_down_sample.h"
#include <memory>

class GridMapUtils
{
public:
	GridMapUtils();

	GridMapUtils(std::shared_ptr<GridMapBase> grid_map);

	GridMapUtils(std::shared_ptr<DownSampleMap> down_map);

	std::shared_ptr<DownSampleMap> GetGridMap() const;

	void UpdateByScan(const Eigen::Vector3f& pose_in_world, const std::vector<Eigen::Vector2f>& scan_points);

	Eigen::Vector2f LaserInScaledLaser(const Eigen::Vector2f& laser_point);

	Eigen::Vector2f LaserPointToWorld(const Eigen::Vector2f& point_in_laser, const Eigen::Vector3f& pose_in_world);

	Eigen::Vector3f WorldToMapFloat(const Eigen::Vector3f& pose_in_world) const; 

    Eigen::Vector2f WorldToMapFloat(const Eigen::Vector2f& point_in_world) const;

    Eigen::Vector3f MapToWorldFloat(const Eigen::Vector3f& pose_in_map) const;

    float GetCellProb(const Eigen::Vector2i& point);
private:
	void InverseModel(const Eigen::Vector2i &p0, const Eigen::Vector2i &p1);

	void BresenhamCellOccupied(const Eigen::Vector2i &p);

	void BresenhamCellFree(const Eigen::Vector2i &p0, const Eigen::Vector2i &p1);

	void BrasenHam(int x0, int y0, int x1, int y1);
private:
	std::shared_ptr<DownSampleMap> m_down_map;
};


#endif // __GRID_MAP_UTILS_H__
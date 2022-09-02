#include"grid_map_utils.h"

GridMapUtils::GridMapUtils()
{
	m_down_map = std::make_shared<DownSampleMap>(DownSampleMap::Params());
}

GridMapUtils::GridMapUtils(std::shared_ptr<GridMapBase> grid_map)
{
	m_down_map = std::make_shared<DownSampleMap>(*grid_map);
}

GridMapUtils::GridMapUtils(std::shared_ptr<DownSampleMap> down_map)
{
	m_down_map = std::make_shared<DownSampleMap>(*down_map);
}

std::shared_ptr<DownSampleMap> GridMapUtils::GetGridMap() const
{
	return m_down_map;
}

void GridMapUtils::UpdateByScan(const Eigen::Vector3f& pose_in_world, const std::vector<Eigen::Vector2f>& scan_points)
{
	Eigen::Vector3f pose_in_map = WorldToMapFloat(pose_in_world);
	Eigen::Vector2i begin_point_in_map_int = Eigen::Vector2i(pose_in_map(0), pose_in_map(1));

	for (Eigen::Vector2f point : scan_points)
	{
		Eigen::Vector2f end_point_in_world = LaserPointToWorld(point, pose_in_world);

		Eigen::Vector2f end_point_in_map = WorldToMapFloat(end_point_in_world);

		Eigen::Vector2i end_point_in_map_int(static_cast<int>( ::round(end_point_in_map(0))), static_cast<int>( ::round(end_point_in_map(1))));

		if(begin_point_in_map_int != end_point_in_map_int)
			InverseModel(begin_point_in_map_int, end_point_in_map_int);
	}
}

Eigen::Vector2f GridMapUtils::LaserInScaledLaser(const Eigen::Vector2f& laser_point)
{
	return laser_point / m_down_map->GetResolution();
}

Eigen::Vector2f GridMapUtils::LaserPointToWorld(const Eigen::Vector2f& point_in_laser, const Eigen::Vector3f& pose_in_world)
{
	Eigen::Matrix2f rotation_matrix;
	rotation_matrix << cos(pose_in_world(2)), -sin(pose_in_world(2)),
						sin(pose_in_world(2)), cos(pose_in_world(2));
	Eigen::Vector2f trans(pose_in_world(0), pose_in_world(1));
	return rotation_matrix * point_in_laser + trans;
}

Eigen::Vector3f GridMapUtils::WorldToMapFloat(const Eigen::Vector3f& pose_in_world) const
{
	return Eigen::Vector3f(
        pose_in_world(0) / m_down_map->GetResolution() + m_down_map->GetMapCenter().cast<float>()(0),
        pose_in_world(1) / m_down_map->GetResolution() + m_down_map->GetMapCenter().cast<float>()(1),
        pose_in_world(2));
}

Eigen::Vector2f GridMapUtils::WorldToMapFloat(const Eigen::Vector2f& point_in_world) const
{
	return (point_in_world / m_down_map->GetResolution() + m_down_map->GetMapCenter().cast<float>());
}

Eigen::Vector3f GridMapUtils::MapToWorldFloat(const Eigen::Vector3f& pose_in_map) const
{
	return Eigen::Vector3f(
        (pose_in_map(0) - m_down_map->GetMapCenter().cast<float>()(0)) * m_down_map->GetResolution(),
        (pose_in_map(1) - m_down_map->GetMapCenter().cast<float>()(1)) * m_down_map->GetResolution(),
        pose_in_map(2));
}

float GridMapUtils::GetCellProb(const Eigen::Vector2i& point)
{
	return m_down_map->GetCellProb(point);
}

void GridMapUtils::InverseModel(const Eigen::Vector2i &p0, const Eigen::Vector2i &p1)
{
	BresenhamCellOccupied(p1);

	BresenhamCellFree(p0, p1);
}

void GridMapUtils::BresenhamCellOccupied(const Eigen::Vector2i &p)
{
	m_down_map->SetCellOccupied(p);
}

void GridMapUtils::BresenhamCellFree(const Eigen::Vector2i &p0, const Eigen::Vector2i &p1)
{
	BrasenHam(p0[0], p0[1], p1[0], p1[1]);
}

void GridMapUtils::BrasenHam(int x0, int y0, int x1, int y1)
{
	int dx = ::abs( x1 - x0 );
	int dy = ::abs( y1 - y0 );
	bool inter_change = false;
	int e = -dx;// error
	int signX = x1 > x0 ? 1 : ( ( x1 < x0 ) ? -1 : 0 );
	int signY = y1 > y0 ? 1 : ( ( y1 < y0 ) ? -1 : 0 );
	if (dy > dx) {
		int temp = dx; dx = dy; dy = temp; inter_change = true;
	}

	int x = x0, y = y0;
	for (int i = 1; i <= dx; i++) { 
		m_down_map->SetCellFree(Eigen::Vector2i(x, y));

		if (!inter_change) {x += signX;}
		else {y += signY;}
		e += 2 * dy;
		if (e >= 0) {
			if (!inter_change) {y += signY;}
			else {x += signX;}
			e -= 2 * dx;
		}
	}
}



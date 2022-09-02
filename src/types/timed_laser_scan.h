#ifndef __TYPES_TIMED_LASER_SCAN_H__
#define __TYPES_TIMED_LASER_SCAN_H__

#include<vector>
#include "point2d.h"
#include <Eigen/Core>
#include "sensor_data.h"

struct TimedLaserScan : public SensorData 
{
	TimedLaserScan(uint64_t time, const std::vector<Point2d>& points) 
	: time(time), points(points)
	{}

	TimedLaserScan()
	{
		time = 0;
		points.clear();
	}

	std::vector<Eigen::Vector2f> GetScan()
	{
		std::vector<Eigen::Vector2f> scan;
		for (auto& point : points)
			scan.push_back(point.point());
		
		return scan;
	}

	uint64_t time;
	std::vector<Point2d> points; 
};

#endif // __TYPES_TIMED_LASER_SCAN_H__

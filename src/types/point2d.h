#ifndef __TYPES_POINT2D_H__
#define __TYPES_POINT2D_H__

#include<Eigen/Core>

class Point2d
{
public:
	Point2d(const Eigen::Vector2f& point)
	:m_point(point)
	{}

	Point2d(const float& x, const float& y)
	{
		m_point(0) = x;
		m_point(1) = y;
	}

	Point2d()
	{
		m_point.setZero();
	}

	Point2d& operator=(const Point2d&) = default;

	Point2d(const Point2d&) = default;

	float x() const
	{
		return m_point(0);
	}

	float y() const
	{
		return m_point(1);
	}

	Eigen::Vector2f point() const
	{
		return m_point;
	}
private:
	Eigen::Vector2f m_point;
};

#endif // __TYPES_POINT2D_H__
#ifndef __TYPES_POSE2D_H__
#define __TYPES_POSE2D_H__

#include<Eigen/Core>
#include<cmath>
#include"point2d.h"

class Pose2d
{
public:
	Pose2d(const Eigen::Vector3f& pose)
	{
		normalize(pose);
	}
	
	Pose2d()
	{
		m_pose.setZero();
	}

	Pose2d(const float& x, const float& y, const float& theta)
	{
		normalize(x, y, theta);
	}

	Pose2d(const Pose2d& pose)
	{
		normalize(Eigen::Vector3f(pose.x(), pose.y(), pose.theta()));
	}

	Pose2d& operator=(const Pose2d &) = default;

	Pose2d& operator*(const float& k)
	{
		normalize(k * m_pose);
		return *this;
	}

	float x() const
	{
		return m_pose(0);
	}

	float y() const
	{
		return m_pose(1);
	}

	float theta() const
	{
		return m_pose(2);
	}

	Eigen::Vector3f pose() const
	{
		return m_pose;
	}

	Pose2d TransFormFrom(const Pose2d& new_pose)
	{
		float dx = new_pose.x() - x();
		float dy = new_pose.y() - y();

		return Eigen::Vector3f(
				dx * std::cos(theta()) + dy * std::sin(theta()),
			   -dx * std::sin(theta()) + dy * std::cos(theta()),
			   new_pose.theta() - theta());
	}

	Point2d TransFormFrom(const Point2d& new_point)
	{
		float dx = new_point.x() - x();
		float dy = new_point.y() - y();

		return Point2d(
			dx * std::cos(theta()) + dy * std::sin(theta()),
			-dx * std::sin(theta()) + dy * std::cos(theta()));
	}

	Pose2d TransFormAdd(const Pose2d& delta_pose)
	{
		float dx = delta_pose.x() * std::cos(theta()) - delta_pose.y() * std::sin(theta());
		float dy = delta_pose.x() * std::sin(theta()) + delta_pose.y() * std::cos(theta());
		return Eigen::Vector3f(x() + dx, y() + dy, theta() + delta_pose.theta());
	}

	Point2d TransFormAdd(const Point2d& delta_point)
	{
		float dx = delta_point.x() * std::cos(theta()) - delta_point.y() * std::sin(theta());
		float dy = delta_point.x() * std::sin(theta()) + delta_point.y() * std::cos(theta());
		return Point2d(x() + dx, y() + dy);
	}
private:
	void normalize(const Eigen::Vector3f& pose)
	{
		m_pose = pose;
		while(m_pose(2) > M_PI) m_pose(2) -= 2 * M_PI;
		while(m_pose(2) < -M_PI) m_pose(2) += 2 * M_PI;
	}

	void normalize(const float& x, const float& y, const float& theta)
	{
		m_pose = Eigen::Vector3f(x, y, theta);
		while(m_pose(2) > M_PI) m_pose(2) -= 2 * M_PI;
		while(m_pose(2) < -M_PI) m_pose(2) += 2 * M_PI;
	}
private:
	Eigen::Vector3f m_pose;
};

#endif // __TYPES_POSE2D_H__
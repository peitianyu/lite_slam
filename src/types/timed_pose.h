#ifndef __TYPES_TIMED_POSE_H__
#define __TYPES_TIMED_POSE_H__

#include "pose2d.h"
#include "sensor_data.h"

struct TimedPose : public SensorData
{
	TimedPose(uint64_t _timestamp, const Pose2d &_pose)
		: timestamp(_timestamp), pose(_pose)
	{
	}

	TimedPose() = default;

	TimedPose TransFormFrom(TimedPose &new_pose)
	{
		return TimedPose(new_pose.timestamp - timestamp, pose.TransFormFrom(new_pose.pose));
	}

	TimedPose TransFormAdd(TimedPose &delta_pose)
	{
		return TimedPose(delta_pose.timestamp + timestamp, pose.TransFormAdd(delta_pose.pose));
	}

	uint64_t timestamp;
	Pose2d pose;
};

#endif // __TYPES_TIMED_POSE_H__
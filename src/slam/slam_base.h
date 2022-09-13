#ifndef __TEST_SLAM_BASE_H__
#define __TEST_SLAM_BASE_H__
#include"timed_pose_que.h"
#include"scan_to_map.h"
#include"scan_stamped.h"
#include"grid_map_manage.h"
#include"scan_context.h"
#include"key_frame.h"

class SlamBase
{
public:
	struct Options
	{
		Options()
		{
			max_distance_change = 0.4f;
			max_angle_change = 0.2f;
		}
		float max_distance_change;
        float max_angle_change;
	};

	struct KeyFrameWithScan
	{
		KeyFrameWithScan(const ScanContext::KeyFrame& _key_frame, const std::vector<Eigen::Vector2f>& _scan_points)
		{
			key_frame = _key_frame;
			scan_points = _scan_points;
		}
		ScanContext::KeyFrame key_frame;
		std::vector<Eigen::Vector2f> scan_points;
	};

public:
	SlamBase(const Options& options);

	~SlamBase();

	void Reset();

	void NewOdom(const TimedPose& new_odom);

	void NewScan(const ScanStamped& new_scan);

	Eigen::Vector3f GetEstimate() const;

	virtual void Run(const Pose2d& prior_pose = Pose2d()) = 0;

protected:
	virtual void Init(const Pose2d& prior_pose);

	int LoopClosure(); // 输出loop_id

	void PoseGraphOptimize();

	void UpdateKeyFrame();

	void Location();

	Eigen::Vector3f PredictPriorPose(Eigen::Vector3f& pose);

	bool PoseDiffLargerThan(const Eigen::Vector3f& old_pose, const Eigen::Vector3f& new_pose);
protected:
	Options m_options;
	Eigen::Vector3f m_estimate_pose;
	Eigen::Vector3f m_last_update_pose;
	Pose2d m_last_odom;
	std::unique_ptr<ScanStamped> m_scan;
    std::unique_ptr<TimedPoseQue> m_odom_que;
    std::unique_ptr<ScanToMap> m_scan_matcher;
    std::unique_ptr<GridMapManage> m_map_manager;
    std::unique_ptr<ScanContext> m_scan_context;
    std::unique_ptr<KeyFrames<KeyFrameWithScan>> m_key_frames;
};

#endif // __TEST_SLAM_BASE_H__
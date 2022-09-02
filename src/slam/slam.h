#ifndef __TEST_SLAM_H__
#define __TEST_SLAM_H__
#include"timed_pose_que.h"
#include"scan_to_map.h"
#include"scan_stamped.h"
#include"grid_map_manage.h"
#include"scan_context.h"

class Slam
{
public:
	enum Status
	{
		INIT,
		RELOCATION,
		LOCATION,
		UPDATE,
		LOOPCLOSURE
	};

	struct Options
	{
		enum Mode
		{
			SLAM,
			LOCATION
		};

		Options()
		{
			mode = Mode::LOCATION;
			max_distance_change = 0.4f;
			max_angle_change = 0.2f;
		}

		Mode mode;
		float max_distance_change;
        float max_angle_change;
	};
public:
	Slam(const Options& options);

	~Slam();

	void Reset();

	void NewOdom(const TimedPose& new_odom);

	void NewScan(const ScanStamped& new_scan);

	Eigen::Vector3f GetEstimate() const;

	void Run(const Pose2d& prior_pose = Pose2d());

private:
	void SlamMode(const Pose2d& prior_pose);

	void LocationMode();

	void Init(const Pose2d& prior_pose);

	void Update();

	void RelocationInit();

	void Relocation();

	void Location();

	void AlignPriorPose(const Eigen::Vector3f& prior_pose);

	Eigen::Vector3f PredictPriorPose(Eigen::Vector3f& pose);

	bool PoseDiffLargerThan(const Eigen::Vector3f& old_pose, const Eigen::Vector3f& new_pose);
private:
	Options m_options;
	Status m_status;
	Eigen::Vector3f m_estimate_pose;
	Eigen::Vector3f m_last_pose;
	Pose2d m_last_odom;
	std::unique_ptr<ScanStamped> m_scan;
    std::unique_ptr<TimedPoseQue> m_odom_que;
    std::unique_ptr<ScanToMap> m_scan_matcher;
    std::unique_ptr<GridMapManage> m_map_manager;
    std::unique_ptr<ScanContext> m_scan_context;
};

#endif // __TEST_SLAM_H__
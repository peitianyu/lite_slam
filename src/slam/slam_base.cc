#include"slam_base.h"

SlamBase::SlamBase(const Options& options)
:m_options(options)
{
	m_scan = std::make_unique<ScanStamped>();
	m_odom_que = std::make_unique<TimedPoseQue>(50);
	m_map_manager = std::make_unique<GridMapManage>();
	m_scan_matcher = std::make_unique<ScanToMap>(ScanToMap::Options());
	m_estimate_pose = Eigen::Vector3f::Zero();
	m_last_update_pose = Eigen::Vector3f::Zero();
	m_last_odom = Pose2d();
	m_scan_context = std::make_unique<ScanContext>(ScanContext::Params());
	m_key_frames = std::make_unique<KeyFrames<KeyFrameWithScan>>();
}

SlamBase::~SlamBase()
{
	m_map_manager->SaveGridMap("../log/map.txt", m_scan_matcher->GetGridMap());
	m_map_manager->DisplayGridMap(m_scan_matcher->GetGridMap(), 1);
	m_map_manager->SaveMapPng(m_scan_matcher->GetGridMap(), "../log/map.png");

	m_scan_context->SaveKeyFrame("../log/scan_context.txt");
}

void SlamBase::Reset()
{
	m_scan = std::make_unique<ScanStamped>();
	m_odom_que = std::make_unique<TimedPoseQue>(50);
	m_map_manager = std::make_unique<GridMapManage>();
	m_scan_matcher = std::make_unique<ScanToMap>(ScanToMap::Options());
	m_estimate_pose = Eigen::Vector3f::Zero();
	m_last_update_pose = Eigen::Vector3f::Zero();
	m_last_odom = Pose2d();
	m_scan_context = std::make_unique<ScanContext>(ScanContext::Params());
	m_key_frames = std::make_unique<KeyFrames<KeyFrameWithScan>>();
}

void SlamBase::NewOdom(const TimedPose& new_odom)
{
	m_odom_que->Push_back(new_odom);
}

void SlamBase::NewScan(const ScanStamped& new_scan)
{
	m_scan = std::make_unique<ScanStamped>(new_scan);
}

Eigen::Vector3f SlamBase::GetEstimate() const
{
	return m_estimate_pose;
}

void SlamBase::Init(const Pose2d& prior_pose)
{
    m_scan_matcher->UpdateByScan(prior_pose.pose(), m_scan->GetScanPoints());
    m_last_update_pose = prior_pose.pose();
    m_last_odom = m_odom_que->Back().pose;
}

void SlamBase::UpdateKeyFrame()
{
	if (PoseDiffLargerThan(m_last_update_pose, m_estimate_pose))
	{
		static uint id = 0;
		m_scan_matcher->UpdateByScan(m_estimate_pose, m_scan->GetScanPoints());
		m_last_update_pose = m_estimate_pose;

		ScanContext::KeyFrame scan_context_key_frame = m_scan_context->AddKeyFrame(id++, m_estimate_pose, m_scan->GetScanPoints());
		
		m_key_frames->Push_back(KeyFrameWithScan(scan_context_key_frame, m_scan->GetScanPoints()));
	}
}

int SlamBase::LoopClosure()
{
	int loop_id = m_scan_context->DetectLoopClosure(m_scan->GetScanPoints(), m_estimate_pose);
	if(loop_id != -1){
		m_estimate_pose = m_scan_matcher->ScanMatch(m_scan->Voxelfilter(m_scan->GetScanPoints(), 0.4), m_estimate_pose, 2, 20); 
		m_estimate_pose = m_scan_matcher->ScanMatch(m_scan->Voxelfilter(m_scan->GetScanPoints(), 0.4), m_estimate_pose, 1, 10); 
		m_estimate_pose = m_scan_matcher->ScanMatch(m_scan->Voxelfilter(m_scan->GetScanPoints(), 0.1), m_estimate_pose, 0, 5);
	}
	return loop_id;	
}

void SlamBase::PoseGraphOptimize()
{
	
}

void SlamBase::Location()
{
	m_estimate_pose = m_scan_matcher->ScanMatch(m_scan->Voxelfilter(m_scan->GetScanPoints(), 0.4), m_estimate_pose, 2, 10); 
	m_estimate_pose = m_scan_matcher->ScanMatch(m_scan->Voxelfilter(m_scan->GetScanPoints(), 0.4), m_estimate_pose, 1, 5); 
	m_estimate_pose = m_scan_matcher->ScanMatch(m_scan->Voxelfilter(m_scan->GetScanPoints(), 0.1), m_estimate_pose, 0, 5);
}

Eigen::Vector3f SlamBase::PredictPriorPose(Eigen::Vector3f& pose)
{
	pose = Pose2d(pose).TransFormAdd(m_last_odom.TransFormFrom(m_odom_que->Back().pose)).pose();
	m_last_odom = m_odom_que->Back().pose.pose();
	return pose;
}

bool SlamBase::PoseDiffLargerThan(const Eigen::Vector3f& old_pose, const Eigen::Vector3f& new_pose)
{	
    return ((std::hypot((old_pose - new_pose)(0), (old_pose - new_pose)(1)) > m_options.max_distance_change) 
    		|| (std::fabs((old_pose - new_pose)(2)) > m_options.max_angle_change));                 
}

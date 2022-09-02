#include"slam.h"

Slam::Slam(const Options& options)
:m_options(options)
{
	m_status = Status::INIT;
	m_scan = std::make_unique<ScanStamped>();
	m_odom_que = std::make_unique<TimedPoseQue>(50);
	m_map_manager = std::make_unique<GridMapManage>();
	m_scan_matcher = std::make_unique<ScanToMap>(ScanToMap::Options());
	m_estimate_pose = Eigen::Vector3f::Zero();
	m_last_pose = Eigen::Vector3f::Zero();
	m_last_odom = Pose2d();
	m_scan_context = std::make_unique<ScanContext>(ScanContext::Params());
}

Slam::~Slam()
{
	m_map_manager->SaveGridMap("../log/map.txt", m_scan_matcher->GetGridMap());
	m_map_manager->DisplayGridMap(m_scan_matcher->GetGridMap(), 1);
	m_map_manager->SaveMapPng(m_scan_matcher->GetGridMap(), "../log/map.png");

	m_scan_context->SaveKeyFrame("../log/scan_context.txt");
}

void Slam::Reset()
{
	m_status = Status::INIT;
	m_scan = std::make_unique<ScanStamped>();
	m_odom_que = std::make_unique<TimedPoseQue>(50);
	m_map_manager = std::make_unique<GridMapManage>();
	m_scan_matcher = std::make_unique<ScanToMap>(ScanToMap::Options());
	m_estimate_pose = Eigen::Vector3f::Zero();
	m_last_pose = Eigen::Vector3f::Zero();
	m_last_odom = Pose2d();
	m_scan_context = std::make_unique<ScanContext>(ScanContext::Params());
}

void Slam::NewOdom(const TimedPose& new_odom)
{
	m_odom_que->Push_back(new_odom);
}

void Slam::NewScan(const ScanStamped& new_scan)
{
	m_scan = std::make_unique<ScanStamped>(new_scan);
}

Eigen::Vector3f Slam::GetEstimate() const
{
	return m_estimate_pose;
}

void Slam::Run(const Pose2d& prior_pose)
{
	switch(m_options.mode)
	{
		case Options::Mode::SLAM:
			SlamMode(prior_pose);
			break;
		case Options::Mode::LOCATION:
			LocationMode();
			break;
	}
}

void Slam::SlamMode(const Pose2d& prior_pose)
{
	switch(m_status)
	{
		case Status::INIT:
			Init(prior_pose);
			break;
		case Status::RELOCATION:
			break;
		case Status::LOCATION:
			Location();
		case Status::UPDATE:
			Update();
			break;
		case Status::LOOPCLOSURE:
			break;
	}
}

void Slam::LocationMode()
{
	switch(m_status)
	{
		case Status::INIT:
			RelocationInit();
			break;
		case Status::RELOCATION:
			Relocation();
			break;
		case Status::LOCATION:
			Location();
		case Status::UPDATE:
			Update();
			break;
		case Status::LOOPCLOSURE:
			break;
	}
}

void Slam::Init(const Pose2d& prior_pose)
{
    m_scan_matcher->UpdateByScan(prior_pose.pose(), m_scan->GetScanPoints());
    m_last_pose = prior_pose.pose();
    m_last_odom = m_odom_que->Back().pose;

    static uint cnt = 0;
	if(cnt++ == 20){
		cnt = 0;
		m_status = Status::LOCATION;
	} 
}

void Slam::Update()
{
	static uint cnt = 0;
	std::cout<<cnt++<<" m_estimate_pose: "<<m_estimate_pose.transpose()<<std::endl;
	if (PoseDiffLargerThan(m_last_pose, m_estimate_pose))
	{
		static uint id = 0;
		std::cout<<"UpdateMap!"<<std::endl;
		static uint update_cnt = 0;
		if(update_cnt++ > 5)
			m_scan_matcher->UpdateByScan(m_estimate_pose, m_scan->GetScanPoints());
		m_last_pose = m_estimate_pose;
		m_map_manager->DisplayGridMap(m_scan_matcher->GetGridMap(), 1);

		m_scan_context->AddKeyFrame(id++, m_estimate_pose, m_scan->GetScanPoints());
	}

	m_status = Status::LOCATION;
}

void Slam::RelocationInit()
{
	std::shared_ptr<GridMapBase> grid_map;
	m_map_manager->LoadGridMap("../log/loop_map.txt", grid_map);
	m_scan_matcher = std::make_unique<ScanToMap>(ScanToMap::Options(), grid_map);
	// m_map_manager->DisplayGridMap(m_scan_matcher->GetGridMap(), 1);

	m_scan_context = std::make_unique<ScanContext>(ScanContext::Params());
	m_scan_context->LoadKeyFrame("../log/loop_scan_context.txt");
	m_status = Status::RELOCATION;
}

void Slam::Relocation()
{
	int loop_id = m_scan_context->DetectLoopClosure(m_scan->GetScanPoints(), m_estimate_pose);
	if(loop_id != -1){
		std::cout<<"loop_id: "<<loop_id<<std::endl;
		std::cout<<"m_estimate_pose0: "<<m_estimate_pose.transpose()<<std::endl;
		m_estimate_pose = m_scan_matcher->ScanMatch(m_scan->Voxelfilter(m_scan->GetScanPoints(), 0.4), m_estimate_pose, 2, 20); 
		std::cout<<"m_estimate_pose1: "<<m_estimate_pose.transpose()<<std::endl;
		m_estimate_pose = m_scan_matcher->ScanMatch(m_scan->Voxelfilter(m_scan->GetScanPoints(), 0.4), m_estimate_pose, 1, 10); 
		std::cout<<"m_estimate_pose2: "<<m_estimate_pose.transpose()<<std::endl;
		m_estimate_pose = m_scan_matcher->ScanMatch(m_scan->Voxelfilter(m_scan->GetScanPoints(), 0.1), m_estimate_pose, 0, 5);
		std::cout<<"m_estimate_pose3: "<<m_estimate_pose.transpose()<<std::endl;

		m_status = Status::LOCATION;
	}	
}

void Slam::Location()
{
	m_estimate_pose = m_scan_matcher->ScanMatch(m_scan->Voxelfilter(m_scan->GetScanPoints(), 0.4), m_estimate_pose, 2, 10); 
	m_estimate_pose = m_scan_matcher->ScanMatch(m_scan->Voxelfilter(m_scan->GetScanPoints(), 0.4), m_estimate_pose, 1, 5); 
	m_estimate_pose = m_scan_matcher->ScanMatch(m_scan->Voxelfilter(m_scan->GetScanPoints(), 0.1), m_estimate_pose, 0, 5);

	m_status = Status::UPDATE;
}

Eigen::Vector3f Slam::PredictPriorPose(Eigen::Vector3f& pose)
{
	pose = Pose2d(pose).TransFormAdd(m_last_odom.TransFormFrom(m_odom_que->Back().pose)).pose();
	m_last_odom = m_odom_que->Back().pose.pose();
	return pose;
}

bool Slam::PoseDiffLargerThan(const Eigen::Vector3f& old_pose, const Eigen::Vector3f& new_pose)
{	
    return ((std::hypot((old_pose - new_pose)(0), (old_pose - new_pose)(1)) > m_options.max_distance_change) 
    		|| (std::fabs((old_pose - new_pose)(2)) > m_options.max_angle_change));                 
}

#ifndef __TEST_LOCATION_H__
#define __TEST_LOCATION_H__

#include"slam_base.h"

class PureLocation : public SlamBase
{
public:
	PureLocation(const Options& options)
	:SlamBase(options)
	{}

	void Run(const Pose2d& prior_pose = Pose2d()) override
	{
		static uint is_first = 0;
		if(is_first++ == 20) {Init(prior_pose);}
		
		static bool is_loop_closure = false;
		if(!is_loop_closure){
			if(LoopClosure() != -1) {is_loop_closure = true;}
		}else{
			Location();
			UpdateKeyFrame();

			static uint id = 0;
			std::cout<<id++<<" m_estimate_pose: "<<m_estimate_pose.transpose()<<std::endl;
			m_map_manager->DisplayGridMap(m_scan_matcher->GetGridMap(), 1);
		}
	}

protected:
	void Init(const Pose2d& prior_pose) override
	{
		std::shared_ptr<GridMapBase> grid_map;
		m_map_manager->LoadGridMap("../log/loop_map.txt", grid_map);
		m_scan_matcher = std::make_unique<ScanToMap>(ScanToMap::Options(), grid_map);
		m_scan_context = std::make_unique<ScanContext>(ScanContext::Params());
		m_scan_context->LoadKeyFrame("../log/loop_scan_context.txt");
	}
};

#endif // __TEST_LOCATION_H__
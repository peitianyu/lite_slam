#ifndef __TEST_MAPPER_H__
#define __TEST_MAPPER_H__

#include"slam_base.h"

class Mapper : public SlamBase
{
public:
	Mapper(const Options& options)
	:SlamBase(options)
	{}

	void Run(const Pose2d& prior_pose = Pose2d()) override
	{
		static uint is_first = 0;
		if(is_first++ < 20){
			Init(prior_pose);
			return;
		}

		Location();
		UpdateKeyFrame();

		static uint id = 0;
		std::cout<<id++<<" m_estimate_pose: "<<m_estimate_pose.transpose()<<std::endl;
		m_map_manager->DisplayGridMap(m_scan_matcher->GetGridMap(), 1);
	}
};

#endif // __TEST_MAPPER_H__
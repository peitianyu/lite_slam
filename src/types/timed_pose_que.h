#ifndef __TYPES_TIMED_POSE_QUE_H__
#define __TYPES_TIMED_POSE_QUE_H__

#include"timed_pose.h"
#include<deque>

class TimedPoseQue
{
public:
	TimedPoseQue(std::size_t size)
	:m_size(size)
	{}

	void Push_back(const TimedPose& pose)
	{
		m_pose_que.push_back(pose);
		while(m_pose_que.size() > m_size)
			m_pose_que.pop_front();
	}

	std::size_t Size() const
	{
		return m_pose_que.size();
	}

	const TimedPose& operator[](std::size_t index) const
	{
		return m_pose_que[index];
	}

	const TimedPose& Front() const
	{
		return m_pose_que.front();
	}

	const TimedPose& Back() const
	{
		return m_pose_que.back();
	}

	bool Empty() const
	{
		return m_pose_que.empty();
	}

	TimedPose SpeculateOdom(const uint64_t &timestamp)
	{
		if(m_pose_que.empty())
			return TimedPose();

		TimedPose front = m_pose_que.front();
		TimedPose back = m_pose_que.back();

		if(front.timestamp>=timestamp)
			return front;
		if(back.timestamp<=timestamp)
			return back;

		for(TimedPose& p : m_pose_que)
		{
			if(p.timestamp == timestamp)
				return p;
			
			if(p.timestamp <= timestamp)
				front = p;

			if(p.timestamp >= timestamp){
				back = p;
				break;
			}
		}

		TimedPose d_p(front.TransFormFrom(back));
		float dk = (float)(timestamp - front.timestamp) / d_p.timestamp;
		TimedPose d_pose(timestamp - front.timestamp, d_p.pose*dk);

		return front.TransFormAdd(d_pose);
	}
private:
	std::size_t m_size;
	std::deque<TimedPose> m_pose_que;
};

#endif // __TYPES_TIMED_POSE_QUE_H__
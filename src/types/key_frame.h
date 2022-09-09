#include<vector>
#include<Eigen/Core>

struct KeyFrame
{
	uint id;
	Eigen::Vector3f pose;
	std::vector<Eigen::Vector2f> scan_points;
};

class KeyFrames
{
public:
	KeyFrames(const uint& num)
	:m_num(num)
	{
		if(num <= 0) {m_key_frames.resize(std::numeric_limits<int>::max());}
		else {m_key_frames.resize(num);}
	}

	void Reset()
	{
		if(m_num <= 0) {m_key_frames.resize(std::numeric_limits<int>::max());}
		else {m_key_frames.resize(m_num);}
	}

	KeyFrames& operator=(const KeyFrames&) = default;

	void Push_back(const KeyFrame& key_frame)
	{
		std::vector<KeyFrame>::iterator it = std::lower_bound(m_key_frames.begin(), m_key_frames.end(), key_frame, 
			[](const KeyFrame& a, const KeyFrame& b){
			return a.id < b.id;
		})
		m_key_frames.insert(it, key_frame);

		while(m_key_frames.size() > m_num)
			m_key_frames.pop_front();
	}

	KeyFrame Pop_front() const
	{
		return m_key_frames.pop_front();
	}

	KeyFrame Front() const
	{
		return m_key_frames.front();
	}

	uint Size() const
	{
		return m_key_frames.size();
	}

	bool Empty() const
	{
		return m_key_frames.empty();
	}

private:
	int m_num;
	std::vector<KeyFrame> m_key_frames;
};
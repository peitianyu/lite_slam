#include<vector>
#include<Eigen/Core>

template<typename KeyFrame>
class KeyFrames
{
public:
	KeyFrames() = default;

	KeyFrames& operator=(const KeyFrames&) = default;

	void Push_back(const KeyFrame& key_frame)
	{
		m_key_frames.push_back(key_frame);
	}

	KeyFrame Front() const
	{
		return m_key_frames.front();
	}

	KeyFrame Back() const
	{
		return m_key_frames.back();
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
	std::vector<KeyFrame> m_key_frames;
};
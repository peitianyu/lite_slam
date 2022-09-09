#include<vector>
#include<Eigen/Core>

struct ScanStamped
{
	struct Params
	{
		size_t size = 3600;
		float angle_min = -3.14159;
		float angle_max = 3.14159;
		float angle_increment = 0.00174533;
		float range_min = 0.1f;
		float range_max = 20.0f;
		float scan_increment = 28.5714; // ms
	};

	ScanStamped() = default;

	ScanStamped& operator=(const ScanStamped& ) = default;

	ScanStamped(const Params& params)
	:m_params(params)
	{
		ranges.resize(params.size);
		intensities.resize(params.size);
	}

	std::vector<Eigen::Vector2f> GetScanPoints() const
	{
		std::vector<Eigen::Vector2f> points;
		for(uint i = 0; i < m_params.size; i++)
			if((ranges[i] > m_params.range_min) & (ranges[i] < m_params.range_max))
				{points.push_back(Eigen::Vector2f(ranges[i] * cos(GetAngle(i)), ranges[i] * sin(GetAngle(i))));}
		
		return points;
	}

	std::vector<Eigen::Vector2f> Voxelfilter(const std::vector<Eigen::Vector2f>& scan, const float& resolution)
    {
        std::vector<Eigen::Vector2f> points;
        int index = std::numeric_limits<int>::max();
        std::map<int, std::vector<Eigen::Vector2f>> points_map;
        for(Eigen::Vector2f point: scan)
        {
        	index = static_cast<int>(point(1) / resolution * (m_params.range_max / resolution) + point(0) / resolution);
        	points_map[index].push_back(point);
        }

        for(auto point_pair: points_map)
        {
        	Eigen::Vector2f sum_point = Eigen::Vector2f::Zero();
        	for(Eigen::Vector2f &p:point_pair.second)
        		sum_point += p;

        	points.push_back(sum_point/static_cast<float>(point_pair.second.size()));
        }
        return points;
    }

	float GetAngle(const uint& index) const // -pi ~ pi
	{
		return (m_params.angle_min + index * m_params.angle_increment);
	}

	Params m_params;
	uint64_t timestamp;
	std::vector<float> ranges;
	std::vector<float> intensities;
};
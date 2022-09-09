#ifndef __ICP_MATCH_H__
#define __ICP_MATCH_H__
#include <Eigen/Core>
#include <Eigen/Dense>
#include "nanoflann.hpp"
#include <vector>

class IcpMatch
{
public:
    struct PointCloud
    {
        PointCloud(const std::vector<Eigen::Vector2f>& points) {pts = points;}

        std::vector<Eigen::Vector2f> pts;

        inline size_t kdtree_get_point_count() const { return pts.size(); }

        inline float kdtree_get_pt(const size_t idx, const size_t dim) const {return pts[idx](dim);}

        template <class BBOX>
        bool kdtree_get_bbox(BBOX& /* bb */) const { return false;}
    };
    struct Options
    {
        float max_correspond_distance = 0.2f;
        uint max_iterations = 3;
    };
public:
    IcpMatch(Options options);

    bool Match(const std::vector<Eigen::Vector2f> &first_scan, const std::vector<Eigen::Vector2f> &second_scan,  Eigen::Vector3f &estimate_in_world);

    float GetFitnessScore(const std::vector<Eigen::Vector2f> &first_scan, const std::vector<Eigen::Vector2f> &second_scan);
private:
    void EstimateTransformationOnce(const std::vector<Eigen::Vector2f> &first_scan, const std::vector<Eigen::Vector2f> &second_scan, Eigen::Vector3f &estimate_in_world);

    float FindNearestNeighbor(const Eigen::Vector2f &point, const std::vector<Eigen::Vector2f> &scan_points, Eigen::Vector2f& nearest_point);
    
private:
    Options m_options;
};

#endif // __ICP_MATCH_H__
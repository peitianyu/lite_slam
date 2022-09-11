#ifndef __NDT_MATCH_H__
#define __NDT_MATCH_H__
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include "nanoflann.hpp"

class NdtMatch
{
public:
    struct Options
    {
        uint max_iterations = 5;
        float laser_dist = 14.0f;
        uint col_num = 28;
        uint row_num = 28;
    };

    struct GridCell
    {
        Eigen::Vector2f mean = Eigen::Vector2f::Zero();;
        Eigen::Matrix2f covarince = Eigen::Matrix2f::Zero();
        std::vector<Eigen::Vector2f> points;
    };
public:
    NdtMatch(const Options &options);

    bool Match( const std::vector<Eigen::Vector2f> &first_scan, const std::vector<Eigen::Vector2f> &second_scan, Eigen::Vector3f &robot_pose);  
    
private:
    void CalculateNdtGrid(const std::vector<Eigen::Vector2f> &scan, std::vector<GridCell>& grids);

    void GetHessianDerived(const std::vector<Eigen::Vector2f>& scan, const Eigen::Vector3f& p, const std::vector<GridCell>& grids, Eigen::Matrix3f& H, Eigen::Vector3f& b);

    void EstimateTransformationOnce(const std::vector<Eigen::Vector2f> &scan, const std::vector<GridCell> &grids, Eigen::Vector3f &p);

private:    
    const int PointToGrid( const Eigen::Vector2f &point ) const;

    const Eigen::Vector2f PointTransform( const Eigen::Vector2f &point, const Eigen::Vector3f &d_pose ) const;

    void NormalizePose(Eigen::Vector3f &pose);

private:
    Options m_options;
};

#endif // __NDT_MATCH_H__

#ifndef __SCAN_TO_MAP_H__
#define __SCAN_TO_MAP_H__
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <map>
#include <memory>
#include "grid_map_utils.h"
class ScanToMap
{
public:
    struct Options
    {
        uint layer_num = 3;
        float max_distance_change = 1e-4;
        float max_angle_change = 1e-4;
    };
public:
    ScanToMap(const Options& options);

    ScanToMap(const Options& options, std::shared_ptr<GridMapBase> grid_map);

    const Eigen::Vector3f ScanMatch(const std::vector<Eigen::Vector2f> &scan_points, const Eigen::Vector3f &begin_estimate_in_world,
                                    const uint& map_layer, const uint& max_interation);      

    void UpdateByScan(const Eigen::Vector3f& pose_in_world, const std::vector<Eigen::Vector2f>& scan_points);

    std::shared_ptr<DownSampleMap> GetGridMap() const;
private:
    bool EstimateTransformationOnce(const std::vector<Eigen::Vector2f> &scan_points, std::shared_ptr<GridMapUtils> map_utils, 
                                    Eigen::Vector3f &estimate_in_world);

    void Normalize(Eigen::Vector3f& p);

    void UpdateEstimatedPose(std::shared_ptr<GridMapUtils> map_utils, Eigen::Vector3f &estimate_in_world, Eigen::Vector3f &d_pose_in_map);

    void GetHessianDerivative(const Eigen::Vector3f &robot_in_world, const std::vector<Eigen::Vector2f> &scan_point, 
                                    std::shared_ptr<GridMapUtils> map_utils, Eigen::Matrix3f &H, Eigen::Vector3f &dTr);

    Eigen::Vector3f BilinearInterpolationWithDerivative(const Eigen::Vector2f &point_in_map, std::shared_ptr<GridMapUtils> map_utils) const;

    bool PoseDiffSmallerThan(const Eigen::Vector3f& old_pose, const Eigen::Vector3f& new_pose);

private:
    Options m_options;
    std::vector<std::shared_ptr<GridMapUtils>> m_map_utils;
};

#endif // __SCAN_TO_MAP_H__
#include"scan_to_map.h"

ScanToMap::ScanToMap(const Options& options)
:m_options(options)
{
    m_map_utils.emplace_back(std::make_shared<GridMapUtils>());
    for(uint i = 0; i < m_options.layer_num - 1; i++)
        m_map_utils.emplace_back(std::make_shared<GridMapUtils>(m_map_utils[i]->GetGridMap()));
    
}

ScanToMap::ScanToMap(const Options& options, std::shared_ptr<GridMapBase> grid_map)
:m_options(options)
{
    m_map_utils.emplace_back(std::make_shared<GridMapUtils>(grid_map));
    for(uint i = 0; i < m_options.layer_num - 1; i++)
        m_map_utils.emplace_back(std::make_shared<GridMapUtils>(m_map_utils[i]->GetGridMap()));
    
}

const Eigen::Vector3f ScanToMap::ScanMatch(const std::vector<Eigen::Vector2f> &scan_points, const Eigen::Vector3f &begin_estimate_in_world, 
                                const uint& map_layer, const uint& max_interation)                                  
{
    if(scan_points.empty() || (map_layer > m_options.layer_num - 1))
        return begin_estimate_in_world;

    Eigen::Vector3f estimate_pose(begin_estimate_in_world);

    for(uint i = 0; i < max_interation; i++){
        if(EstimateTransformationOnce(scan_points, m_map_utils[map_layer], estimate_pose)) return estimate_pose;
    }

    return estimate_pose;
}

void ScanToMap::UpdateByScan(const Eigen::Vector3f& pose_in_world, const std::vector<Eigen::Vector2f>& scan_points)
{
    for(uint i = 0; i < m_options.layer_num - 1; i++)
        m_map_utils[i]->UpdateByScan(pose_in_world, scan_points);
}

std::shared_ptr<DownSampleMap> ScanToMap::GetGridMap() const
{
    return m_map_utils[0]->GetGridMap();
}

bool ScanToMap::EstimateTransformationOnce(const std::vector<Eigen::Vector2f> &scan_points, std::shared_ptr<GridMapUtils> map_utils, Eigen::Vector3f &estimate_in_world)
{
    Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
    Eigen::Vector3f dTr = Eigen::Vector3f::Zero();

    Eigen::Vector3f last_estimate_pose = estimate_in_world;
    GetHessianDerivative(estimate_in_world, scan_points, map_utils, H, dTr);

    if(H(0, 0) != 0 || H(1, 1) != 0)
    {
        Eigen::Vector3f d_pose_in_map = H.inverse() * dTr;
        UpdateEstimatedPose(map_utils, estimate_in_world, d_pose_in_map);
    }

    if(PoseDiffSmallerThan(last_estimate_pose, estimate_in_world)) return true;

    return false;
}

void ScanToMap::UpdateEstimatedPose(std::shared_ptr<GridMapUtils> map_utils, Eigen::Vector3f &estimate_in_world, Eigen::Vector3f &d_pose_in_map)
{       
    Eigen::Vector3f pose_in_map = map_utils->WorldToMapFloat(estimate_in_world);

    estimate_in_world = map_utils->MapToWorldFloat(pose_in_map + d_pose_in_map);

    Normalize(estimate_in_world);
}

void ScanToMap::Normalize(Eigen::Vector3f& p)
{
    while(p(2) > M_PI) p(2) -= 2 * M_PI;
	while(p(2) < -M_PI) p(2) += 2 * M_PI;
}

void ScanToMap::GetHessianDerivative(const Eigen::Vector3f &robot_in_world, const std::vector<Eigen::Vector2f> &scan_point, 
                                            std::shared_ptr<GridMapUtils> map_utils, Eigen::Matrix3f &H, Eigen::Vector3f &dTr)
{
    float ss = sin(robot_in_world(2));
    float cs = cos(robot_in_world(2));

    for(uint i = 0; i < scan_point.size(); i++)
    {
        Eigen::Vector2f point_in_scaled_laser = map_utils->LaserInScaledLaser(scan_point[i]);

        Eigen::Vector2f point_in_world = map_utils->LaserPointToWorld(scan_point[i], robot_in_world);

        Eigen::Vector2f point_in_map = map_utils->WorldToMapFloat(point_in_world);

        Eigen::Vector3f interpolated_value = BilinearInterpolationWithDerivative(point_in_map, map_utils);

        float func_val = 1 - interpolated_value(0);
        dTr(0) += func_val * interpolated_value(1);
        dTr(1) += func_val * interpolated_value(2);
        float rot_deriv = (interpolated_value(1) * (-ss * point_in_scaled_laser(0) - cs * point_in_scaled_laser(1)) +
                           interpolated_value(2) * (cs * point_in_scaled_laser(0) - ss * point_in_scaled_laser(1)));
        dTr(2) += rot_deriv * func_val;

        H(0, 0) += interpolated_value(1) * interpolated_value(1);
        H(1, 1) += interpolated_value(2) * interpolated_value(2);
        H(2, 2) += rot_deriv * rot_deriv;

        H(0, 1) += interpolated_value(1) * interpolated_value(2);
        H(0, 2) += interpolated_value(1) * rot_deriv;
        H(1, 2) += interpolated_value(2) * rot_deriv;
    }

    H(1, 0) = H(0, 1);
    H(2, 0) = H(0, 2);
    H(2, 1) = H(1, 2);
}


Eigen::Vector3f ScanToMap::BilinearInterpolationWithDerivative(const Eigen::Vector2f &point_in_map, std::shared_ptr<GridMapUtils> map_utils) const
{   
    float factor0 = point_in_map(0) - floor(point_in_map(0));
    float factor1 = point_in_map(1) - floor(point_in_map(1));
    float factor0_inv = 1 - factor0;
    float factor1_inv = 1 - factor1;

    float p00 = map_utils->GetCellProb(point_in_map.cast<int>());
    float p01 = map_utils->GetCellProb(point_in_map.cast<int>() + Eigen::Vector2i(0, 1));
    float p10 = map_utils->GetCellProb(point_in_map.cast<int>() + Eigen::Vector2i(1, 0));
    float p11 = map_utils->GetCellProb(point_in_map.cast<int>() + Eigen::Vector2i(1, 1));

    return Eigen::Vector3f( ( (factor1 * ( factor0 * p11 + factor0_inv * p01)) + (factor1_inv * ( factor0 * p10 + factor0_inv * p00) ) ),
                                			( factor1 * ( p11 - p01 ) + factor1_inv * ( p10 - p00 ) ),
                                			( factor0 * ( p11 - p10 ) + factor0_inv * ( p01 - p00 ) ));
}

bool ScanToMap::PoseDiffSmallerThan(const Eigen::Vector3f& old_pose, const Eigen::Vector3f& new_pose) 
{   
    // std::cout<<"dist: "<<(std::hypot((old_pose - new_pose)(0), (old_pose - new_pose)(1)))<<" angle: "
    //             <<(std::fabs((old_pose - new_pose)(2)))<<std::endl;
    return ((std::hypot((old_pose - new_pose)(0), (old_pose - new_pose)(1)) < m_options.max_distance_change) 
            && (std::fabs((old_pose - new_pose)(2)) < m_options.max_angle_change));                 
}

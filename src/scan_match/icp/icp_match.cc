#include "icp_match.h"

IcpMatch::IcpMatch(Options options)
:m_options(options)
{

}

bool IcpMatch::Match(const std::vector<Eigen::Vector2f> &first_scan, const std::vector<Eigen::Vector2f> &second_scan, Eigen::Vector3f &estimate_in_world)
{
    if(first_scan.empty() || second_scan.empty())
        {return false;}

    for(uint i = 0; i < m_options.max_iterations; i++){
        EstimateTransformationOnce(first_scan, second_scan, estimate_in_world);
    }

    return true;
}

float IcpMatch::GetFitnessScore(const std::vector<Eigen::Vector2f> &first_scan, const std::vector<Eigen::Vector2f> &second_scan)
{
    float fitness_score = 0.0f;
    for(const Eigen::Vector2f &point:second_scan){
        Eigen::Vector2f nearest_point = Eigen::Vector2f::Zero();
        fitness_score += FindNearestNeighbor(point, first_scan, nearest_point);
    }

    return fitness_score / static_cast<float>(second_scan.size());
}

void IcpMatch::EstimateTransformationOnce(const std::vector<Eigen::Vector2f> &first_scan, const std::vector<Eigen::Vector2f> &second_scan, Eigen::Vector3f &estimate_in_world)
{
    Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
    Eigen::Vector3f dTr = Eigen::Vector3f::Zero();

    for(const Eigen::Vector2f &point:second_scan){
        Eigen::Vector2f nearest_point = Eigen::Vector2f::Zero();
        FindNearestNeighbor(point, first_scan, nearest_point);

        Eigen::Vector2f error = point - nearest_point;
        Eigen::Matrix<float, 2, 3> jacobian = Eigen::Matrix<float, 2, 3>::Zero();
        jacobian << 1.0, 0.0, -point(0) * sin(estimate_in_world(2)) - point(1) * cos(estimate_in_world(2)),
            0.0, 1.0, point(1) * cos(estimate_in_world(2)) - point(0) * sin(estimate_in_world(2));

        H += jacobian.transpose() * jacobian;
        dTr += jacobian.transpose() * error;
    }

    estimate_in_world -= H.inverse() * dTr;
}

float IcpMatch::FindNearestNeighbor(const Eigen::Vector2f &point, const std::vector<Eigen::Vector2f> &scan_points, Eigen::Vector2f& nearest_point)
{
    using kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<float, PointCloud >,
        PointCloud, 2>;
    
    std::vector<Eigen::Vector2f> points;
    points.assign(scan_points.begin(), scan_points.end() - 1);
    PointCloud point_cloud(points);
    kd_tree_t kd_tree(2, point_cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    kd_tree.buildIndex();

    nanoflann::KNNResultSet<float> result_set(1);
    std::vector<size_t> ret_index(1);
    std::vector<float> out_dist_sqr(1);
    result_set.init(&ret_index[0], &out_dist_sqr[0]);
    kd_tree.findNeighbors(result_set, point.data(), nanoflann::SearchParams(10));
    nearest_point = scan_points[ret_index[0]];
    return out_dist_sqr.front();
}
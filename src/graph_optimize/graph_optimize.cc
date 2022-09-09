#include"graph_optimize.h"

GraphOptimize::GraphOptimize(Options options)
    : m_options(options)
{
    m_edges.clear(); 
    m_vertex.clear();
    m_ret_poses.clear();
}

void GraphOptimize::Reset()
{
    m_edges.clear();
    m_vertex.clear();
    m_ret_poses.clear();
}

void GraphOptimize::AddVertex(const uint &id, const Eigen::Vector3f &pose)
{
    m_vertex[id] = pose;
}

void GraphOptimize::AddEdge(const uint &from_id, const uint &to_id, const Eigen::Vector3f &measurement, const Eigen::Matrix3f &info_matrix)
{
    m_edges.push_back(Edge{from_id, to_id, measurement, info_matrix});
}

void GraphOptimize::EraseVertextAndEdge(const uint& id)
{
    std::map<uint, Eigen::Vector3f>::iterator iter = m_vertex.find(id);
    m_vertex.erase(iter);

    for(auto it = m_edges.begin(); it != m_edges.end();)
    {
        if(it->from_id == id || it->to_id == id)
            it = m_edges.erase(it);
    }
}

void GraphOptimize::Optimize()
{
    // Display();
    for(uint i = 0; i < m_options.max_iterations; i++)
        EstimateOnce();
    
    // Display();
}

const std::vector<Eigen::Vector3f> &GraphOptimize::GetOptimizedPoses() const
{
    return m_ret_poses;
}

const std::map<uint, Eigen::Vector3f> &GraphOptimize::GetOptimizedVertexes() const
{
    return m_vertex;
}

void GraphOptimize::EstimateOnce()
{
    // FIXME: 思考scan_points角度不变性,降为2维的可能性
    Eigen::MatrixXf H = Eigen::MatrixXf::Zero(m_vertex.size() * 3, m_vertex.size() * 3);
    Eigen::VectorXf b = Eigen::VectorXf::Zero(m_vertex.size() * 3);
    GetHessianDerived(H, b);

    Eigen::SimplicialLDLT<Eigen::SparseMatrix<float>> solver;
    solver.compute(H.sparseView());
    if(solver.info() != Eigen::Success)
        return;

    Eigen::VectorXf d_x = Eigen::VectorXf::Zero(m_vertex.size() * 3);
    d_x = solver.solve(b);
    if(solver.info() != Eigen::Success)
        return;

    m_ret_poses.clear();
    for(auto &vertice : m_vertex)
    {
        vertice.second += d_x.segment<3>(vertice.first * 3);
        m_ret_poses.push_back(vertice.second);
    }

    NormalizePose();
}

void GraphOptimize::GetHessianDerived(Eigen::MatrixXf &H, Eigen::VectorXf &b)
{
    for(Edge &edge : m_edges)
    {
        Eigen::Matrix3f A; Eigen::Matrix3f B; Eigen::Vector3f e;
        LinearFactors(edge, A, B, e);

        Eigen::Vector3f b_i = -A.transpose() * edge.info_matrix *e;
        Eigen::Vector3f b_j = -B.transpose() * edge.info_matrix *e;
        Eigen::Matrix3f H_ii = A.transpose() * edge.info_matrix * A;
        Eigen::Matrix3f H_ij = A.transpose() * edge.info_matrix * B;
        // Eigen::Matrix3f H_ji = B.transpose() * edge.info_matrix * A;
        Eigen::Matrix3f H_jj = B.transpose() * edge.info_matrix * B;

        H.block<3, 3>(edge.from_id * 3, edge.from_id * 3) += H_ii;
        H.block<3, 3>(edge.from_id * 3, edge.to_id * 3) += H_ij;
        H.block<3, 3>(edge.to_id * 3, edge.from_id * 3) += H_ij.transpose();
        H.block<3, 3>(edge.to_id * 3, edge.to_id * 3) += H_jj;

        b.segment<3>(edge.from_id * 3) += b_i;
        b.segment<3>(edge.to_id * 3) += b_j;
    }

    H(0, 0) = H(1, 1) = H(2, 2) = 1;
}

void GraphOptimize::LinearFactors(const Edge &edge, Eigen::Matrix3f &A, Eigen::Matrix3f &B, Eigen::Vector3f &e)
{
    Eigen::Vector3f from_pose = m_vertex[edge.from_id];
    Eigen::Vector3f to_pose = m_vertex[edge.to_id];
    Eigen::Vector3f d_pose = edge.measurement;

    Eigen::Matrix3f t_from_pose = V2T(from_pose);
    Eigen::Matrix3f t_to_pose = V2T(to_pose);
    Eigen::Matrix3f t_d_pose = V2T(d_pose);

    Eigen::Matrix3f f = t_from_pose.inverse() * t_to_pose;

    Eigen::Vector2f from_point = Eigen::Vector2f(from_pose(0), from_pose(1));
    Eigen::Vector2f to_point = Eigen::Vector2f(to_pose(0), to_pose(1));
    Eigen::Vector2f d_point = to_point - from_point;

    float s = std::sin(from_pose(2));
    float c = std::cos(from_pose(2));

    A << -c, -s, -s * d_point(0) + c * d_point(1),
          s, -c, -c * d_point(0) - s * d_point(1),
          0,  0,                               -1;

    B << c, s, 0,
        -s, c, 0,
         0 ,0, 1;

    Eigen::Matrix3f t_d_pose_inv = t_d_pose.inverse();
    e = T2V(t_d_pose_inv * f);

    t_d_pose_inv(0, 2) = 0;
    t_d_pose_inv(1, 2) = 0;

    A = A * t_d_pose_inv;
    B = B * t_d_pose_inv;
}

Eigen::Matrix3f GraphOptimize::V2T(const Eigen::Vector3f &v)
{
    Eigen::Matrix3f t;
    t << cos(v(2)), -sin(v(2)), v(0),
        sin(v(2)), cos(v(2)), v(1),
        0, 0, 1;
    return t;
}

Eigen::Vector3f GraphOptimize::T2V(const Eigen::Matrix3f &t)
{
    Eigen::Vector3f v;
    v << t(0, 2),
        t(1, 2),
        atan2(t(1, 0), t(0, 0));
    return v;
}

void GraphOptimize::NormalizePose()
{
    for (Eigen::Vector3f &pose : m_ret_poses)
    {
        while(pose(2) > M_PI) pose(2) -= 2 * M_PI;
		while(pose(2) < -M_PI) pose(2) += 2 * M_PI;
    }
}

void GraphOptimize::Display()
{
    constexpr float scale = 2.5f;
    const size_t size = 1200;
    cv::Mat img = cv::Mat::zeros(size, size, CV_8UC3);
    for(auto &vertice : m_vertex)
    {
        cv::Point2f point = cv::Point2f(vertice.second(0) * scale + size / 2, vertice.second(1) * scale + size / 2);
        cv::circle(img, point, 2, cv::Scalar(0, 255, 0), -1);
    }

    for(auto &edge : m_edges)
    {
        cv::Point2f from_point = cv::Point2f(m_vertex[edge.from_id](0) * scale + size / 2, m_vertex[edge.from_id](1) * scale + size / 2);
        cv::Point2f to_point = cv::Point2f(m_vertex[edge.to_id](0) * scale + size / 2, m_vertex[edge.to_id](1) * scale + size / 2);
        cv::line(img, from_point, to_point, cv::Scalar(0, 0, 255), 1);
    }

    cv::imshow("img", img);
    cv::waitKey(0);
}
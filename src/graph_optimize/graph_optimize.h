#ifndef __GRAPH_OPTIMIZE_H__
#define __GRAPH_OPTIMIZE_H__

#include <vector>
#include <map>
#include <memory>
#include <cmath>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Eigen>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseLU>
#include <opencv2/opencv.hpp>

class GraphOptimize
{
public:
    struct Edge
    {
        uint from_id;
        uint to_id;
        Eigen::Vector3f measurement;
        Eigen::Matrix3f info_matrix;
    };

    struct Options
    {
        Options()
        {
            max_iterations = 2;
            edge_num = 10;
            iteration_threshold = 0.02;
            info_matrix << 20.0f, 0, 0,
                            0, 20.0f, 0,
                            0, 0, 100.0f;
        }
        uint edge_num;
        float iteration_threshold;
        uint max_iterations;
        Eigen::Matrix3f info_matrix;
    };
    
public:
    GraphOptimize(Options options);

    void Reset();

    void AddVertex(const uint &id, const Eigen::Vector3f &pose);

    void AddEdge(const uint &from_id, const uint &to_id, const Eigen::Vector3f &measurement, const Eigen::Matrix3f &info_matrix);

    void EraseVertextAndEdge(const uint& id);

    void Optimize();

    const std::vector<Eigen::Vector3f> &GetOptimizedPoses() const;

    const std::map<uint, Eigen::Vector3f> &GetOptimizedVertexes() const;

    void Display();

private:
    void EstimateOnce();

    void GetHessianDerived(Eigen::MatrixXf &H, Eigen::VectorXf &b);

    void LinearFactors(const Edge &edge, Eigen::Matrix3f &A, Eigen::Matrix3f &B, Eigen::Vector3f &e);

    Eigen::Matrix3f V2T(const Eigen::Vector3f& v);

    Eigen::Vector3f T2V(const Eigen::Matrix3f& t);

    void NormalizePose();
private:
    Options m_options;
    std::vector<Edge> m_edges;
    //       uint = id ,  Vector3f = pose
    std::map<uint, Eigen::Vector3f> m_vertex;
    std::vector<Eigen::Vector3f> m_ret_poses;
};

#endif // __POSE_GRAPH_OPTIMIZE_H__
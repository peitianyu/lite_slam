#ifndef __SCAN_CONTEXT_PCA_H
#define __SCAN_CONTEXT_PCA_H

#include <iostream>
#include <Eigen/Dense>
#include <memory>
#include<fstream>
#include<map>
#include<vector>
#include "nanoflann.hpp"

#define KDTREE_DIM 60     //注意这里应该与Params::num_sector相等

class ScanContext
{
public:
    struct KeyFrame
    {
        KeyFrame() = default;
        
        KeyFrame(const size_t &_id, const Eigen::Vector3f &_pose, const Eigen::MatrixXf &_scan_context)
        {
            id = _id;
            key_pose = _pose;
            scan_context = _scan_context;
            key = CalculatePcaForKeyResult();
        }

        KeyFrame(const size_t &_id, const Eigen::Vector3f &_pose, const Eigen::VectorXf& _key, const Eigen::MatrixXf &_scan_context)
        {
            id = _id;
            key_pose = _pose;
            key = _key;
            scan_context = _scan_context;
        }

        size_t id;
        Eigen::Vector3f key_pose;
        Eigen::VectorXf key;
        Eigen::MatrixXf scan_context;
    private:
        Eigen::VectorXf CalculatePcaForKeyResult()
        {
            Eigen::MatrixXf tmp_scan_context = scan_context;
            tmp_scan_context.colwise() -= tmp_scan_context.rowwise().mean();
            Eigen::MatrixXf cov = tmp_scan_context.transpose() * tmp_scan_context * (1 / tmp_scan_context.rows() - 1);
            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eigen_solver(cov);

            return eigen_solver.eigenvectors().rightCols(1);
        }
    };

    struct KeyFrames
    {
        KeyFrames() = default;

        void push_back(const KeyFrame &key_frame) {key_frames.push_back(key_frame);} // TODO: 加入个数限制

        KeyFrame& back() {return key_frames.back();}

        size_t size() const {return key_frames.size();}

        std::vector<KeyFrame> key_frames;

        inline size_t kdtree_get_point_count() const  {return key_frames.size(); }

        inline float kdtree_get_pt(const size_t idx, const size_t dim) const {return key_frames[idx].key(dim);}

        template <class BBOX>
        bool kdtree_get_bbox(BBOX & /* bb */) const{return false;}
    };

    struct Params
    {
        int num_ring  = 60;
        int num_sector = 60;
        float max_range = 30.0;
        float min_range = 0.2;
        size_t num_exclude_recent = 15;
        size_t reconstruct_tree_period = 10;
        size_t leaf_max_size = 10;
        float min_dist = 0.5f;
    };

    ScanContext(const Params& params);

    const int DetectLoopClosure(const std::vector<Eigen::Vector2f> &scan, Eigen::Vector3f &loop_pose);

    const KeyFrame& AddKeyFrame(const size_t &id, const Eigen::Vector3f& curr_pose, const std::vector<Eigen::Vector2f> &scan);

    void SaveKeyFrame(const std::string &file);

    void LoadKeyFrame(const std::string& datafile);
private:
    Eigen::VectorXf CaculateKeyResult(const std::vector<Eigen::Vector2f> &scan);

    Eigen::MatrixXf MakeScanContext(const std::vector<Eigen::Vector2f> &scan);

    void FindNearestNeighbor(const KeyFrame &key, const KeyFrames &key_frames, std::vector<size_t> &nearest_index);
    
    Eigen::Vector3f AlignScanContext(const Eigen::MatrixXf &curr_scan_context, const Eigen::MatrixXf &loop_scan_context);

    Eigen::Vector3f TransformAdd(const Eigen::Vector3f& old_pose, const Eigen::Vector3f& delta_pose);

    void NormalizePose(Eigen::Vector3f& p);

private:
    Params m_params;
    std::unique_ptr<KeyFrames> m_key_frames;
};

#endif
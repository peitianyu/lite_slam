#include"scan_context.h"
#include<opencv2/opencv.hpp>

ScanContext::ScanContext(const Params& params)
: m_params(params)
{
    m_key_frames = std::make_unique<KeyFrames>();
}

const int ScanContext::DetectLoopClosure(const std::vector<Eigen::Vector2f> &scan, Eigen::Vector3f &loop_pose)
{
    // std::cout<<"size: "<<m_key_frames->size()<<std::endl;

    Eigen::MatrixXf scan_context = MakeScanContext(scan);
    KeyFrame key_frame = KeyFrame{std::numeric_limits<size_t>::max(), Eigen::Vector3f::Zero(), scan_context};

    if(m_key_frames->size() < m_params.num_exclude_recent + 1){
        return -1;
    }

    std::vector<size_t> indexes(m_params.leaf_max_size);
    FindNearestNeighbor(key_frame, *m_key_frames, indexes);

    float min_dist = std::numeric_limits<float>::max();
    size_t nearest_id = 0;
    for(size_t &index: indexes){
        float dist = (m_key_frames->key_frames[index].key_pose.head(2) - key_frame.key_pose.head(2)).norm();
        if(dist < min_dist){
            min_dist = dist; nearest_id = index;
        }
    }

    std::cout<<"min_dist: "<<min_dist<<std::endl;

    if(min_dist < m_params.min_dist){
        Eigen::Vector3f delta_pose = AlignScanContext(scan_context, m_key_frames->key_frames[nearest_id].scan_context);
        loop_pose = TransformAdd(m_key_frames->key_frames[nearest_id].key_pose, delta_pose);
        NormalizePose(loop_pose);
        return m_key_frames->key_frames[nearest_id].id;
    }

    return -1;
}

const ScanContext::KeyFrame& ScanContext::AddKeyFrame(const size_t &id, const Eigen::Vector3f& curr_pose, const std::vector<Eigen::Vector2f> &scan)
{
    Eigen::MatrixXf scan_context = MakeScanContext(scan);
    KeyFrame key_frame = KeyFrame{id, curr_pose, scan_context};
    m_key_frames->push_back(key_frame);
    return m_key_frames->back();
}

Eigen::MatrixXf ScanContext::MakeScanContext(const std::vector<Eigen::Vector2f> &scan)
{
    Eigen::MatrixXf scan_context(m_params.num_ring, m_params.num_sector);
    scan_context.setZero();

    for(auto point: scan)
    {
        float angle = std::atan2(point(0), point(1)) / M_PI * 180.0f + 180.0f;
        float dist = std::hypot(point(0), point(1));

        if(dist >= m_params.min_range && dist <= m_params.max_range){
            int ring = std::max(std::min(m_params.num_ring - 1, static_cast<int>(round((dist / m_params.max_range) * m_params.num_ring))), 0);
            int sector = std::max(std::min(m_params.num_sector - 1, static_cast<int>(round((angle / 360.0f) * m_params.num_sector))), 0);

            scan_context(ring, sector) += 1;
        } 
    }
    return scan_context;
}

void ScanContext::FindNearestNeighbor(const KeyFrame &key, const KeyFrames &key_frames, std::vector<size_t> &index)
{
    constexpr size_t num_ring_dim = KDTREE_DIM; // 注意这里应该等于m_params.num_sector

    using kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<float, KeyFrames >,
        KeyFrames, num_ring_dim>;
    
    KeyFrames keys = key_frames;
    keys.key_frames.assign(key_frames.key_frames.begin(), key_frames.key_frames.end() - m_params.num_exclude_recent);
    kd_tree_t kd_tree(num_ring_dim, keys, nanoflann::KDTreeSingleIndexAdaptorParams(m_params.leaf_max_size));
    kd_tree.buildIndex();

    nanoflann::KNNResultSet<float> result_set(m_params.leaf_max_size);
    std::vector<size_t> ret_index(m_params.leaf_max_size);
    std::vector<float> out_dist_sqr(m_params.leaf_max_size);
    result_set.init(&ret_index[0], &out_dist_sqr[0]);
    kd_tree.findNeighbors(result_set, key.key.data(), nanoflann::SearchParams(m_params.leaf_max_size));
    index = ret_index;
}

Eigen::Vector3f ScanContext::AlignScanContext(const Eigen::MatrixXf &curr_scan_context, const Eigen::MatrixXf &loop_scan_context)
{
    Eigen::MatrixXf tmp_scan_context = loop_scan_context;
    float sum = std::numeric_limits<float>::max();
    int d_sector = 0;
    for(int i = 0; i < m_params.num_sector; i++)
    {
        tmp_scan_context.setZero();
        tmp_scan_context.rightCols(i) = loop_scan_context.leftCols(i);
        tmp_scan_context.leftCols(m_params.num_sector - i) = loop_scan_context.rightCols(m_params.num_sector - i);
        
        float d_sum = 0;
        for(int j = 0; j < m_params.num_sector; j++){
            d_sum += fabs(tmp_scan_context.col(j).sum() -  curr_scan_context.col(j).sum());
        }

        if( sum > d_sum){
            sum = d_sum; d_sector = i;
        }
    }

    sum = std::numeric_limits<float>::max();
    int d_ring = 0;
    for(int i = 0; i < m_params.num_ring; i++)
    {
        tmp_scan_context.setZero();
        tmp_scan_context.topRows(i) = loop_scan_context.bottomRows(i);
        tmp_scan_context.bottomRows(m_params.num_ring - i) = loop_scan_context.topRows(m_params.num_ring - i);
        
        float d_sum = 0;
        for(int j = 0; j < m_params.num_ring; j++){
            d_sum += fabs(tmp_scan_context.row(j).sum() -  curr_scan_context.row(j).sum());
        }

        if( sum > d_sum){
            sum = d_sum; d_ring = i;
        }
    }

    int tmp_d_ring = m_params.num_ring - d_ring;
    if(tmp_d_ring > (m_params.num_ring / 2)) tmp_d_ring -= m_params.num_ring;

    
    float angle = static_cast<float>((d_sector + 0.5) * 360 / m_params.num_sector) / 180.0f * M_PI;
    float min_dist = static_cast<float>(tmp_d_ring + 0.5) * m_params.max_range / m_params.num_ring;
    if(angle > M_PI) angle -= 2 * M_PI;
    
    return Eigen::Vector3f(min_dist * cos(angle), min_dist * sin(angle), angle);
}

Eigen::Vector3f ScanContext::TransformAdd(const Eigen::Vector3f& old_pose, const Eigen::Vector3f& delta_pose)
{
    float dx = delta_pose(0) * std::cos(old_pose(2)) - delta_pose(1) * std::sin(old_pose(2));
    float dy = delta_pose(0) * std::sin(old_pose(2)) + delta_pose(1) * std::cos(old_pose(2));
    float angle = old_pose(2) + delta_pose(2);
    return Eigen::Vector3f(old_pose(0) + dx, old_pose(1) + dy, angle);
}

void ScanContext::NormalizePose(Eigen::Vector3f& p)
{
    if(p(2) > M_PI) p(2) -= 2 * M_PI;
    if(p(2) < -M_PI) p(2) += 2 * M_PI;
}

void ScanContext::SaveKeyFrame(const std::string &file)
{
    std::ofstream ofs(file, std::ios::out);

    ofs << "id[1] pose[3] key[60] scan_context[20][60]" << std::endl;
    for(const KeyFrame &key_frame: m_key_frames->key_frames){
        ofs << "KeyFrame: " << key_frame.id << " " << key_frame.key_pose.transpose() << " " << key_frame.key.transpose() << std::endl;
        ofs << key_frame.scan_context << std::endl;
    }
    ofs.close();
}


void ScanContext::LoadKeyFrame(const std::string& datafile)
{
    std::ifstream ifs(datafile);

    std::string line;
    while (std::getline(ifs, line))
    {
        std::istringstream iss(line);
        std::string type;
        iss >> type;
        if(type == "KeyFrame:")
        {
            uint id = 0;
            Eigen::Vector3f key_pose = Eigen::Vector3f::Zero();
            iss >> id >> key_pose(0) >> key_pose(1) >> key_pose(2);
            Eigen::VectorXf key(m_params.num_sector);
            key.setZero();
            for(int col = 0; col < m_params.num_sector; col++) {iss >> key(col); }
            Eigen::MatrixXf scan_context(m_params.num_ring, m_params.num_sector);
            scan_context.setZero();
            int row = 0;
            while(row++ != m_params.num_ring - 1)
            {
                std::getline(ifs, line);
                std::istringstream iss(line);
                for(int col = 0; col < m_params.num_sector; col++) {iss >> scan_context(row, col); }
            }
            // std::cout<<scan_context<<std::endl;
            m_key_frames->push_back(KeyFrame(id, key_pose, key, scan_context));
        }
        // std::cout<<"------------------------------------"<<std::endl;
    }
}
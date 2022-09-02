#include"grid_map_down_sample.h"


DownSampleMap::DownSampleMap(const Params& p)
:GridMapBase(p)
{}

DownSampleMap::DownSampleMap(const GridMapBase& grid_map)
{
    m_params = grid_map.GetParams();
    m_data.setZero();
    m_data = grid_map.GetData();
}

DownSampleMap::DownSampleMap(const DownSampleMap& down_map)
{
    m_params = down_map.GetParams();
    m_params.resolution = down_map.GetResolution() * 2.0;
    m_params.size = Eigen::Vector2i(static_cast<int>(down_map.GetSize()(0) / 2.0 + 0.5), static_cast<int>(down_map.GetSize()(1) / 2.0 + 0.5));
    
    m_data.resize(m_params.size(0), m_params.size(1));
    m_data.setConstant(0);

    for (int row = 0; row < m_params.size(0); row++){
    for (int col = 0; col < m_params.size(1); col++){
        for (int i = 0; i < 2; i++){
        for (int j = 0; j < 2; j++){
            m_data(row, col) = (m_data(row, col) < fabs(down_map.GetCellLogOdds(Eigen::Vector2i(2 * row + i, 2 * col + j)))) ? 
                down_map.GetCellLogOdds(Eigen::Vector2i(2 * row + i, 2 * col + j)):m_data(row, col);   
        }    
        }  
    }
    }
}

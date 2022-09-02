#include"grid_map_base.h"

GridMapBase::GridMapBase(const Params& params)
    : m_params(params)
{
    m_data.resize(m_params.size(0), m_params.size(1));
    m_data.setConstant(0);
}

void GridMapBase::Reset()
{
    m_data.resize(m_params.size(0), m_params.size(1));
    m_data.setConstant(0);
}

float GridMapBase::GetResolution() const
{
    return m_params.resolution;
}

Eigen::Vector2i GridMapBase::GetSize() const
{
    return m_params.size;
}


Eigen::Matrix2i GridMapBase::GetMapLimit() const // top-bottom, left-right
{
    Eigen::Matrix2i map_limit = Eigen::Matrix2i::Zero();
    map_limit << m_params.size(1) , 0, m_params.size(0) , 0; 

    for(int i = 0; i < m_params.size(0); i++){
        for(int j = 0; j < m_params.size(1); j++){
            if(fabs(m_data(i,j)) > 1.0f){
                if(i < map_limit(0,0)) {map_limit(0,0) = i;}
                if(i > map_limit(0,1)) {map_limit(0,1) = i;}
                if(j < map_limit(1,0)) {map_limit(1,0) = j;}
                if(j > map_limit(1,1)) {map_limit(1,1) = j;}
            }
        }
    }
    return map_limit;
}

bool GridMapBase::IsValid(const Eigen::Vector2i& cell_index) const
{
    return (cell_index(0) >= 0 && cell_index(0) < m_params.size(0) && cell_index(1) >= 0 && cell_index(1) < m_params.size(1));
}

const GridMapBase::Params& GridMapBase::GetParams() const
{
    return m_params;
}

const Eigen::MatrixXf& GridMapBase::GetData() const
{
    return m_data;
}

float GridMapBase::GetCellProb(const Eigen::Vector2i& cell_index) const 
{
    if(!IsValid(cell_index)) return 0.5f;

    float odds = std::exp(GetCellLogOdds(cell_index));
    float prob = (odds / (odds + 1));
    if(std::isnan(prob)) return 0;
    return prob;
}

void GridMapBase::SetCellOccupied(const Eigen::Vector2i& cell_index)
{
    if(IsValid(cell_index))
        m_data(cell_index(0), cell_index(1)) += m_params.log_odds_p_occ;
}

void GridMapBase::SetCellFree(const Eigen::Vector2i& cell_index)
{
    if(IsValid(cell_index))
        m_data(cell_index(0), cell_index(1)) -= m_params.log_odds_p_free;
}

void GridMapBase::SetCellValue(const Eigen::Vector2i& cell_index, const float& value)
{
    if(IsValid(cell_index))
        m_data(cell_index(0), cell_index(1)) = value;
}

Eigen::Vector2i GridMapBase::GetMapCenter() const
{
    return Eigen::Vector2i((m_params.origin(0) / m_params.resolution + m_params.size(0)),
                           (m_params.origin(1) / m_params.resolution + m_params.size(1)));
}

float GridMapBase::Prob2LogOdds(const uint& prob) const
{
    return 0;
}

float GridMapBase::Odds2Prob(const float& odds) const
{
    float prob = (odds / (odds + 1));
    if(std::isnan(prob)) return 0.0f;
    return prob;
}

float GridMapBase::GetCellLogOdds(const Eigen::Vector2i& cell_index) const
{
    if(!IsValid(cell_index)) return 0.0f;
    return m_data(cell_index(0), cell_index(1));
}




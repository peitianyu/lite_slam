#ifndef __GRID_MAP_BASE_H__
#define __GRID_MAP_BASE_H__
#include <iostream>
#include <Eigen/Core>

class GridMapBase
{
public:
    struct Params
    {
        float resolution = 0.05; // meter / pixel
        Eigen::Vector2i size = Eigen::Vector2i(2000, 2000); // (rows, cols)
        Eigen::Vector2f origin = Eigen::Vector2f(-60.0f, -60.0f); // left-bottom corner of the map
        float log_odds_p_occ = 0.6f;
        float log_odds_p_free = 0.4f;
    };
public:
    GridMapBase() = default;

    GridMapBase(const Params& params);

    GridMapBase& operator=(const GridMapBase&) = default;

    void Reset();

    float GetResolution() const;

    Eigen::Vector2i GetSize() const;

    Eigen::Matrix2i GetMapLimit() const; 

    const Params& GetParams() const;

    const Eigen::MatrixXf& GetData() const;

    bool IsValid(const Eigen::Vector2i& cell_index) const;

    float GetCellProb(const Eigen::Vector2i& cell_index) const; 

    void SetCellOccupied(const Eigen::Vector2i& cell_index);

    void SetCellFree(const Eigen::Vector2i& cell_index); 

    void SetCellValue(const Eigen::Vector2i& cell_index, const float& value);

    float Prob2LogOdds(const uint& prob) const;

    float Odds2Prob(const float& odds) const;

    float GetCellLogOdds(const Eigen::Vector2i& cell_index) const;

    Eigen::Vector2i GetMapCenter() const;
protected:
    Eigen::MatrixXf m_data;
    Params m_params;
};


#endif // __GRID_MAP_BASE_H__

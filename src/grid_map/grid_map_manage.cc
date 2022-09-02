#include"grid_map_manage.h"

void GridMapManage::SaveProbMap(const std::string& datafile, std::shared_ptr<GridMapBase> grid_map)
{
	Eigen::Matrix2i limit_map = grid_map->GetMapLimit();
	std::ofstream outfile(datafile, std::ios::out);
	outfile << grid_map->GetParams().resolution << " " << grid_map->GetParams().size.transpose() << " " << grid_map->GetParams().origin.transpose()
			 << " " << grid_map->GetParams().log_odds_p_occ << " " << grid_map->GetParams().log_odds_p_free<<" ";

	outfile << limit_map(0, 0) << " "<< limit_map(0, 1) << " "<< limit_map(1, 0) << " "<< limit_map(1, 1) << std::endl;
	
	for (int i = limit_map(0, 0); i < limit_map(0, 1); i++)
	{
		for (int j = limit_map(1, 0); j < limit_map(1, 1); j++)
			{outfile << std::setw(3) << static_cast<int>(255 * grid_map->GetCellProb(Eigen::Vector2i(i, j))) << " ";}
		outfile << std::endl;
	}
	outfile.close();
}

void GridMapManage::SaveGridMap(const std::string& datafile, std::shared_ptr<GridMapBase> grid_map)
{
	Eigen::Matrix2i limit_map = grid_map->GetMapLimit();
	std::ofstream outfile(datafile, std::ios::out);
	outfile << grid_map->GetParams().resolution << " " << grid_map->GetParams().size.transpose() << " " << grid_map->GetParams().origin.transpose()
			 << " " << grid_map->GetParams().log_odds_p_occ << " " << grid_map->GetParams().log_odds_p_free<<" ";

	outfile << limit_map(0, 0) << " "<< limit_map(0, 1) << " "<< limit_map(1, 0) << " "<< limit_map(1, 1) << std::endl;
	
	for (int row = limit_map(0, 0); row < limit_map(0, 1); row++)
	{
		for (int col = limit_map(1, 0); col < limit_map(1, 1); col++)
			{outfile << grid_map->GetData()(row, col) << " ";}
		outfile << std::endl;
	}
	outfile.close();
}

void GridMapManage::LoadGridMap(const std::string& datafile, std::shared_ptr<GridMapBase>& grid_map)
{
	std::ifstream ifs(datafile);
    std::string line;
    std::getline(ifs, line);
    std::istringstream iss(line);
    GridMapBase::Params params;
    iss >> params.resolution >> params.size(0) >> params.size(1) >> params.origin(0) >> params.origin(1) >> params.log_odds_p_occ >> params.log_odds_p_free;
    grid_map = std::make_unique<GridMapBase>(params);
    Eigen::Matrix2i limit_map = Eigen::Matrix2i::Zero();
    iss >> limit_map(0, 0) >> limit_map(0, 1) >> limit_map(1, 0) >> limit_map(1, 1); 
    int row = limit_map(0, 0);
    while (std::getline(ifs, line))
    {
        std::istringstream iss(line);
        float log_odds, col = limit_map(1, 0);
        while(!iss.eof())
        {
        	iss >> log_odds;
        	grid_map->SetCellValue(Eigen::Vector2i(row, col), log_odds);
        	col++;
        }
        row++;
    }
    ifs.close();
}

void GridMapManage::DisplayGridMap(std::shared_ptr<GridMapBase> grid_map, const uint& type)
{
	constexpr int scaler = 2;
	cv::Mat map_mat(grid_map->GetSize()(0) / scaler, grid_map->GetSize()(1) / scaler, CV_8UC1);
	float prob = 0.5;
	for (int i = 0; i < grid_map->GetSize()(0) / scaler ; i++)
	{
		for (int j = 0; j < grid_map->GetSize()(1) / scaler; j++)
		{
			prob = grid_map->GetCellProb(Eigen::Vector2i(i * scaler, j * scaler));
			map_mat.at<uchar>(grid_map->GetSize()(1) / scaler - j - 1, i) = (1 - prob) * 255;
		}
	}
	cv::imshow("map", map_mat);
	cv::waitKey(type);
}

void GridMapManage::SaveMapPng(std::shared_ptr<GridMapBase> grid_map, const std::string& datafile)
{
	constexpr int scaler = 2;
	cv::Mat map_mat(grid_map->GetSize()(0) / scaler, grid_map->GetSize()(1) / scaler, CV_8UC1);
	float prob = 0.5;
	for (int i = 0; i < grid_map->GetSize()(0) / scaler ; i++)
	{
		for (int j = 0; j < grid_map->GetSize()(1) / scaler; j++)
		{
			prob = grid_map->GetCellProb(Eigen::Vector2i(i * scaler, j * scaler));
			map_mat.at<uchar>(j, i) = (1 - prob) * 255;
		}
	}
	cv::imshow("map", map_mat);
	cv::imwrite(datafile, map_mat);
}



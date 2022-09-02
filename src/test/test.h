#ifndef __TEST_H__
#define __TEST_H__

#include<memory>
#include"grid_map_manage.h"
#include"scan_context.h"

class Test
{
public:
	Test()
	{
		m_map_manager = std::make_unique<GridMapManage>();
		m_scan_context = std::make_unique<ScanContext>(ScanContext::Params());
	}


	void TestLoadMap(const std::string& map_file)
	{
		std::shared_ptr<GridMapBase> grid_map;
		m_map_manager->LoadGridMap(map_file, grid_map);
		m_map_manager->SaveGridMap("../log/map1.txt", grid_map);
		m_map_manager->DisplayGridMap(grid_map, 0);
	}

	void TestLoadKeyFrame(const std::string& scan_context_file)
	{
		m_scan_context->LoadKeyFrame(scan_context_file);
		m_scan_context->SaveKeyFrame("../log/scan_context1.txt");
	}


private:
	std::unique_ptr<GridMapManage> m_map_manager;
	std::unique_ptr<ScanContext> m_scan_context;
};


#endif // __TEST_H__
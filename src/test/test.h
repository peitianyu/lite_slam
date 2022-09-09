#ifndef __TEST_H__
#define __TEST_H__

#include<memory>
#include"grid_map_manage.h"
#include"scan_context.h"
#include"ini_parse.h"
#include "graph_optimize.h"
#include "read_graph_data.h"
#include <chrono>

class Test
{
public:
	Test()
	{
		m_map_manager = std::make_unique<GridMapManage>();
		m_scan_context = std::make_unique<ScanContext>(ScanContext::Params());
		m_ini_parse = std::make_unique<IniParse>(IniParse::Param());
		m_graph_optimize = std::make_unique<GraphOptimize>(GraphOptimize::Options());
	}

	void Run()
	{
		// TestLoadMap("../log/map.txt");
		// TestLoadKeyFrame("../log/scan_context.txt");
		// TestIniParse("../cfg/test.ini");
		TestPoseGraphOptimize();
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

	void TestIniParse(const std::string& datafile)
	{
		int a = 0;
		float b = 0.;
		std::string c;
		bool d = 0;

		uint sum = 0;
		sum += m_ini_parse->GetValue(datafile, "Test", "int", a);
		sum += m_ini_parse->GetValue(datafile, "Test", "float", b);
		sum += m_ini_parse->GetValue(datafile, "Test", "string", c);
		sum += m_ini_parse->GetValue(datafile, "Test", "bool", d);

		std::cout<<a<<" "<<b<<" "<<c<<" "<<d<<" "<<sum<<std::endl; 
	}

	void TestPoseGraphOptimize()
	{
		simulation::ReadGraphData simu;
	    simu.openEdgeFile("../data/killian-e.dat");
	    simu.openVertexFile("../data/killian-v.dat");

	    while( simu.getEdgeCount() < 3995 ){
			int id_from = 0;
			int id_to = 0;
			Eigen::Vector3d mean( 0.0, 0.0, 0.0 );
			Eigen::Matrix3d info_matrix = Eigen::Matrix3d::Zero();
			simu.readAEdge( id_from, id_to, mean, info_matrix );

	        m_graph_optimize->AddEdge( id_from, id_to, mean.cast<float>(), info_matrix.cast<float>());
		}

		while( simu.getVertexCount() < 1941 ){
			int vertex_id = 0;
			Eigen::Vector3d pose( 0.0, 0.0, 0.0 );
			simu.readAVertex( vertex_id, pose );

	        m_graph_optimize->AddVertex( vertex_id, pose.cast<float>());
		}

		std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

	    m_graph_optimize->Optimize();

	    std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
		std::chrono::duration<double,std::milli> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);//毫秒
		std::cout<<"time_used: "<<time_used.count()<<" ms"<<std::endl;
	}


private:
	std::unique_ptr<GridMapManage> m_map_manager;
	std::unique_ptr<ScanContext> m_scan_context;
	std::unique_ptr<IniParse> m_ini_parse;
	std::unique_ptr<GraphOptimize> m_graph_optimize;
};


#endif // __TEST_H__
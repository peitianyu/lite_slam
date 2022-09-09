#ifndef __POSE_GRAPH_DATA_H
#define __POSE_GRAPH_DATA_H

#include <iostream>
#include <vector>
#include <fstream>
#include <string.h>
#include <sstream>
#include <iomanip>

#include <Eigen/Dense>

namespace simulation
{

class ReadGraphData
{
public:
	ReadGraphData()	{  }
	~ReadGraphData() {  }

	bool openEdgeFile( const std::string &edge_file_name )
	{
		edge_file.open( edge_file_name.c_str(), std::ifstream::in );
		
		if( !edge_file.is_open() ){
			std::cerr<<"Failed to Open the Edge FILE !"<<std::endl;
			return false;
		}
		
		std::cout<<"Open the Edge FILE !"<<std::endl;
		return true;
	} 

	bool openVertexFile( const std::string &vertex_file_name )
	{
		vertex_file.open( vertex_file_name.c_str(), std::ifstream::in );

		if( !vertex_file.is_open() ){
			std::cerr<<"Failed to Open the Vertex FILE !"<<std::endl;
			return false;
		}

		std::cout<<"Open the Vertex FILE !"<<std::endl;
		return true;
	}

	void closeEdgeFile()
	{
		return edge_file.close();
	}

	void closeVertexFile()
	{
		return vertex_file.close();
	}

	void readAEdge( int &idFrom, int &idTo, Eigen::Vector3d &mean, Eigen::Matrix3d &info_matrix )
	{
		std::string line;
		std::getline( edge_file, line );
		
		std::istringstream iss( line );
		std::string tag;
		iss >> tag;
		std::string num;
		
		if( tag.compare( "EDGE2" ) == 0 ){
			iss >> num;
			idFrom = std::stoi( num );
	
			iss >> num;
			idTo = std::stoi( num );
	
			for( int i = 0; i < 3; i ++ ){
				iss >> num;
				mean[i] = std::stod( num );
			}
		
			double data[6];
			for( int i = 0; i < 6; i ++ ){
				iss >> num;
				data[i] = std::stod( num );
			}
		
			info_matrix( 0, 0 ) = data[0];
			info_matrix( 1, 1 ) = data[2];
			info_matrix( 2, 2 ) = data[3];
			
			edge_count ++;
		}
	}

	void readAVertex( int &id, Eigen::Vector3d &pose )
	{
		std::string line;
                std::getline( vertex_file, line );
	//	std::cout<<line<<std::endl;
	
                std::istringstream iss( line );
                std::string tag;
                iss >> tag;
                std::string num;

                if( tag.compare( "VERTEX2" ) == 0 ){
			iss >> num;
			id = std::stoi( num );

			for( int i = 0; i < 3; i ++ ){
				iss >> num;
				pose[i] = std::stod( num );
			}

			vertex_count ++;
		}
	}

	const int endOfEdgeFile() const
	{
		return edge_file.eof();
	}

	const int endOfVertexFile() const
	{
		return vertex_file.eof();
	}

	const long getEdgeCount() const
	{
		return edge_count;
	}

	const long getVertexCount() const
	{
		return vertex_count;
	}

private:
	std::ifstream edge_file;
	std::ifstream vertex_file;	
	long edge_count = 0;
	long vertex_count = 0;
};

}

#endif

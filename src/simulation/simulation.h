#ifndef __SIMULATION_H__
#define __SIMULATION_H__

#include<iostream>
#include<fstream>
#include<iostream>
#include<fstream>
#include<vector>
#include<memory>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include"timed_pose.h"
#include"slam.h"

class Simulation
{
public:
	Simulation()
	{
		m_slam = std::make_unique<Slam>(Slam::Options());
	}

	void Run(const std::string& scanfile, const std::string& odomfile)
	{
		std::vector<ScanStamped> scan_stampeds = ParseScanData(scanfile);
		std::vector<TimedPose> timed_odoms = ParseOdomData(odomfile);

		std::cout<<"scans.size(): "<<scan_stampeds.size()<<" odoms.size(): "<< timed_odoms.size()<<std::endl;

		bool is_run = true;
		while(!timed_odoms.empty() & !scan_stampeds.empty())
		{
			if(KeyBoardHit()) {if(getchar() == ' ') is_run = is_run? false: true;}

			if(is_run)
			{
				if(timed_odoms.front().timestamp <= scan_stampeds.front().timestamp)
				{
					m_slam->NewOdom(timed_odoms.front());
					timed_odoms.erase(timed_odoms.begin());
				}
				else
				{
					m_slam->NewScan(scan_stampeds.front());
					scan_stampeds.erase(scan_stampeds.begin());

					m_slam->Run();
				}
			}
		}
		
	}
private:

	bool KeyBoardHit()
	{
		struct termios oldt, newt;
	    int ch;
	    int oldf;
	    tcgetattr(STDIN_FILENO, &oldt);
	    newt = oldt;
	    newt.c_lflag &= ~(ICANON | ECHO);
	    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
	    ch = getchar();
	    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	    fcntl(STDIN_FILENO, F_SETFL, oldf);

	    if (ch != EOF)
	    {
	        ungetc(ch, stdin);
	        return 1;
	    }

	    return 0;
	}

	std::vector<ScanStamped> ParseScanData(const std::string& datafile)
    {
        std::vector<ScanStamped> scan_stamps;

        std::ifstream ifs(datafile);
        std::string line;
        while (std::getline(ifs, line))
        {
        	std::istringstream iss(line);

            ScanStamped::Params params;
            uint64_t timestamp;
            iss >> timestamp >> params.size >> params.angle_min >> params.angle_max >> 
                params.angle_increment >> params.range_min >> params.range_max >> params.scan_increment;
            ScanStamped scan(params);

            scan.timestamp = timestamp;
            for(uint i = 0; i < params.size; i++) {iss >> scan.ranges[i];}
            for(uint i = 0; i < params.size; i++) {iss >> scan.intensities[i];}

           	scan_stamps.push_back(scan);
        }
        ifs.close();
        return scan_stamps;
    }

    std::vector<TimedPose> ParseOdomData(const std::string& datafile)
    {
        std::vector<TimedPose> odoms;

        std::ifstream ifs(datafile);
        std::string line;
        while (std::getline(ifs, line))
        {
        	std::istringstream iss(line);

            uint64_t timestamp;
            float x, y, theta;
            iss >> timestamp >> x >> y >> theta;
            odoms.push_back(TimedPose(timestamp, Pose2d(x, y, theta)));
        }
        ifs.close();
        return odoms;
    }
private:
	std::unique_ptr<Slam> m_slam;
};

#endif // __SIMULATION_H__

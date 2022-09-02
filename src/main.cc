#include<iostream>
#include "simulation.h"
#include "test.h"

int main()
{
    Simulation sim;
    sim.Run("../data/loop_scan.txt", "../data/odom.txt");

    // Test test;
    // test.TestLoadMap("../log/map.txt");
    // test.TestLoadKeyFrame("../log/scan_context.txt");
}
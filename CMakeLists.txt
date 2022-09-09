# 指定cmake版本
cmake_minimum_required(VERSION 3.3)
# 工程名
project(slam)

#cmake的c++设置
# 告知當前使用的是交叉編譯方式，必須配置
SET(CMAKE_SYSTEM_NAME Linux)
SET(CMAKE_C_COMPILER "gcc")
SET(CMAKE_CXX_COMPILER "g++")
# 执行路径设置
SET(EXECUTABLE_OUTPUT_PATH ../bin)
# 设置编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -Wall -std=c++14 -fPIC -g")

# 添加eigen库
find_package(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

# 添加opencv库
find_package(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

# 添加.h文件
include_directories(src/simulation
                    src/types
                    src/third_parties
                    src/scan_match/scan_to_map
                    src/scan_match/ndt
                    src/scan_match/icp
                    src/grid_map
                    src/slam
                    src/test
                    src/loop_closure
                    src/graph_optimize
                    src/ini_parse)

# 添加.cpp文件
aux_source_directory(src  SRC)
aux_source_directory(src/simulation SIMULATION)
aux_source_directory(src/types TYPES)
aux_source_directory(src/third_parties THIRD_PARTIES)
aux_source_directory(src/scan_match/scan_to_map SCAN_TO_MAP)
aux_source_directory(src/scan_match/icp ICP)
aux_source_directory(src/scan_match/ndt NDT)
aux_source_directory(src/grid_map GRID_MAP)
aux_source_directory(src/slam SLAM)
aux_source_directory(src/test TEST)
aux_source_directory(src/graph_optimize GRAPH_OPTIMIZE)
aux_source_directory(src/loop_closure LOOP_CLOSURE)

# 执行文件
add_executable(${PROJECT_NAME} ${SRC} ${SIMULATION} ${TYPES} ${THIRD_PARTIES} ${SCAN_TO_MAP} ${GRID_MAP} 
                ${SLAM} ${LOOP_CLOSURE} ${TEST} ${GRAPH_OPTIMIZE} ${ICP} ${NDT})
target_link_libraries(${PROJECT_NAME} ${EIGEN3_LIBRARY} ${OpenCV_LIBS})

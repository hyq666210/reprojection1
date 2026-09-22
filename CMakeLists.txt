cmake_minimum_required(VERSION 3.16)
project(reprojection LANGUAGES CXX)

# ---- C++17 是作业要求，而且必须是标准 C++17（不要 gnu++17 扩展）----
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---- 打开警告。它会帮你发现「变量没用」「有符号无符号比较」这类问题 ----
add_compile_options(-Wall -Wextra -Wpedantic)

# ---- 核心逻辑编成静态库，供主程序和测试共同链接 ----
# 为什么用库？因为 main.cpp 和 test_reprojection.cpp 都要用 camera.cpp 里的函数。
# 做成库，两边链接同一个库就行；直接塞进两个 add_executable 会编两遍，还容易重复定义。
add_library(reprojection_core src/camera.cpp)
target_include_directories(reprojection_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(reprojection_app src/main.cpp)
target_link_libraries(reprojection_app PRIVATE reprojection_core)

add_executable(reprojection_tests tests/test_reprojection.cpp)
target_link_libraries(reprojection_tests PRIVATE reprojection_core)

# ---- 可执行文件统一生成到 build/bin/ 下，路径整齐好记 ----
set_target_properties(reprojection_app reprojection_tests
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# ---- 注册到 CTest，之后一个 ctest 命令就能跑完全部测试 ----
enable_testing()
add_test(NAME unit_tests COMMAND reprojection_tests)
add_test(NAME run_dataset
         COMMAND reprojection_app ${CMAKE_CURRENT_SOURCE_DIR}/data/points.txt)

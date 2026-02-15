cmake_minimum_required(VERSION 3.20)
project(LeadPursuit LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets)

add_executable(LeadPursuit
    src/main.cpp
    src/math/InterceptionSolver.cpp
    src/gui/MainWindow.cpp
    src/gui/InterceptCanvas.cpp
)

target_include_directories(LeadPursuit PRIVATE src)
target_link_libraries(LeadPursuit PRIVATE Qt6::Widgets)

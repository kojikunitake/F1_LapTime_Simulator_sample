cmake_minimum_required(VERSION 3.10)

# Set the project name
project(F1LaptimeSimulator VERSION 1.0 LANGUAGES CXX)

# Use C++17 Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Specify the file and create simulator.exe
add_executable(simulator 
    main.cpp 
    simulator.cpp
    simulator.hpp
)

#include "simulator.hpp"
#include <iostream>

int main() {
    // Relative environment lookup pathing
    std::string input_path = "../../input_data/circuits";
    std::string output_path = "../../output_data";

    std::cout << "=====================================================" << std::endl;
    std::cout << "       F1 Optimal Trajectory Simulator Engine        " << std::endl;
    std::cout << "=====================================================" << std::endl;

    // Instance simulation context
    LaptimeSimulator simulator(input_path, output_path);

    // Target map descriptor string
    std::string target_circuit = "suzuka";

    // Multi-staged data processing routine
    if (simulator.loadCircuit(target_circuit)) {
        bool debug_mode = false; // Set to true to enable telemetry logging
        
        simulator.runSimulation(debug_mode);
        simulator.saveResult(target_circuit);
    }

    std::cout << "=====================================================" << std::endl;
    std::cout << "             Execution Cycle Completed               " << std::endl;
    std::cout << "=====================================================" << std::endl;
    
    return 0;
}
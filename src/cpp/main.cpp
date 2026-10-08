#include "simulator.hpp"
#include <iostream>

int main() {
    // Relative environment lookup pathing
    std::string input_path = "../../input_data/circuits";
    std::string output_path = "../../output_data";

    std::cout << "=====================================================" << std::endl;
    std::cout << "       F1 2-Pass Optimal Trajectory Simulator        " << std::endl;
    std::cout << "=====================================================" << std::endl;

    // Instance simulation context
    LaptimeSimulator simulator(input_path, output_path);

    // Target map descriptor string
    std::string target_circuit = "suzuka";

    // Multi-staged data processing routine
    if (simulator.loadCircuit(target_circuit)) {
        std::cout << "\n[Step 1] Running Baseline Simulation (Centerline)..." << std::endl;
        // If you want to log the initial baseline lap time (centerline trajectory) into telemetry (vmax_log.csv), set this flag to true.
        bool enable_initial_log = false; 
        simulator.runSimulation(enable_initial_log);

        std::cout << "\n[Step 2] Executing Trajectory Optimisation Loop..." << std::endl;
        // Execute the gradient descent optimisation routine to refine the racing line
        simulator.optimiseLineGradient();

        std::cout << "\n[Step 3] Running Final Simulation (Optimised Racing Line)..." << std::endl;
        // If you want to log the final optimised lap time into telemetry (vmax_log.csv), set this flag to true.
        bool enable_final_log = true;
        simulator.runSimulation(enable_final_log);

        std::cout << "\n[Step 4] Exporting Spatial Trajectory Coordinates..." << std::endl;
        // 導き出された理想の2Dレーシングラインの座標データをCSV（suzuka_optimised_line.csv）へ保存
        simulator.saveResult(target_circuit);
    }

    std::cout << "=====================================================" << std::endl;
    std::cout << "             Execution Cycle Completed               " << std::endl;
    std::cout << "=====================================================" << std::endl;
    
    return 0;
}
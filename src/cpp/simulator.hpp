#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP

#include <string>
#include <vector>
#include <utility>
#include <filesystem>

// 2D Vector structure for spatial coordinates
struct Vec2 {
    double x;
    double y;
};

// Structure holding centerline coordinates and left/right boundaries
struct TrackPoint {
    double x, y, z;
    double left_x, left_y;
    double right_x, right_y;
};

// Represents vehicle physics state parameters
struct VehicleState {
    double v = 0.0;        // Speed (m/s)
    double s = 0.0;        // Cumulative distance along the track (m)
    double lateral = 0.0;  // Lateral deviation from center (m)
};

// Hardcore vehicle dynamic properties
struct VehicleParams {
    double mass;              // Vehicle mass (kg)
    double max_drive_force;   // Max propulsion engine force (N)
    double max_brake_force;   // Max mechanical braking force (N)
    double drag_coefficient;  // Aerodynamic drag coefficient (Cd)
    double frontal_area;      // Frontal cross-sectional area (m^2)
    double air_density;       // Ambient air density (kg/m^3)
};

class LaptimeSimulator {
private:
    std::filesystem::path input_dir;
    std::filesystem::path output_dir;
    
    std::vector<TrackPoint> track_data;
    std::vector<double> dist_table;
    std::vector<double> lateral_offsets;
    VehicleParams vehicle_params;
    double dt; // Simulation time step

    // Internal helper methods for track math and coordinate transforms
    std::vector<double> buildDistanceTable(const std::vector<TrackPoint>& track);
    size_t findTrackIndex(double s);
    Vec2 computeOffsetPos(size_t idx, double lateral);
    double curvatureFromLine(const Vec2& A, const Vec2& B, const Vec2& C);
    double computeVmaxFromLine(size_t idx, const std::vector<double>& current_lateral);
    
    // Core physics updater
    void updateVehicle(VehicleState& state, double throttle, double brake);
    
    // Constraints logic
    std::pair<double, double> computeLateralLimit(const TrackPoint& p);
    double clampLateral(double lateral, double max_left, double max_right);
    Vec2 computeLinePos(size_t idx, double lateral);

public:
    LaptimeSimulator(const std::string& input_path, const std::string& output_path);

    // Entry point to load the telemetry database from JSON
    bool loadCircuit(const std::string& circuit_name);

    // Added a flag to control telemetry output via argument
    void runSimulation(bool enable_telemetry = false);

    // Runs Gradient Descent Optimization to solve the racing line
    void optimizeLineGradient();

    // Saves the calculated optimal trajectory into a CSV
    bool saveResult(const std::string& circuit_name);

    // Extended computeLapTime to accept a telemetry output flag for conditional logging
    double computeLapTime(const std::vector<double>& current_lateral, bool enable_telemetry = false);
};

#endif // SIMULATOR_HPP

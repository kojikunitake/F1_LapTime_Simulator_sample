#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP

#include <string>
#include <vector>
#include <utility>
#include <filesystem>

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

// ==== 【HPC EXTENSION: SoA CACHE LAYOUT】 ====
struct TrackSoA {
    size_t size = 0;
    std::vector<double> center_x;
    std::vector<double> center_y;
    std::vector<double> left_x;
    std::vector<double> left_y;
    std::vector<double> right_x;
    std::vector<double> right_y;
    std::vector<double> max_left_limit;
    std::vector<double> max_right_limit;
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

    // Structure of Arrays (SoA) representation for efficient access
    TrackSoA track_soa;

    // 2-Pass Solver Profile Buffers
    std::vector<double> v_max_profile;   // Geometry/G-grip limits
    std::vector<double> v_fwd_profile;   // Forward acceleration profile
    std::vector<double> v_bwd_profile;   // Backward deceleration profile
    std::vector<double> v_final_profile; // Synthesised target velocity profile

    // ADDED FOR G-G TELEMETRY EXTENSION
    std::vector<double> g_long_profile; // Longitudinal G profile
    std::vector<double> g_lat_profile;  // Lateral G profile

    // Internal helper methods for track math and coordinate transforms
    std::vector<double> buildDistanceTable(const std::vector<TrackPoint>& track);
    size_t findTrackIndex(double s);

    // Computes curvature (kappa) from three consecutive points A, B, C using the circumcircle method
    static inline double curvatureFromLine(
        double ax, double ay, // A (Prev)
        double bx, double by, // B (Curr)
        double cx, double cy  // C (Next)
    ) {
        // Calculate distances between points A, B, C
        double dx_ab = bx - ax;
        double dy_ab = by - ay;
        double a = std::sqrt(dx_ab * dx_ab + dy_ab * dy_ab);

        double dx_bc = cx - bx;
        double dy_bc = cy - by;
        double b = std::sqrt(dx_bc * dx_bc + dy_bc * dy_bc);

        double dx_ca = cx - ax;
        double dy_ca = cy - ay;
        double c = std::sqrt(dx_ca * dx_ca + dy_ca * dy_ca);

        // Using Heron's formula to compute the area of the triangle formed by points A, B, C
        double s = (a + b + c) / 2.0;
        double area_sq = s * (s - a) * (s - b) * (s - c);
        double area = std::sqrt(std::max(area_sq, 0.0));

        // Safe guard against zero division and near-zero area
        if (area < 1e-6 || (a * b * c) < 1e-6) {
            return 0.0;
        }
        
        return (4.0 * area) / (a * b * c);
    }

    double computeVmaxFromLine(size_t idx, const std::vector<double>& current_lateral);
    
    // Core physics updater (Legacy loop compatibility)
    void updateVehicle(VehicleState& state, double throttle, double brake);
    
    // Constraints logic
    std::pair<double, double> computeLateralLimit(const TrackPoint& p);
 
    // Internal calculation engine for the Two-Pass Solver

    // Computes the lateral offset position based on the SoA representation and a given lateral deviation alpha
    static inline void computeOffsetPos(
        const TrackSoA& soa,
        size_t idx,
        double alpha,
        double& out_x,
        double& out_y
    ) {
        // Linear interpolation between left (1.0) and right (-1.0) boundaries based on alpha
        double w_left = (alpha + 1.0) * 0.5;
        double w_right = 1.0 - w_left;

        out_x = w_left * soa.left_x[idx] + w_right * soa.right_x[idx];
        out_y = w_left * soa.left_y[idx] + w_right * soa.right_y[idx];
    }

    // Inner loop function to compute corner limits for a specific segment of the track, leveraging SoA for performance
    void computeCornerLimits(
        const TrackSoA& soa,
        size_t start_idx,
        size_t end_idx,
        double* __restrict out_v_max_profile
    );

    void computeForwardPass();
    void computeBackwardPass();
    void synthesiseVelocityProfile();

    // Converts the AoS track representation into a SoA format for SIMD-friendly access
    void convertToSoA();

public:
    LaptimeSimulator(const std::string& input_path, const std::string& output_path);

    // Entry point to load the telemetry database from JSON
    bool loadCircuit(const std::string& circuit_name);

    // Solves for base and optimised lap profile
    void runSimulation(bool initialisation = false, bool enable_telemetry = false);

    // Runs Gradient Descent Optimisation to solve the racing line
    void optimiseLineGradient();

    // Saves the calculated optimal trajectory into a CSV
    bool saveResult(const std::string& circuit_name);
    
    // Integrates and extracts total lap time from the final profile
    double calculateLapTimeFromProfile();

};

#endif // SIMULATOR_HPP

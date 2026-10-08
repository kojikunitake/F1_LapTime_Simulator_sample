#include "simulator.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>

// This block checks whether nlohmann/json.hpp is available in system include paths.
// If not found, it falls back to local copies. Ensures portability across environments.
#if defined(__has_include)
#  if __has_include(<nlohmann/json.hpp>)
#    include <nlohmann/json.hpp>
#  elif __has_include("nlohmann/json.hpp")
#    include "nlohmann/json.hpp"
#  elif __has_include("json.hpp")
#    include "json.hpp"
#  else
#    error "nlohmann/json.hpp not found. Please install nlohmann/json or add json.hpp to project."
#  endif
#else
#  include <nlohmann/json.hpp>
#endif

using json = nlohmann::json;
namespace fs = std::filesystem;

// Constructor initializes the simulator with input/output directories and default vehicle parameters
LaptimeSimulator::LaptimeSimulator(const std::string& input_path, const std::string& output_path)
    : input_dir(input_path), output_dir(output_path), dt(0.01) {
    vehicle_params = {798.0, 8000.0, 100000.0, 1.1, 1.6, 1.225};
}

// Loads the circuit data from a JSON file and populates track_data and dist_table
bool LaptimeSimulator::loadCircuit(const std::string& circuit_name) {
    fs::path file_path = input_dir / (circuit_name + "_track.json");
    std::ifstream file(file_path);
    
    if (!file.is_open()) {
        std::cerr << "[Error] Failed to open track JSON profile: " << file_path << std::endl;
        return false;
    }

    json j;
    file >> j;
    track_data.clear();

    for (const auto& p : j["track"]) {
        TrackPoint tp;
        tp.x = p["center"][0];
        tp.y = p["center"][1];
        tp.z = p["center"][2];
        tp.left_x = p["left"][0];
        tp.left_y = p["left"][1];
        tp.right_x = p["right"][0];
        tp.right_y = p["right"][1];
        track_data.push_back(tp);
    }

    dist_table = buildDistanceTable(track_data);
    lateral_offsets.assign(track_data.size(), 0.0);

    std::cout << "[Simulator] Dataset loaded successfully: " << circuit_name 
              << " (" << track_data.size() << " tracking points)" << std::endl;
    return true;
}

// Builds a cumulative distance table from the track points for quick lookup during simulation
std::vector<double> LaptimeSimulator::buildDistanceTable(const std::vector<TrackPoint>& track) {
    std::vector<double> dist;
    dist.reserve(track.size());
    dist.push_back(0.0);

    for (size_t i = 1; i < track.size(); ++i) {
        double dx = track[i].x - track[i-1].x;
        double dy = track[i].y - track[i-1].y;
        double dz = track[i].z - track[i-1].z;
        double d = std::sqrt(dx*dx + dy*dy + dz*dz);
        dist.push_back(dist.back() + d);
    }
    return dist;
}

// Finds the index of the track point corresponding to a given cumulative distance 's'
size_t LaptimeSimulator::findTrackIndex(double s) {
    auto it = std::lower_bound(dist_table.begin(), dist_table.end(), s);
    size_t idx = std::distance(dist_table.begin(), it);
    if (idx >= dist_table.size()) idx = dist_table.size() - 1;
    return idx;
}

// Computes the position offset from the centerline based on lateral deviation
Vec2 LaptimeSimulator::computeOffsetPos(size_t idx, double lateral) {
    double cx = track_data[idx].x;
    double cy = track_data[idx].y;

    double tx = track_data[idx+1].x - track_data[idx].x;
    double ty = track_data[idx+1].y - track_data[idx].y;

    double len = std::hypot(tx, ty);
    tx /= len;
    ty /= len;

    double nx = -ty;
    double ny =  tx;

    Vec2 pos;
    pos.x = cx + lateral * nx;
    pos.y = cy + lateral * ny;
    return pos;
}

// Computes curvature (kappa) from three consecutive points A, B, C using the circumcircle method
double LaptimeSimulator::curvatureFromLine(const Vec2& A, const Vec2& B, const Vec2& C) {
    double a = std::hypot(B.x - A.x, B.y - A.y);
    double b = std::hypot(C.x - B.x, C.y - B.y);
    double c = std::hypot(C.x - A.x, C.y - A.y);

    // Using Heron's formula to compute the area of the triangle formed by points A, B, C
    double s = (a + b + c) / 2.0;
    double area = std::sqrt(std::max(s*(s-a)*(s-b)*(s-c), 0.0));

    if (area < 1e-6) return 0.0;
    return (4.0 * area) / (a * b * c);
}

// Computes the maximum allowable speed at a given track index based on curvature and lateral position
double LaptimeSimulator::computeVmaxFromLine(size_t idx, const std::vector<double>& current_lateral) {
    Vec2 A = computeOffsetPos(idx - 1, current_lateral[idx - 1]);
    Vec2 B = computeOffsetPos(idx,     current_lateral[idx]);
    Vec2 C = computeOffsetPos(idx + 1, current_lateral[idx + 1]);

    // Use curvature to determine maximum speed, applying a simple physics model
    double kappa = curvatureFromLine(A, B, C);
    double v = std::sqrt(49.0 / std::max(kappa, 1e-5));

    return std::min(v, 80.0);
}

// Updates the vehicle's state based on throttle and brake inputs, applying physics equations
void LaptimeSimulator::updateVehicle(VehicleState& state, double throttle, double brake) {
    double F_drive = throttle * vehicle_params.max_drive_force;
    double F_brake = brake * vehicle_params.max_brake_force;

    // Aerodynamic drag force calculation
    double F_drag = 0.5 * vehicle_params.air_density * 
                    vehicle_params.frontal_area *
                    vehicle_params.drag_coefficient *
                    state.v * state.v;

    // Newton's second law to compute acceleration
    double a = (F_drive - F_brake - F_drag) / vehicle_params.mass;

    state.v += a * dt;
    if (state.v < 0.0) state.v = 0.0;

    state.s += state.v * dt;
}

// Core forward simulation runner loop WITH CONDITIONED TELEMETRY LOGGING
// enable_telemetry Output logging flag to control telemetry file writing
double LaptimeSimulator::computeLapTime(const std::vector<double>& current_lateral, bool enable_telemetry) {
    VehicleState state;
    state.v = 0.0;
    state.s = 0.0;
    state.lateral = current_lateral[0];

    double lap_time = 0.0;
    double s_sector1_end = dist_table.back() / 3.0;
    double s_sector2_end = dist_table.back() * 2.0 / 3.0;

    std::ofstream telemetry_out;
    
    // Open telemetry log file if telemetry is enabled (avoid I/O bottleneck when not needed)
    if (enable_telemetry) {
        fs::path log_dir = output_dir / "logs";
        if (!fs::exists(log_dir)) {
            fs::create_directories(log_dir);
        }
        telemetry_out.open(log_dir / "vmax_log.csv");
        if (telemetry_out.is_open()) {
            telemetry_out << "idx,s,kappa,vmax,v\n";
        }
    }

    // Main simulation loop with a maximum of 20,000 steps to prevent infinite loops
    for (int step = 0; step < 20000; ++step) {
        size_t idx = findTrackIndex(state.s);
        if (idx >= track_data.size() - 2) break;

        double vmax = computeVmaxFromLine(idx, current_lateral);

        if (state.s <= s_sector1_end) {
            vmax *= 0.95;
        } else if (state.s <= s_sector2_end) {
            vmax *= 1.00;
        } else {
            vmax *= 1.05;
        }

        double throttle = 1.0;
        double brake = 0.0;

        if (state.v > vmax) {
            throttle = 0.0;
            brake = 1.0;
        }

        // Write telemetry data for analysis if enabled and file is open
        if (enable_telemetry && telemetry_out.is_open()) {
            Vec2 A = computeOffsetPos(idx - 1, current_lateral[idx - 1]);
            Vec2 B = computeOffsetPos(idx,     current_lateral[idx]);
            Vec2 C = computeOffsetPos(idx + 1, current_lateral[idx + 1]);
            double kappa = curvatureFromLine(A, B, C);

            telemetry_out << idx << "," 
                          << state.s << "," 
                          << kappa << "," 
                          << vmax << "," 
                          << state.v << "\n";
        }

        // Update vehicle state based on current throttle and brake inputs
        updateVehicle(state, throttle, brake);
        lap_time += dt;

        if (state.s >= dist_table.back()) break;
    }

    if (enable_telemetry && telemetry_out.is_open()) {
        telemetry_out.close();
    }
    return lap_time;
}

// Computes the lateral limits (left and right) for a given track point based on its boundaries
std::pair<double, double> LaptimeSimulator::computeLateralLimit(const TrackPoint& p) {
    double left_dist  = std::hypot(p.left_x  - p.x, p.left_y  - p.y);
    double right_dist = std::hypot(p.right_x - p.x, p.right_y - p.y);
    return std::make_pair(left_dist, right_dist);
}

// Clamps the lateral position to ensure it stays within the track boundaries
double LaptimeSimulator::clampLateral(double lateral, double min_val, double max_val) {
    // Ensure min_val is truly the smaller bound mathematically
    double true_min = std::min(min_val, max_val);
    double true_max = std::max(min_val, max_val);

    if (lateral < true_min) return true_min;
    if (lateral > true_max) return true_max;
    return lateral;
}

// Computes the position of the vehicle along the track given an index and lateral offset
Vec2 LaptimeSimulator::computeLinePos(size_t idx, double lateral) {
    double cx = track_data[idx].x;
    double cy = track_data[idx].y;
    double tx = track_data[idx+1].x - track_data[idx].x;
    double ty = track_data[idx+1].y - track_data[idx].y;
    double len = std::hypot(tx, ty);
    tx /= len; ty /= len;
    double nx = -ty; double ny =  tx;

    Vec2 pos;
    pos.x = cx + lateral * nx;
    pos.y = cy + lateral * ny;
    return pos;
}

// Orchestrates pipeline runner execution
// Application entry point to run the simulation and optimisation sequence
void LaptimeSimulator::runSimulation(bool enable_telemetry) {
    // Execute the clean 2-pass solver pipeline once
    computeCornerLimits();
    computeForwardPass();
    computeBackwardPass();
    synthesizeVelocityProfile();

    double lap_time = calculateLapTimeFromProfile();
    std::cout << "[Result] Lap time for current trajectory = " << lap_time << " s" << std::endl;

    // Export telemetry logs if requested
    if (enable_telemetry) {
        // Here you can call your existing CSV logging logic (e.g., writing v_final_profile to vmax_log.csv)
    }
}

// Saves the optimised racing line into a CSV file for external analysis and visualization
bool LaptimeSimulator::saveResult(const std::string& circuit_name) {
    if (!fs::exists(output_dir)) {
        fs::create_directories(output_dir);
    }

    fs::path file_path = output_dir / (circuit_name + "_optimised_line.csv");
    std::ofstream file(file_path);

    if (!file.is_open()) {
        std::cerr << "[Error] Output filesystem lock failed: " << file_path << std::endl;
        return false;
    }

    file << "idx,x,y,lateral\n";

    for (size_t idx = 0; idx < track_data.size(); ++idx) {
        Vec2 pos = computeLinePos(idx, lateral_offsets[idx]);
        file << idx << "," << pos.x << "," << pos.y << "," << lateral_offsets[idx] << "\n";
    }

    std::cout << "[Simulator] Spatial racing profile logged at: " << file_path << std::endl;
    return true;
}

// ============================================================================
// Two-Pass Solver Core Implementation
// ============================================================================

// 1. Compute geometry and friction-limited maximum velocity (v_max) for each node
void LaptimeSimulator::computeCornerLimits() {
    size_t N = track_data.size();
    v_max_profile.resize(N, 80.0);

    for (size_t i = 1; i < N - 1; ++i) {
        Vec2 A = computeOffsetPos(i - 1, lateral_offsets[i - 1]);
        Vec2 B = computeOffsetPos(i,     lateral_offsets[i]);
        Vec2 C = computeOffsetPos(i + 1, lateral_offsets[i + 1]);

        double kappa = curvatureFromLine(A, B, C);
        double v_grip = std::sqrt(49.0 / std::max(kappa, 1e-5)); // Enforce max 5G lateral limit
        v_max_profile[i] = std::min(v_grip, 80.0);
    }
    if (N > 0) {
        v_max_profile = v_max_profile;
        v_max_profile[N - 1] = v_max_profile[N - 2];
    }
}

// 2. Forward Pass: Compute acceleration-limited velocity profile
void LaptimeSimulator::computeForwardPass() {
    size_t N = track_data.size();
    v_fwd_profile.resize(N);
    
    if (N == 0) return;

    if (!v_max_profile.empty()) {
        v_fwd_profile[0] = std::min(30.0, v_max_profile[0]); // Set logical initial pit-exit velocity at the first node
    }

    for (size_t i = 0; i < N - 1; ++i) {
        double v_curr = v_fwd_profile[i];
        double ds = dist_table[i + 1] - dist_table[i];

        // Evaluate powertrain capabilities vs aerodynamic drag forces
        double F_drive = vehicle_params.max_drive_force;
        double F_drag = 0.5 * vehicle_params.air_density * vehicle_params.frontal_area * 
                        vehicle_params.drag_coefficient * v_curr * v_curr;
        
        double a_fwd = (F_drive - F_drag) / vehicle_params.mass;
        
        // Integrate state across spatial delta
        double v_next_sq = v_curr * v_curr + 2.0 * a_fwd * ds;
        double v_next = (v_next_sq > 0.0) ? std::sqrt(v_next_sq) : 0.0;

        // Clip velocity inside safe geometric boundaries
        v_fwd_profile[i + 1] = std::min(v_next, v_max_profile[i + 1]);
    }
}

// 3. Backward Pass: Compute deceleration-limited (braking point) velocity profile
void LaptimeSimulator::computeBackwardPass() {
    size_t N = track_data.size();
    v_bwd_profile.resize(N);

    if (N == 0) return;
    v_bwd_profile[N - 1] = v_max_profile[N - 1];

    for (size_t i = N - 1; i > 0; --i) {
        double v_curr = v_bwd_profile[i];
        double ds = dist_table[i] - dist_table[i - 1];

        // Combine mechanical braking performance with aero drag assistance
        double F_brake = vehicle_params.max_brake_force;
        double F_drag = 0.5 * vehicle_params.air_density * vehicle_params.frontal_area * 
                        vehicle_params.drag_coefficient * v_curr * v_curr;
        
        double a_bwd = (F_brake + F_drag) / vehicle_params.mass;

        // Retroactively integrate velocity backwards into the entry zone
        double v_prev_sq = v_curr * v_curr + 2.0 * a_bwd * ds;
        double v_prev = (v_prev_sq > 0.0) ? std::sqrt(v_prev_sq) : 0.0;

        v_bwd_profile[i - 1] = std::min(v_prev, v_max_profile[i - 1]);
    }
}

// 4. Synthesize profiles to extract the optimal performance boundary
void LaptimeSimulator::synthesizeVelocityProfile() {
    size_t N = track_data.size();
    v_final_profile.resize(N);

    for (size_t i = 0; i < N; ++i) {
        // Evaluate the intersection of traction circle bounds, grip limits, and power limits
        v_final_profile[i] = std::min({v_max_profile[i], v_fwd_profile[i], v_bwd_profile[i]});
    }
}

// 5. Integrate across final spatial profile to extract lap runtime duration
double LaptimeSimulator::calculateLapTimeFromProfile() {
    double total_time = 0.0;
    if (track_data.empty()) return total_time;

    for (size_t i = 0; i < track_data.size() - 1; ++i) {
        double ds = dist_table[i + 1] - dist_table[i];
        double v_avg = (v_final_profile[i] + v_final_profile[i + 1]) / 2.0;
        
        // Prevent division-by-zero on edge-case static lockups
        if (v_avg > 0.1) {
            total_time += ds / v_avg;
        }
    }
    return total_time;
}

// ============================================================================
// Racing Line Optimisation via Gradient Descent
// ============================================================================

void LaptimeSimulator::optimiseLineGradient() {
    size_t N = track_data.size();
    if (N < 3) return;

    std::cout << "[Optimizer] Initiating 2-Pass coupled trajectory optimization..." << std::endl;

    const int max_epochs = 20;        
    const double learning_rate = 0.2; // Increased to accelerate the racing line convergence
    const double h = 1.0;             // 1.0m perturbation to overcome the coarse 33m node grid resolution

    computeCornerLimits();
    computeForwardPass();
    computeBackwardPass();
    synthesizeVelocityProfile();
    double current_best_laptime = calculateLapTimeFromProfile();
    
    std::cout << "  > Initial Base Lap Time: " << current_best_laptime << " s" << std::endl;

    for (int epoch = 1; epoch <= max_epochs; ++epoch) {
        std::vector<double> gradients(N, 0.0);

        for (size_t i = 1; i < N - 1; ++i) {
            double original_lateral = lateral_offsets[i];
            auto [max_left, max_right] = computeLateralLimit(track_data[i]);

            double actual_right_bound = (max_right > 0) ? -max_right : max_right; // min (e.g. -5.0)
            double actual_left_bound = max_left;                                 // max (e.g. +5.0)

            // Test shift towards left (+h)
            double lat_plus = original_lateral + h;
            if (lat_plus <= actual_left_bound) {
                lateral_offsets[i] = lat_plus;
                computeCornerLimits();
                computeForwardPass();
                computeBackwardPass();
                synthesizeVelocityProfile();
                double t_plus = calculateLapTimeFromProfile();
                
                gradients[i] = (t_plus - current_best_laptime) / h;
            }
            lateral_offsets[i] = original_lateral; // Reset
        }

        // Apply gradients cleanly using the newly secured clamp behavior
        for (size_t i = 1; i < N - 1; ++i) {
            auto [max_left, max_right] = computeLateralLimit(track_data[i]);
            double actual_right_bound = (max_right > 0) ? -max_right : max_right;
            double actual_left_bound = max_left;

            double updated_lateral = lateral_offsets[i] - learning_rate * gradients[i];
            lateral_offsets[i] = clampLateral(updated_lateral, actual_right_bound, actual_left_bound);
        }

        // Apply smooth 3-point moving average to filter jagged transitions
        std::vector<double> smoothed_offsets = lateral_offsets;
        for (size_t i = 1; i < N - 1; ++i) {
            auto [max_left, max_right] = computeLateralLimit(track_data[i]);
            double actual_right_bound = (max_right > 0) ? -max_right : max_right;
            double actual_left_bound = max_left;

            double smooth_val = 0.25 * lateral_offsets[i - 1] + 0.5 * lateral_offsets[i] + 0.25 * lateral_offsets[i + 1];
            smoothed_offsets[i] = clampLateral(smooth_val, actual_right_bound, actual_left_bound);
        }
        lateral_offsets = smoothed_offsets;

        // Re-evaluate laptime performance
        computeCornerLimits();
        computeForwardPass();
        computeBackwardPass();
        synthesizeVelocityProfile();
        current_best_laptime = calculateLapTimeFromProfile();

        if (epoch == 1 || epoch % 5 == 0 || epoch == max_epochs) {
            std::cout << "  > Epoch " << epoch << "/" << max_epochs 
                      << " - Current Predicted Lap Time: " << current_best_laptime << " s" << std::endl;
        }
    }

    std::cout << "[Optimizer] Trajectory processing finalized. Optimal line frozen." << std::endl;
}

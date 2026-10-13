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

// Constructor initialises the simulator with input/output directories and default vehicle parameters
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
    size_t N = track_data.size();
    double cx = track_data[idx].x;
    double cy = track_data[idx].y;

    // Wrap around for circular track indexing
    size_t next_idx = (idx + 1) % N;
    double tx = track_data[next_idx].x - track_data[idx].x;
    double ty = track_data[next_idx].y - track_data[idx].y;

    double len = std::hypot(tx, ty);

    // If the segment length is too small, return the centerline position to avoid division by zero
    if (len < 1e-6) {
        return Vec2{cx, cy};
    }

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

// Orchestrates pipeline runner execution
// Application entry point to run the simulation and optimisation sequence
void LaptimeSimulator::runSimulation(bool enable_telemetry) {
    // Execute the clean 2-pass solver pipeline once
    computeCornerLimits();
    computeForwardPass();
    computeBackwardPass();
    synthesiseVelocityProfile();

    double lap_time = calculateLapTimeFromProfile();
    std::cout << "[Result] Lap time for current trajectory = " << lap_time << " s" << std::endl;

    // Export telemetry logs if requested
    if (enable_telemetry) {
        // Here you can call your existing CSV logging logic (e.g., writing v_final_profile to vmax_log.csv)
    }
}

// Saves the optimised racing line into a CSV file for external analysis and visualisation
bool LaptimeSimulator::saveResult(const std::string& circuit_name) {
    std::filesystem::create_directories(output_dir);
    std::filesystem::path file_path = output_dir / (circuit_name + "_optimised_line.csv");

    std::ofstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "[Error] Failed to open file for exporting coordinates: " << file_path << std::endl;
        return false;
    }

    size_t N = track_data.size();
    g_long_profile.resize(N, 0.0);
    g_lat_profile.resize(N, 0.0);

    const double g_accel = 9.81; // Gravity constant
    const double max_friction_g = 5.0; // Explicit F1 Tire Grip Limit (5.0G)

    // STEP 1: Reconstruct the ACTUAL Optimised Racing Line Coordinates
    std::vector<double> opt_x(N, 0.0);
    std::vector<double> opt_y(N, 0.0);

    for (size_t i = 0; i < N; ++i) {
        size_t prev_idx = (i - 1 + N) % N;
        size_t next_idx = (i + 1) % N;
        
        double dx = track_data[next_idx].x - track_data[prev_idx].x;
        double dy = track_data[next_idx].y - track_data[prev_idx].y;
        double mag = std::sqrt(dx*dx + dy*dy);
        if (mag < 1e-5) mag = 1.0;
        
        double nx = -dy / mag;
        double ny = dx / mag;

        opt_x[i] = track_data[i].x + nx * lateral_offsets[i];
        opt_y[i] = track_data[i].y + ny * lateral_offsets[i];
    }

    // STEP 2: Compute Pure Vehicle Dynamics G-Forces with Ellipse Guard
    for (size_t i = 0; i < N; ++i) {
        size_t prev_idx = (i - 1 + N) % N;
        size_t next_idx = (i + 1) % N;
        
        double dx_seg = opt_x[next_idx] - opt_x[i];
        double dy_seg = opt_y[next_idx] - opt_y[i];
        double ds = std::sqrt(dx_seg*dx_seg + dy_seg*dy_seg);
        if (ds < 1e-3) ds = 1.0;

        double v_curr = v_final_profile[i];
        double v_next = v_final_profile[next_idx];

        // Energy-based longitudinal acceleration
        double accel_long = (v_next * v_next - v_curr * v_curr) / (2.0 * ds);
        double raw_long_g = accel_long / g_accel;

        // Trajectory curvature-based lateral acceleration
        Vec2 p_prev = { opt_x[prev_idx], opt_y[prev_idx] };
        Vec2 p_curr = { opt_x[i],        opt_y[i] };
        Vec2 p_next = { opt_x[next_idx], opt_y[next_idx] };

        double actual_curvature = curvatureFromLine(p_prev, p_curr, p_next);
        double raw_lat_g = (v_curr * v_curr * actual_curvature) / g_accel;

        double cross_product = (p_curr.x - p_prev.x) * (p_next.y - p_curr.y) - (p_curr.y - p_prev.y) * (p_next.x - p_curr.x);
        double turn_sign = (cross_product >= 0.0) ? 1.0 : -1.0;
        double signed_lat_g = raw_lat_g * turn_sign;

        // FRICTION ELLIPSE GUARD: Dynamically scale back numerical discretisation spikes
        double combined_g = std::sqrt(raw_long_g * raw_long_g + signed_lat_g * signed_lat_g);
        if (combined_g > max_friction_g && combined_g > 1e-4) {
            double scale = max_friction_g / combined_g;
            raw_long_g *= scale;
            signed_lat_g *= scale;
        }

        g_long_profile[i] = raw_long_g;
        g_lat_profile[i] = signed_lat_g;
    }

    // Write to CSV
    file << "idx,x,y,lateral,vmax,vfwd,vbwd,v,long_g,lat_g\n";
    for (size_t i = 0; i < N; ++i) {
        file << i << ","
             << track_data[i].x << ","
             << track_data[i].y << ","
             << lateral_offsets[i] << ","
             << v_max_profile[i] << ","
             << v_fwd_profile[i] << ","
             << v_bwd_profile[i] << ","
             << v_final_profile[i] << ","
             << g_long_profile[i] << ","  
             << g_lat_profile[i] << "\n"; 
    }
    file.close();
    std::cout << "[Simulator] Spatial racing profile with Aligned & Guarded G-Forces logged successfully." << std::endl;
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

// 4. Synthesise profiles to extract the optimal performance boundary
void LaptimeSimulator::synthesiseVelocityProfile() {
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

    std::cout << "[Optimiser] Initiating 2-Pass coupled trajectory optimization..." << std::endl;

    const int max_epochs = 20;        
    const double learning_rate = 0.15; // Balanced learning rate for robust central convergence
    const double h = 1.0;              // 1.0m perturbation to overcome coarse grid grid resolution

    // 1. Establish the authentic global baseline performance
    computeCornerLimits();
    computeForwardPass();
    computeBackwardPass();
    synthesiseVelocityProfile();
    double current_best_laptime = calculateLapTimeFromProfile();
    
    std::cout << "  > Initial Base Lap Time: " << current_best_laptime << " s" << std::endl;

    for (int epoch = 1; epoch <= max_epochs; ++epoch) {
        std::vector<double> gradients(N, 0.0);

        // STEP 1: Compute High-Fidelity Central Differences Across ALL Nodes (Cyclic Loop)
        for (size_t i = 0; i < N; ++i) {
            double original_lateral = lateral_offsets[i];
            auto [max_left, max_right] = computeLateralLimit(track_data[i]);
            double actual_right_bound = (max_right > 0) ? -max_right : max_right;
            double actual_left_bound = max_left;

            double t_plus = current_best_laptime;
            double t_minus = current_best_laptime;

            // Test Upper Bound Shift (Left: +h)
            double lat_plus = original_lateral + h;
            if (lat_plus <= actual_left_bound) {
                lateral_offsets[i] = lat_plus;
                // Run localised 2-pass update (In a full scalable architecture, compute local window. 
                // For N=172, central evaluation secures perfect physical interaction across long straights)
                computeCornerLimits();
                computeForwardPass();
                computeBackwardPass();
                synthesiseVelocityProfile();
                t_plus = calculateLapTimeFromProfile();
            }
            
            // Test Lower Bound Shift (Right: -h)
            double lat_minus = original_lateral - h;
            if (lat_minus >= actual_right_bound) {
                lateral_offsets[i] = lat_minus;
                computeCornerLimits();
                computeForwardPass();
                computeBackwardPass();
                synthesiseVelocityProfile();
                t_minus = calculateLapTimeFromProfile();
            }

            lateral_offsets[i] = original_lateral; // Strict restoration of spatial state

            // CENTRAL DIFFERENCE EQUATION: Secures smooth dual-sided physical gradients
            gradients[i] = (t_plus - t_minus) / (2.0 * h);
        }

        // STEP 2: Simultaneous Gradient Descent Update with Clamp Enforcement
        for (size_t i = 0; i < N; ++i) {
            auto [max_left, max_right] = computeLateralLimit(track_data[i]);
            double actual_right_bound = (max_right > 0) ? -max_right : max_right;
            double actual_left_bound = max_left;

            double updated_lateral = lateral_offsets[i] - learning_rate * gradients[i];
            lateral_offsets[i] = clampLateral(updated_lateral, actual_right_bound, actual_left_bound);
        }

        // STEP 3: Continuous Cyclical 3-Point Moving Average (Eradicates Boundary Discontinuity)
        std::vector<double> smoothed_offsets = lateral_offsets;
        for (size_t i = 0; i < N; ++i) {
            size_t prev_idx = (i - 1 + N) % N; // Cyclical wrap-around for node 0
            size_t next_idx = (i + 1) % N;     // Cyclical wrap-around for node N-1

            auto [max_left, max_right] = computeLateralLimit(track_data[i]);
            double actual_right_bound = (max_right > 0) ? -max_right : max_right;
            double actual_left_bound = max_left;

            // Laplacian kernel [0.25, 0.50, 0.25]
            double smooth_val = 0.25 * lateral_offsets[prev_idx] + 0.5 * lateral_offsets[i] + 0.25 * lateral_offsets[next_idx];
            smoothed_offsets[i] = clampLateral(smooth_val, actual_right_bound, actual_left_bound);
        }
        lateral_offsets = smoothed_offsets;

        // STEP 4: Synchronise Absolute Core Physics Profiles and Update Master Lap Time
        computeCornerLimits();
        computeForwardPass();
        computeBackwardPass();
        synthesiseVelocityProfile();
        current_best_laptime = calculateLapTimeFromProfile();

        if (epoch == 1 || epoch % 5 == 0 || epoch == max_epochs) {
            std::cout << "  > Epoch " << epoch << "/" << max_epochs 
                      << " - Current Predicted Lap Time: " << current_best_laptime << " s" << std::endl;
        }
    }

    std::cout << "[Optimiser] Trajectory processing finalised. Optimal line frozen." << std::endl;
}

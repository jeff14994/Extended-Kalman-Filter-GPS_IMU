#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include <string>
#include <limits>
#include <sstream>
#include <map>
#include <algorithm>
#include "ekf.h"
#include "geo_ned.h"
#include "utm.h"
using namespace std;
using namespace Eigen;
#include <iomanip>

namespace {
double parse_numeric_field(const std::string& value) {
    if (value.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    size_t start = value.find_first_not_of(" \t\r\n\"");
    if (start == std::string::npos) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    size_t end = value.find_last_not_of(" \t\r\n\"");
    std::string cleaned = value.substr(start, end - start + 1);
    if (cleaned.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    try {
        return std::stod(cleaned);
    } catch (const std::exception&) {
        return std::numeric_limits<double>::quiet_NaN();
    }
}

long long parse_timestamp_field(const std::string& value) {
    double ts = parse_numeric_field(value);
    if (std::isnan(ts)) {
        throw std::invalid_argument("Invalid timestamp");
    }
    return static_cast<long long>(ts);
}

// Trim whitespace from both ends of a string
std::string trim(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    size_t end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

// ── CSV format detection ──────────────────────────────────────────────
// We support two formats:
//   FORMAT_OLD  – localization_log2.csv style
//                 header starts with "timestamp,gps_lat,gps_lon,..."
//   FORMAT_CAN  – CAN-decoded style
//                 header starts with "TimeStamp,0x100_MillisecondsSinceMidnight,..."
enum CsvFormat { FORMAT_OLD, FORMAT_CAN };

struct ColumnMap {
    CsvFormat format;
    // Column indices (0-based, relative to columns AFTER the timestamp column)
    int col_lat;          // GPS latitude
    int col_lon;          // GPS longitude
    int col_speed;        // forward velocity
    int col_yaw;          // yaw angle (degrees) — may be -1 if unavailable
    int col_yaw_rate;     // yaw rate
    // For CAN format: quaternion columns for computing yaw when 0x388_Y is empty
    int col_qw, col_qx, col_qy, col_qz;  // -1 if unavailable
    // Unit conversion factors (applied after reading the raw value)
    double lat_scale;     // multiply raw value by this to get degrees
    double lon_scale;
    double speed_scale;   // multiply raw value by this to get m/s
    double yaw_rate_scale; // multiply raw value by this to get rad/s
};

// Build column map by scanning the header
ColumnMap detect_format(const std::string& header_line) {
    // Parse header into column names (skip the first column which is timestamp)
    std::stringstream ss(header_line);
    std::string token;
    std::getline(ss, token, ','); // skip timestamp column

    std::vector<std::string> col_names;
    while (std::getline(ss, token, ',')) {
        col_names.push_back(trim(token));
    }

    // Build name→index map
    std::map<std::string, int> name_to_idx;
    for (size_t i = 0; i < col_names.size(); ++i) {
        name_to_idx[col_names[i]] = static_cast<int>(i);
    }

    ColumnMap cm{};
    cm.col_qw = cm.col_qx = cm.col_qy = cm.col_qz = -1;
    cm.col_yaw = -1;

    // Detect format
    if (name_to_idx.count("gps_lat") && name_to_idx.count("gps_lon")) {
        // ── Old format ──
        cm.format = FORMAT_OLD;
        cm.col_lat = name_to_idx["gps_lat"];
        cm.col_lon = name_to_idx["gps_lon"];
        cm.col_speed = name_to_idx.count("gps_speed") ? name_to_idx["gps_speed"] : 3;
        cm.col_yaw = name_to_idx.count("imu_yaw") ? name_to_idx["imu_yaw"] : 9;
        cm.col_yaw_rate = name_to_idx.count("imu_yaw_rate") ? name_to_idx["imu_yaw_rate"] : 12;
        cm.lat_scale = 1.0;
        cm.lon_scale = 1.0;
        cm.speed_scale = 1.0;       // already m/s
        cm.yaw_rate_scale = 1.0;    // already rad/s
        std::cerr << "[INFO] Detected OLD format (localization_log2 style)" << std::endl;
    } else if (name_to_idx.count("0x400_Latitude") || name_to_idx.count("0x400_Logitude")) {
        // ── CAN-decoded format ──
        cm.format = FORMAT_CAN;
        cm.col_lat = name_to_idx.count("0x400_Latitude") ? name_to_idx["0x400_Latitude"] : -1;
        cm.col_lon = name_to_idx.count("0x400_Logitude") ? name_to_idx["0x400_Logitude"] : -1;
        // Velocity: prefer 0x408_Velocity (mm/s), fallback to 0x402_Vx
        if (name_to_idx.count("0x408_Velocity")) {
            cm.col_speed = name_to_idx["0x408_Velocity"];
            cm.speed_scale = 0.001;  // mm/s → m/s
        } else if (name_to_idx.count("0x402_Vx")) {
            cm.col_speed = name_to_idx["0x402_Vx"];
            cm.speed_scale = 1.0;    // already m/s
        } else {
            cm.col_speed = -1;
            cm.speed_scale = 1.0;
        }
        // Yaw angle from 0x388_Y (degrees) — may be empty at runtime
        cm.col_yaw = name_to_idx.count("0x388_Y") ? name_to_idx["0x388_Y"] : -1;
        // Yaw rate from gyro Z: 0x288_g_z (deg/s → rad/s)
        cm.col_yaw_rate = name_to_idx.count("0x288_g_z") ? name_to_idx["0x288_g_z"] : -1;
        cm.lat_scale = 1.0e-7;   // integer × 10^7 → degrees
        cm.lon_scale = 1.0e-7;
        cm.yaw_rate_scale = M_PI / 180.0;  // deg/s → rad/s
        // Quaternion columns for computing yaw when 0x388_Y is unavailable
        if (name_to_idx.count("0x488_q_w")) cm.col_qw = name_to_idx["0x488_q_w"];
        if (name_to_idx.count("0x488_q_x")) cm.col_qx = name_to_idx["0x488_q_x"];
        if (name_to_idx.count("0x488_q_y")) cm.col_qy = name_to_idx["0x488_q_y"];
        if (name_to_idx.count("0x488_q_z")) cm.col_qz = name_to_idx["0x488_q_z"];
        std::cerr << "[INFO] Detected CAN format (0x400_Latitude style)" << std::endl;
    } else {
        std::cerr << "[WARN] Unknown CSV format — falling back to old-style column indices" << std::endl;
        cm.format = FORMAT_OLD;
        cm.col_lat = 0;
        cm.col_lon = 1;
        cm.col_speed = 3;
        cm.col_yaw = 9;
        cm.col_yaw_rate = 12;
        cm.lat_scale = 1.0;
        cm.lon_scale = 1.0;
        cm.speed_scale = 1.0;
        cm.yaw_rate_scale = 1.0;
    }
    return cm;
}

// Compute yaw (rad) from quaternion (w, x, y, z) — ZYX Euler yaw
double yaw_from_quaternion(double qw, double qx, double qy, double qz) {
    // atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    return std::atan2(siny_cosp, cosy_cosp);
}

// Safe column access: returns NaN if index is out of range or -1
double safe_get(const std::vector<double>& row, int idx) {
    if (idx < 0 || idx >= static_cast<int>(row.size()))
        return std::numeric_limits<double>::quiet_NaN();
    return row[idx];
}

std::string safe_get_str(const std::vector<std::string>& row, int idx) {
    if (idx < 0 || idx >= static_cast<int>(row.size()))
        return "";
    return row[idx];
}

} // anonymous namespace

int main(int argc, char* argv[]) {
    std::string input_path = "localization_log2.csv";
    if (argc > 1) {
        input_path = argv[1];
    }

    // Create an instance of utmconv
    utmconv::utm_coords coords{};

    std::vector<std::array<double, 3>> obs_trajectory_xyz;
    std::vector<std::array<double, 3>> gt_trajectory_xyz;
    std::vector<double> gt_yaws; // [yaw_angle(rad),] x N
    std::vector<double> obs_yaw_rates; // [vehicle_yaw_rate(rad/s),] x N
    std::vector<double> obs_forward_velocities; // [vehicle_forward_velocity(m/s),] x N
    std::vector<double> ts;
    double last_non_empty_speed = 0.0;
    double last_non_empty_yaw = 0.0;
    double last_non_empty_yaw_rate = 0.0;
    double last_non_empty_lat = 0.0;
    double last_non_empty_lon = 0.0;

    std::ifstream raw_data(input_path);
    if (!raw_data.is_open()) {
        std::cerr << "Failed to open input CSV: " << input_path << std::endl;
        return 1;
    }

    std::string line, value;

    // Read header and detect format
    std::getline(raw_data, line);
    ColumnMap cm = detect_format(line);

    long long int first_timestamp = 0;
    bool first_line = true;

    while (std::getline(raw_data, line)) {
        std::stringstream line_stream(line);
        std::vector<std::string> string_row;
        std::vector<double> row;

        std::getline(line_stream, value, ',');
        long long int timestamp;
        try {
            timestamp = parse_timestamp_field(value);
        } catch (...) {
            continue; // skip malformed lines
        }

        if (first_line) {
            first_timestamp = timestamp;
            first_line = false;
        }
        double elapsed_time = (timestamp - first_timestamp) / 1000000.0;
        ts.push_back(elapsed_time);

        while (std::getline(line_stream, value, ',')) {
            string_row.push_back(value);
            row.push_back(parse_numeric_field(value));
        }

        // ── Extract fields using column map ──
        double raw_lat = safe_get(row, cm.col_lat);
        double raw_lon = safe_get(row, cm.col_lon);
        double lat = raw_lat * cm.lat_scale;
        double lon = raw_lon * cm.lon_scale;
        bool has_gps = !std::isnan(raw_lat) && !std::isnan(raw_lon)
                       && !safe_get_str(string_row, cm.col_lat).empty()
                       && !safe_get_str(string_row, cm.col_lon).empty();

        // Speed
        double raw_speed = safe_get(row, cm.col_speed);
        double speed = std::isnan(raw_speed) ? last_non_empty_speed : raw_speed * cm.speed_scale;

        // Yaw rate
        double raw_yaw_rate = safe_get(row, cm.col_yaw_rate);
        double yaw_rate = std::isnan(raw_yaw_rate) ? last_non_empty_yaw_rate : raw_yaw_rate * cm.yaw_rate_scale;

        // Yaw angle (degrees → rad)
        double yaw_deg;
        if (cm.col_yaw >= 0) {
            double raw_yaw = safe_get(row, cm.col_yaw);
            yaw_deg = std::isnan(raw_yaw) ? last_non_empty_yaw : raw_yaw;
        } else if (cm.col_qw >= 0 && cm.col_qx >= 0 && cm.col_qy >= 0 && cm.col_qz >= 0) {
            // Compute yaw from quaternion
            double qw = safe_get(row, cm.col_qw);
            double qx = safe_get(row, cm.col_qx);
            double qy = safe_get(row, cm.col_qy);
            double qz = safe_get(row, cm.col_qz);
            if (!std::isnan(qw) && !std::isnan(qx) && !std::isnan(qy) && !std::isnan(qz)) {
                // yaw_from_quaternion returns radians; convert to degrees for consistency
                yaw_deg = yaw_from_quaternion(qw, qx, qy, qz) * 180.0 / M_PI;
            } else {
                yaw_deg = last_non_empty_yaw;
            }
        } else {
            yaw_deg = last_non_empty_yaw;
        }

        // ── Populate vectors ──
        if (!has_gps) {
            // No GPS this row — use last known position for ground truth
            utmconv::geodetic_to_utm(last_non_empty_lat, last_non_empty_lon, coords);
            gt_trajectory_xyz.push_back({coords.easting, coords.northing});
            // Push NaN for observed trajectory so EKF skips the update step
            obs_trajectory_xyz.push_back({std::numeric_limits<double>::quiet_NaN(),
                                          std::numeric_limits<double>::quiet_NaN()});
            obs_forward_velocities.push_back(speed);
            last_non_empty_yaw_rate = yaw_rate;
            last_non_empty_yaw = yaw_deg;
            gt_yaws.push_back(deg2rad(yaw_deg));
            obs_yaw_rates.push_back(yaw_rate);
        } else {
            // GPS available
            last_non_empty_lat = lat;
            last_non_empty_lon = lon;
            utmconv::geodetic_to_utm(lat, lon, coords);
            obs_trajectory_xyz.push_back({coords.easting, coords.northing});
            gt_trajectory_xyz.push_back({coords.easting, coords.northing});
            last_non_empty_speed = speed;
            obs_forward_velocities.push_back(speed);
            last_non_empty_yaw = yaw_deg;
            last_non_empty_yaw_rate = yaw_rate;
            gt_yaws.push_back(deg2rad(yaw_deg));
            obs_yaw_rates.push_back(yaw_rate);
        }
    }

    size_t N = ts.size();
    if (N == 0) {
        std::cerr << "No data rows found in " << input_path << std::endl;
        return 1;
    }
    std::cerr << "[INFO] Loaded " << N << " data rows" << std::endl;

    double xy_obs_noise_std = 5.0;
    double yaw_rate_noise_std = 0.02;
    double forward_velocity_noise_std = 0.3;

    double initial_yaw_std = M_PI;
    double initial_yaw = gt_yaws[0] + sample_normal_distribution(0, initial_yaw_std);

    Eigen::Vector3d x(obs_trajectory_xyz[0][0], obs_trajectory_xyz[0][1], initial_yaw);

    // If the first observation is NaN (no GPS on first row), find the first GPS row
    if (std::isnan(x[0]) || std::isnan(x[1])) {
        for (size_t i = 0; i < N; ++i) {
            if (!std::isnan(obs_trajectory_xyz[i][0])) {
                x[0] = obs_trajectory_xyz[i][0];
                x[1] = obs_trajectory_xyz[i][1];
                std::cerr << "[INFO] First GPS fix found at row " << i << std::endl;
                break;
            }
        }
    }

    ExtendedKalmanFilter kf;
    kf.initialize(x, xy_obs_noise_std, yaw_rate_noise_std, forward_velocity_noise_std, initial_yaw_std);

    std::vector<double> mu_x = {x[0]};
    std::vector<double> mu_y = {x[1]};
    std::vector<double> mu_theta = {x[2]};
    std::vector<double> var_x = {kf.P(0, 0)};
    std::vector<double> var_y = {kf.P(1, 1)};
    std::vector<double> var_theta = {kf.P(2, 2)};

    double t_last = 0.0;
    for (size_t t_idx = 1; t_idx < N; ++t_idx) {
        double t = ts[t_idx];
        double dt = t - t_last;
        Eigen::Vector2d u(obs_forward_velocities[t_idx], obs_yaw_rates[t_idx]);
        // Propagate!
        kf.propagate(u, dt);
        // Update only when GPS is available
        if (!std::isnan(obs_trajectory_xyz[t_idx][0])) {
            Eigen::Vector2d z(obs_trajectory_xyz[t_idx][0], obs_trajectory_xyz[t_idx][1]);
            kf.update(z);
        }
        mu_x.push_back(kf.x_[0]);
        mu_y.push_back(kf.x_[1]);
        mu_theta.push_back(normalize_angles(kf.x_[2]));
        var_x.push_back(kf.P_(0, 0));
        var_y.push_back(kf.P_(1, 1));
        var_theta.push_back(kf.P_(2, 2));
        t_last = t;
    }

    std::ofstream output_file("output_utm.csv");
    output_file << std::setprecision(12) << "easting,northing,yaw,state_x,state_y,state_yaw" << std::endl;
    for (size_t i = 0; i < obs_trajectory_xyz.size(); ++i) {
        double easting = gt_trajectory_xyz[i][0];
        double northing = gt_trajectory_xyz[i][1];
        double yaw = gt_yaws[i];
        double state_x = mu_x[i];
        double state_y = mu_y[i];
        double state_yaw = mu_theta[i];
        output_file << easting << "," << northing << "," << yaw << "," << state_x << "," << state_y << "," << state_yaw << std::endl;
    }
    output_file.close();
    std::cerr << "[INFO] Output written to output_utm.csv" << std::endl;
    return 0;
}

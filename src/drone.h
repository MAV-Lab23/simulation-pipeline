#ifndef GROUP_10_DRONE_H
#define GROUP_10_DRONE_H

#include "types.h"
#include "utility.h"
#include "constants.h"

// TODO: Figure out how to make this function be more accurate.
static float correctPitch(float pitch_rate) {
    // TODO: Perhaps use CAMERA_TILT here.
    const float EXPECTED_MAX_PITCH_RATE = degToRad(11.0f);
    const float PITCH_CORRECTION_FACTOR = degToRad(8);
    const float PITCH_RATE_FACTOR = -degToRad(5);

    float pitch_fraction = abs(pitch_rate) / EXPECTED_MAX_PITCH_RATE;

    /*
    // TODO: Maybe do some more scaling with pitch rates here?
    if (abs(pitch_rate) > degToRad(0.1)) {
        pitch_fraction *= ???;
        PITCH_CORRECTION_FACTOR += ???;
    }
    */

    return PITCH_CORRECTION_FACTOR + PITCH_RATE_FACTOR * pitch_fraction;
}

static Vector3f optitrackToScreenRotation(Vector3f pos, enum CoordinateSystem coordinate_system) {
    
    float angle = TRUE_NORTH_TO_CARPET_ANGLE; // radians

    if (coordinate_system == NED) {
        pos = {
            pos.x * cos(angle) - pos.y * sin(angle),
            pos.x * sin(angle) + pos.y * cos(angle),
            -pos.z,
        };
    } else if (coordinate_system == ENU) {
        pos = {
            pos.x * sin(angle) + pos.y * cos(angle),
            pos.x * cos(angle) - pos.y * sin(angle),
            pos.z,
        };
    }

    return pos;
} 

static DroneState optitrackToScreenState(DroneState state, enum CoordinateSystem coordinate_system) {
    // Translate yaw from relative to true north to relative to screen north (up).
    // Clockwise is positive here.
    state.optitrack_angle.z += M_PI / 4 - TRUE_NORTH_TO_CARPET_ANGLE + M_PI;

    state.optitrack_pos = optitrackToScreenRotation(state.optitrack_pos, coordinate_system);
    return state;
}

#ifdef IN_PAPARAZZI

#include "state.h"

// Returns drone state relative to OpenCV coordinate system: (0, 0) = top left; (>0, >0) = bottom right.
static DroneState getDroneState() {

    Vector3f fixed_optitrack_pos_v = { stateGetPositionNed_f()->x, stateGetPositionNed_f()->y, stateGetPositionNed_f()->z };
    Vector3f optitrack_angle_v = { stateGetNedToBodyEulers_f()->phi, stateGetNedToBodyEulers_f()->theta, stateGetNedToBodyEulers_f()->psi };
    Vector3f optitrack_ang_rates_v = { stateGetBodyRates_f()->p, stateGetBodyRates_f()->q, stateGetBodyRates_f()->r };
    
    DroneState state;

    state.optitrack_pos = fixed_optitrack_pos_v;
    state.optitrack_angle = optitrack_angle_v;
    state.optitrack_ang_rates = optitrack_ang_rates_v;

    state = optitrackToScreenState(state, NED);

    return state;
}

#else

#include <vector>
#include <array>
#include <string>
#include <filesystem>
#include <iostream>
#include <cassert>
#include <algorithm>

#include <opencv2/opencv.hpp>

#include <parser.hpp>

static void fixDroneState(DroneState& in_state) {
    // Fix drone state to center at given inclination for testing.
    in_state.optitrack_pos.x = 0;
    in_state.optitrack_pos.y = 0;
    in_state.optitrack_pos.z = -1;
    in_state.optitrack_angle.x = 0;
    in_state.optitrack_angle.y = -degToRad(45);
    //in_state.optitrack_angle.z = 0;
}

long double getImageTimestamp(const std::filesystem::path& path) {
    // Convert from microseconds to seconds.
    // Add data and image timestamp delay here (in seconds).
    return std::stol(path.filename().string()) / 1000000.0; // - 1.547248;
}

std::pair<std::vector<std::pair<cv::Mat, DroneState>>, std::vector<Obstacle>> getDroneDataNew(
    const std::filesystem::path& drone_images_directory, CoordinateSystem coordinate_system) {

    const std::string image_directory_name = drone_images_directory.parent_path().filename().string();

    std::cout << "INFO: Parsing image directory: " << image_directory_name << std::endl;

    std::filesystem::path data_name = image_directory_name + ".csv";

    std::filesystem::path data_file = drone_images_directory / data_name;

    assert(std::filesystem::exists(data_file) && "Data file not found for input image directory");

    std::vector<std::filesystem::path> sorted_paths;

    for (const auto& entry : std::filesystem::directory_iterator(drone_images_directory)) {
        if (entry.path().extension() == ".jpg") {
            sorted_paths.push_back(entry.path());
        }
    }

    std::cout << "INFO: Found " << sorted_paths.size() << " image files" << std::endl;

    std::sort(sorted_paths.begin(), sorted_paths.end(), [](const auto& path1, const auto& path2) {
        return std::stoi(path1.filename().string()) < std::stoi(path2.filename().string());
    });

    std::vector<std::pair<cv::Mat, DroneState>> drone_data;

    drone_data.reserve(sorted_paths.size());

    std::cout << "INFO: Reading drone data from: " << data_file << std::endl;

    std::ifstream f(data_file);

    aria::csv::CsvParser image_parser(f);

    int row_index = 0;
    int image_index = 0;

    std::array<float, 9> rotation_matrix;

    for (auto& row : image_parser) {
        if (row_index == 0) {
            row_index += 1;
            continue; // Skip label row.
        }

        if (row_index == 1) {
            rotation_matrix = { std::stof(row[13]), std::stof(row[14]), std::stof(row[15]),
                                std::stof(row[16]), std::stof(row[17]), std::stof(row[18]),
                                std::stof(row[19]), std::stof(row[20]), std::stof(row[21]) };
        }

        uint32_t time = std::stoi(row[0]);

        const std::filesystem::path image_file = sorted_paths[image_index];

        uint32_t image_time = std::stoi(image_file.stem());

        if(image_time != time) {
            std::cout << "WARNING: Could not match image (" << image_time << ") to timestamp in data file (" << time << ")" << std::endl;
            row_index += 1;
            continue;
        }

        DroneState state;
        state.optitrack_pos = { std::stof(row[1]), std::stof(row[2]), std::stof(row[3]) };
        state.optitrack_angle = { std::stof(row[7]), std::stof(row[8]), std::stof(row[9]) };
        state.optitrack_ang_rates = { std::stof(row[10]), std::stof(row[11]), std::stof(row[12]) };

        state = optitrackToScreenState(state, coordinate_system);

        //printf(" %.3f, %.3f, %.3f \n", state.optitrack_pos.x, state.optitrack_pos.y, state.optitrack_pos.z);

        cv::Mat img = cv::imread(image_file.string());

        if (!img.data) {
            std::cout << "ERROR: Could not read image: " << image_file << std::endl;
            row_index += 1;
            continue;
        }

        cv::rotate(img, img, cv::ROTATE_90_COUNTERCLOCKWISE);

        image_index++;
        drone_data.push_back({ img, state });

    }

    std::vector<Obstacle> obstacles;

    std::filesystem::path obstacle_file = drone_images_directory / std::filesystem::path("obstacles.csv");

    if(std::filesystem::exists(obstacle_file)) {
        std::cout << "INFO: Extracting obstacle info from:" << obstacle_file << std::endl;

        std::ifstream o_f(obstacle_file);

        aria::csv::CsvParser obstacle_parser(o_f);

        int obstacle_index = 0;
        for (auto& o_row : obstacle_parser) {
            if (obstacle_index == 0) {
                obstacle_index += 1;
                continue; // Skip label row.
            }

            Obstacle obstacle;

            // TODO: Currently assuming all obstacles start from the ground and extend infinitely upward.
            float OBSTACLE_Z = 0.0f;

            obstacle.optitrack_pos = { std::stof(o_row[1]), std::stof(o_row[2]), OBSTACLE_Z };
            obstacle.optitrack_angle = { std::stof(o_row[3]), std::stof(o_row[4]), std::stof(o_row[5]) };

            obstacle.optitrack_angle.z += TRUE_NORTH_TO_CARPET_ANGLE;

            // ENU is default if you are reading positions from Gazebo.
            //obstacle.optitrack_pos = optitrackToScreenRotation(obstacle.optitrack_pos, ENU);

            float angle = M_PI / 2 - TRUE_NORTH_TO_CARPET_ANGLE;

            obstacle.optitrack_pos = {
                -(obstacle.optitrack_pos.x * cos(angle) - obstacle.optitrack_pos.y * sin(angle)),
                obstacle.optitrack_pos.x * sin(angle) + obstacle.optitrack_pos.y * cos(angle),
                OBSTACLE_Z,
            };

            obstacles.push_back(obstacle);
        }
        std::cout << "INFO: Found " << obstacles.size() << " obstacles in file!" << std::endl;
    } else {
        std::cout << "INFO: No obstacle file found!" << std::endl;
    }

    std::cout << "SUCCESS: File parsing complete!" << std::endl;

    assert(drone_data.size() == sorted_paths.size());

    return { drone_data, obstacles };
}

std::vector<std::pair<cv::Mat, DroneState>> getDroneData(
    const std::filesystem::path& drone_images_directory,
    const std::filesystem::path& drone_data_file,
    const std::filesystem::path& cache_data_directory,
    const std::filesystem::path& drone_data_directory,
    CoordinateSystem coordinate_system) {
    if (!std::filesystem::is_directory(cache_data_directory) || 
        !std::filesystem::exists(cache_data_directory)) {
        std::filesystem::create_directory(cache_data_directory);
    }

    std::vector<std::filesystem::path> sorted_paths;

    for (const auto& entry : std::filesystem::directory_iterator(drone_images_directory)) {
        sorted_paths.push_back(entry.path());
    }

    std::sort(sorted_paths.begin(), sorted_paths.end(), [](const auto& path1, const auto& path2) {
        return std::stoi(path1.filename().string()) < std::stoi(path2.filename().string());
    });

    std::vector<std::pair<cv::Mat, DroneState>> drone_data;

    drone_data.reserve(sorted_paths.size());

    std::filesystem::path cache_file = cache_data_directory / drone_data_file;

    if (std::filesystem::exists(cache_file)) {

        std::cout << "Reading drone data from cache file at: " << cache_file << std::endl;

        std::ifstream f(cache_file);

        aria::csv::CsvParser parser(f);

        int row_index = 0;

        for (auto& row : parser) {
            if (row_index == 0) {
                row_index += 1;
                continue; // Skip label row.
            }

            int image_index = std::stoi(row[0]);

            const std::string filepath = sorted_paths[image_index].string();

            cv::Mat img = cv::imread(filepath);

            if (!img.data) {
                std::cout << "Error reading image: " << filepath << std::endl;
                row_index += 1;
                continue;
            }

            DroneState state;
            state.optitrack_pos = { std::stof(row[1]), std::stof(row[2]), std::stof(row[3]) };
            state.optitrack_angle = { std::stof(row[4]), std::stof(row[5]), std::stof(row[6]) };
            state.optitrack_ang_rates = { std::stof(row[7]), std::stof(row[8]), std::stof(row[9]) };

            drone_data.push_back({ img, state });

            row_index += 1;
        }

    } else {

        std::ifstream f(drone_data_directory / drone_data_file);
        aria::csv::CsvParser parser(f);

        std::vector<long double> indexes;
        std::vector<long double> x;
        std::vector<long double> y;
        std::vector<long double> z;
        std::vector<long double> roll;
        std::vector<long double> yaw;
        std::vector<long double> pitch;
        std::vector<long double> roll_rate;
        std::vector<long double> yaw_rate;
        std::vector<long double> pitch_rate;

        assert(sorted_paths.size() > 0);

        int row_index = 0;
        int image_index = 0;
        long double previous_time = 0;
        DroneState previous_state;

        std::cout << "Parsing drone images and data..." << std::endl;

        for (auto& row : parser) {
            if (row_index == 0) {
                row_index += 1;
                continue; // Skip label row.
            }

            long double time = std::stold(row[0]);

            DroneState state;
            state.optitrack_pos = { std::stof(row[1]), std::stof(row[2]), std::stof(row[3]) };
            //                                   roll                pitch              yaw
            state.optitrack_angle = { std::stof(row[7]), std::stof(row[8]), std::stof(row[9]) };
            state.optitrack_ang_rates = { std::stof(row[10]), std::stof(row[11]), std::stof(row[12]) };

            state = optitrackToScreenState(state, coordinate_system);

            if (image_index >= sorted_paths.size()) break;

            long double image_timestamp = getImageTimestamp(sorted_paths[image_index]);

            if (time >= image_timestamp) {

                const std::string filepath = sorted_paths[image_index].string();

                cv::Mat img = cv::imread(filepath);

                if (!img.data) {
                    std::cout << "Error reading image: " << filepath << std::endl;
                    row_index += 1;
                    continue;
                }

                const long double prev_diff = abs(image_timestamp - previous_time);
                const long double diff = abs(image_timestamp - time);

                DroneState chosen_state = prev_diff > diff ? previous_state : state;

                drone_data.push_back({ img, chosen_state });

                indexes.push_back(image_index);
                x.push_back(chosen_state.optitrack_pos.x);
                y.push_back(chosen_state.optitrack_pos.y);
                z.push_back(chosen_state.optitrack_pos.z);
                roll.push_back(chosen_state.optitrack_angle.x);
                pitch.push_back(chosen_state.optitrack_angle.y);
                yaw.push_back(chosen_state.optitrack_angle.z);
                roll_rate.push_back(chosen_state.optitrack_ang_rates.x);
                pitch_rate.push_back(chosen_state.optitrack_ang_rates.y);
                yaw_rate.push_back(chosen_state.optitrack_ang_rates.z);

                if (image_index % 30 == 0) {
                    std::cout << "Parsed " << (int)((double)image_index / (double)sorted_paths.size() * 100.0) << "% of images..." << std::endl;
                }
                image_index++;
            }
            row_index += 1;
            previous_time = time;
            previous_state = state;
        }

        std::cout << "Image parsing complete!" << std::endl;

        assert(drone_data.size() <= sorted_paths.size());

        writeCSV(cache_file, {
            { "index", indexes },
            { "x", x }, { "y", y }, { "z", z },
            { "roll", roll }, { "pitch", pitch }, { "yaw", yaw },
            { "roll_rate", roll_rate }, { "pitch_rate", pitch_rate }, { "yaw_rate", yaw_rate },
        });

        std::cout << "Created cache file at: " << cache_file << std::endl;
    }
    return drone_data;
}

static cv::Mat getHomogenousTransform(const DroneState state) {
    // Conversion of Euler to rotation matrix.
    // https://en.wikipedia.org/wiki/Rotation_matrix#General_3D_rotations
    // a = yaw, b = pitch, y = roll
    // TODO: Figure these out.
    float a = state.optitrack_angle.z;
    float b = -state.optitrack_angle.y;
    float y = state.optitrack_angle.x;

    float cos_yaw = cos(a);
    float cos_pitch = cos(b);
    float cos_roll = cos(y);

    float sin_yaw = sin(a);
    float sin_pitch = sin(b);
    float sin_roll = sin(y);

    //std::cout << "cos_yaw: " << cos_yaw << std::endl;
    //std::cout << "sin_yaw: " << sin_yaw << std::endl;
    //std::cout << "cos_pitch: " << cos_pitch << std::endl;
    //std::cout << "sin_pitch: " << sin_pitch << std::endl;
    //std::cout << "cos_roll: " << cos_roll << std::endl;
    //std::cout << "sin_roll: " << sin_roll << std::endl;

    cv::Mat rotation_matrix = cv::Mat(4, 4, CV_32F);

    rotation_matrix.at<float>(0, 0) = cos_yaw * cos_pitch;
    rotation_matrix.at<float>(0, 1) = cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll;
    rotation_matrix.at<float>(0, 2) = cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll;
    rotation_matrix.at<float>(0, 3) = state.optitrack_pos.x;

    rotation_matrix.at<float>(1, 0) = sin_yaw * cos_pitch;
    rotation_matrix.at<float>(1, 1) = sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll;
    rotation_matrix.at<float>(1, 2) = sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll;
    rotation_matrix.at<float>(1, 3) = state.optitrack_pos.y;

    rotation_matrix.at<float>(2, 0) = -sin_pitch;
    rotation_matrix.at<float>(2, 1) = cos_pitch * sin_roll;
    rotation_matrix.at<float>(2, 2) = cos_pitch * cos_roll;
    rotation_matrix.at<float>(2, 3) = state.optitrack_pos.z;

    rotation_matrix.at<float>(3, 0) = 0;
    rotation_matrix.at<float>(3, 1) = 0;
    rotation_matrix.at<float>(3, 2) = 0;
    rotation_matrix.at<float>(3, 3) = 1;

    return rotation_matrix;
}

static Vector3f vectorDroneToOpti(const DroneState state, const Vector3f drone_point) {
    cv::Mat drone_vector = cv::Mat(4, 1, CV_32F);

    drone_vector.at<float>(0, 0) = drone_point.x;
    drone_vector.at<float>(1, 0) = drone_point.y;
    drone_vector.at<float>(2, 0) = drone_point.z;
    drone_vector.at<float>(3, 0) = 0;

    cv::Mat rotation_matrix = cv::Mat(4, 4, CV_32F);
    rotation_matrix = getHomogenousTransform(state);

    cv::Mat result = cv::Mat(4, 1, CV_32F); 

    result = rotation_matrix * drone_vector;

    Vector3f output = { result.at<float>(0, 0), result.at<float>(1, 0), result.at<float>(2, 0) };

    return output;
}

static Vector2f getGridPosition(
    const cv::Size drone_cam_size,
    const cv::Point2f cam_point,
    float drone_fov_width,
    DroneState state,
    bool correct_longitude) {

    float ground_height = 0;

    float aspect_ratio = (float)drone_cam_size.width / (float)drone_cam_size.height;

    assert(aspect_ratio > 0);

    float drone_fov_height = drone_fov_width / aspect_ratio;

    assert(drone_fov_height < drone_fov_width && "Drone fov height should not be more than fov width (for landscape images)");

    float image_fov[2] = { drone_fov_width, drone_fov_height };

    // Convert values from 0 to cam_size -> 0 to 1.
    float point_norm[2] = { normalizeValue(cam_point.x, 0.0f, (float)drone_cam_size.width),
                            normalizeValue(cam_point.y, 0.0f, (float)drone_cam_size.height) };

    assert(point_norm[0] >= 0.0 && point_norm[1] >= 0.0);
    assert(point_norm[0] <= 1.0 && point_norm[1] <= 1.0);

    // Convert values from 0 to 1 -> -1 to 1.
    float point[2] = { (point_norm[0] * 2) - 1, (point_norm[1] * 2) - 1 };

    assert(point[0] >= -1.0 && point[1] >= -1.0);
    assert(point[0] <= 1.0 && point[1] <= 1.0);

    float longitude = point[0] * image_fov[0] / 2.0;
    float latitude = -point[1] * image_fov[1] / 2.0;

    if (correct_longitude) {
        // I noticed that FOV might be shifted or distorted with respect to the center of the drone slightly.
        float FOV_WIDTH_SHIFT_CORRECTION_FACTOR = degToRad(7.0);

        longitude += FOV_WIDTH_SHIFT_CORRECTION_FACTOR;
    }

    float sin_lat = sin(latitude);
    float cos_lat = cos(latitude);
    float sin_lon = sin(longitude);
    float cos_lon = cos(longitude);

    // Direction vector of the line
    Vector3f direction_vector_drone = { cos_lat * cos_lon,
                                        cos_lat * sin_lon,
                                       -cos_lon * sin_lat };

    Vector3f direction_vector_opti = vectorDroneToOpti(state, direction_vector_drone);

    // Plane equation coefficients (for a plane parallel to xy-plane)
    float a = 0;
    float b = 0;
    float c = 1;
    float d = -ground_height;

    float pos_dot = state.optitrack_pos.x * a + state.optitrack_pos.y * b + state.optitrack_pos.z * c;
    float dir_dot = direction_vector_opti.x * a + direction_vector_opti.y * b + direction_vector_opti.z * c;
    
    // Intersection parameter
    float t = (-d - pos_dot) / dir_dot;

    Vector2f translated_point = { 0, 0 };

    // Intersection point
    float intersection_point_x = state.optitrack_pos.x + t * direction_vector_opti.x;
    float intersection_point_y = state.optitrack_pos.y + t * direction_vector_opti.y;

    translated_point = { intersection_point_x, intersection_point_y };

    /*
    // Prevent points behind the drone
    if (t >= 0) {

        // Intersection point
        float intersection_point_x = state.optitrack_pos.x + t * direction_vector_opti.x;
        float intersection_point_y = state.optitrack_pos.y + t * direction_vector_opti.y;

        translated_point = { intersection_point_x, intersection_point_y };
    }
    */

    return translated_point;
}

#endif

#endif
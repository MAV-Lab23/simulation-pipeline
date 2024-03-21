#pragma once

#include "types.h"
#include "utility.h"

#ifdef IN_PAPARAZZI

#include "state.h"

// TODO: Implement paparazzi version of GetDroneState
static DroneState getDroneState() {
    Vector3f optitrack_pos = { stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, stateGetPositionEnu_f()->z };
    Vector3f optitrack_angle = { stateGetNedToBodyEulers_f()->phi, stateGetNedToBodyEulers_f()->theta, stateGetNedToBodyEulers_f()->psi };
    DroneState state = { optitrack_pos, optitrack_angle };
    return state;
}

#else

#include <vector>
#include <string>
#include <filesystem>
#include <iostream>
#include <cassert>
#include <algorithm>

#include <opencv2/opencv.hpp>

#include <parser.hpp>

long double getImageTimestamp(const std::filesystem::path& path) {
    // Convert from microseconds to seconds.
    return std::stol(path.filename().string()) / 1000000.0;
}

std::vector<DroneData> getDroneData(
    const std::filesystem::path& drone_images_directory,
    const std::filesystem::path& cache_data_directory,
    const std::filesystem::path& drone_data_directory,
    const std::filesystem::path& drone_data_file) {
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

    std::vector<DroneData> drone_data;

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

            Image img = cv::imread(filepath);

            if (!img.data) {
                std::cout << "Error reading image: " << filepath << std::endl;
                row_index += 1;
                continue;
            }

            DroneState state;
            state.optitrack_pos = { std::stof(row[1]), std::stof(row[2]), std::stof(row[3]) };
            state.optitrack_angle = { std::stof(row[4]), std::stof(row[5]), std::stof(row[6]) };

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
        std::vector<long double> phi;
        std::vector<long double> theta;
        std::vector<long double> psi;

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
            state.optitrack_angle = { std::stof(row[7]), std::stof(row[8]), std::stof(row[9]) };

            if (image_index >= sorted_paths.size()) break;

            long double image_timestamp = getImageTimestamp(sorted_paths[image_index]);

            if (time >= image_timestamp) {

                const std::string filepath = sorted_paths[image_index].string();

                Image img = cv::imread(filepath);

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
                phi.push_back(chosen_state.optitrack_angle.x);
                theta.push_back(chosen_state.optitrack_angle.y);
                psi.push_back(chosen_state.optitrack_angle.z);

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

        writeCSV(cache_file, { { "index", indexes }, { "x", x }, { "y", y }, { "z", z }, { "phi", phi }, { "theta", theta }, { "psi", psi } });

        std::cout << "Created cache file at: " << cache_file << std::endl;
    }
    return drone_data;
}

#endif

static Vector2f getObstacleGridPosition(
    const Vector2i drone_cam_size,
    const Vector2i point,
    float drone_fov_width,
    const DroneState drone_state) {

    float aspect_ratio = drone_cam_size.y / drone_cam_size.x;
    int center_x = drone_cam_size.x / 2;
    int center_y = drone_cam_size.y / 2;

    float fov_h = drone_fov_width * aspect_ratio;

    int dist_x = point.x - center_x;
    int dist_y = point.y - center_y;

    float frac_screen_x = (float)dist_x / (float)drone_cam_size.x;
    float frac_screen_y = (float)dist_y / (float)drone_cam_size.y;

    float angle_y = frac_screen_y * fov_h;
    float angle_x = frac_screen_x * drone_fov_width;

    // anti clockwise is positive

    float heading = drone_state.optitrack_angle.z;

    float line_angle = drone_state.optitrack_angle.y - angle_y;

    // TODO: Change this to better reflect height of camera.
    float height_camera = drone_state.optitrack_pos.z;

    float x_pos_from_drone = height_camera / tan(line_angle);
    float y_pos_from_drone = x_pos_from_drone * tan(angle_x);

    float R_x = cos(heading) - sin(heading);
    float R_y = sin(heading) + cos(heading);

    // TODO: Check that this operation is in correct order.
    float x_grid = R_x * x_pos_from_drone;
    float y_grid = R_y * y_pos_from_drone;

    Vector2f final_pos = { x_grid, y_grid };
    return final_pos;
}

#ifndef GROUP_10_FILE_H
#define GROUP_10_FILE_H

#include <vector>
#include <array>
#include <string>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <cassert>
#include <algorithm>
#include <utility>

#include <opencv2/opencv.hpp>
#include <parser.hpp>

#include "transform.h"
#include "utility.h"
#include "obstacle.h"
#include "drone.h"

// From: https://www.gormanalysis.com/blog/reading-and-writing-csv-files-with-cpp/
void writeCSV(const std::filesystem::path& filename, const std::vector<std::pair<std::string, std::vector<long double>>> dataset) {
	// Make a CSV file with one or more columns of integer values
	// Each column of data is represented by the pair <column name, column data>
	//   as std::pair<std::string, std::vector<int>>
	// The dataset is represented as a vector of these columns
	// Note that all columns should be the same size

	// Create an output filestream object
	std::ofstream myFile(filename);

	// Send column names to the stream
	for (int j = 0; j < dataset.size(); ++j)
	{
		myFile << dataset.at(j).first;
		if (j != dataset.size() - 1) myFile << ","; // No comma at end of line
	}
	myFile << "\n";

	// Send data to the stream
	for (int i = 0; i < dataset.at(0).second.size(); ++i)
	{
		for (int j = 0; j < dataset.size(); ++j)
		{
			myFile << dataset.at(j).second.at(i);
			if (j != dataset.size() - 1) myFile << ","; // No comma at end of line
		}
		myFile << "\n";
	}

	// Close the file
	myFile.close();
}


long double getImageTimestamp(const std::filesystem::path& path) {
    // Convert from microseconds to seconds.
    return std::stol(path.filename().string()) / 1000000.0;
}

// Each image in the new data format has a corresponding data row entry.
std::pair<std::vector<std::pair<cv::Mat, DroneState>>, std::vector<cv::Point>> getDroneDataNew(
    const std::filesystem::path& drone_images_directory, CoordinateSystem coordinate_system) {
    assert(std::filesystem::is_directory(drone_images_directory) && "Could not find provided drone images directory");

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
        state.pos = { std::stof(row[1]), std::stof(row[2]), std::stof(row[3]) };
        state.heading = { std::stof(row[7]), std::stof(row[8]), std::stof(row[9]) };
        state.optitrack_ang_rates = { std::stof(row[10]), std::stof(row[11]), std::stof(row[12]) };

        state = optitrackToScreenState(state, coordinate_system);

        //printf(" %.3f, %.3f, %.3f \n", state.pos.x, state.pos.y, state.pos.z);

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

    std::vector<cv::Point> obstacles;

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

            cv::Point obstacle;

            transformObstacleToGrid(std::stof(o_row[1]), std::stof(o_row[2]), &obstacle.x, &obstacle.y); 

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

// This supports reading the legacy file data logging format where there are many more lines of data than images.
// Due to how long the image timestamp and data matching can take for large datasets, it caches the entries.
std::vector<std::pair<cv::Mat, DroneState>> getDroneData(
    const std::filesystem::path& drone_images_directory,
    const std::filesystem::path& drone_data_file,
    const std::filesystem::path& drone_data_directory,
    const std::filesystem::path& cache_data_directory,
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

            cv::rotate(img, img, cv::ROTATE_90_COUNTERCLOCKWISE);

            DroneState state;
            state.pos = { std::stof(row[1]), std::stof(row[2]), std::stof(row[3]) };
            state.heading = { std::stof(row[4]), std::stof(row[5]), std::stof(row[6]) };
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
            state.pos = { std::stof(row[1]), std::stof(row[2]), std::stof(row[3]) };
            //                                   roll                pitch              yaw
            state.heading = { std::stof(row[7]), std::stof(row[8]), std::stof(row[9]) };
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
                x.push_back(chosen_state.pos.x);
                y.push_back(chosen_state.pos.y);
                z.push_back(chosen_state.pos.z);
                roll.push_back(chosen_state.heading.x);
                pitch.push_back(chosen_state.heading.y);
                yaw.push_back(chosen_state.heading.z);
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

#endif
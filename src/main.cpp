#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <image_processing.h>
#include <vector>
#include <string>

int main() {
	// cv::Mat a = cv::Mat::eye(640, 640, CV_8UC1)*255;
	// cv::imshow("1", a);
	// cv::waitKey(0);


    // Specify the path to your image
    string imagePath = "../img_0139.jpg";

    // Read the image from the specified file
    Mat img = imread(imagePath, IMREAD_COLOR);
    if (img.empty()) {
        cerr << "Could not read the image: " << imagePath << endl;
        return -1;
    }

    // Vector to hold detected object positions and their distances
    vector<pair<Point, double>> objectDistances;

    // Process the image and display the results
    processImageForObjects(img, objectDistances);

    // Optionally, print out the object distances
    cout << "Detected object positions and distances:" << endl;
    for (const auto& item : objectDistances) {
        const Point& pos = item.first;
        const double& distance = item.second;
        cout << "Position: (" << pos.x << ", " << pos.y << "), Distance: " << distance << endl;
    }

    return 0;
}
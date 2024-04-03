# NOTE TO COURSE ORGANIZERS

This repository has two entry points: main.cpp and group_10_obstacle_detector.c/group_10_avoider.c depending on the context it is built in.

main.cpp represents the entry point for the offboard simulation code. Essentially, it behaves exactly like Paparazzi would except instead of getting images from the drone, it pulls them from predefined image directories with corresponding data files. This allows for recorded runs of the drone to be analyzed in detail and the behavior of the algorithm to be simulated frame by frame. In order to test the various prerecorded test scenarios, one must go to the bottom of the main.cpp file and change the "dir" string variable to be equal to one of the directory paths provided above (best_run, floor_mats_many, orange_poles_1, etc). In order to have access to these scenarios, you must download the images from [this Google Drive link](https://drive.google.com/file/d/1RThJCWmrpizffw9vcTzS49rgbQAeIgz-/view?usp=sharing). Extract the zip into a directory titled "images" in the root of this repository. In other words, after downloading these images, you should have the directory "simulation-pipeline/images/orange_poles." This will allow you to set dir=orange_poles_1 to view the run conducted in Gazebo with orange poles.

An additional added layer of complexity is that to optimize performance, we modified the video_capture.h module in our Paparazzi fork to log data in a specific format so we can easily relate each image to a time (each image only has one row of data). Because of this, the data is parsed in a very particular way (see file.h). To allow for compatibility with the traditional Paparazzi data sets, however, we provide a variable in the main.cpp file called old_data_files. If this boolean is set to true, it will search the data directory for a file with the old data format (default format produced by Paparazzi's logger.h file) alongside images in the images/<specified_dir_name> directory. Since the data files produced by logger.h contain more data rows than there are images, the C++ program must match each image to its correct row time in the data CSV file. As a result, the operation can take quite a bit of time. To combat this, we implemented a simple cached CSV file. Essentially, we take the logger.h's output CSV file and parse it for rows of data that correspond to the time stamps of the default video_capture.h's images. All the rest of the data is then discarded and this is outputted to a temporary cache directory to speed up future viewings of runs.

Finally, main.cpp has a define at the top called IN_REAL_LIFE which must be set to 0 (default) if using Gazebo images and 1 if using real-world images. This flag is in charge of switching the HSV threshold ranges when isolating the green floor. It is automatically set by the module in Paparazzi but this cannot be easily done in an offboard simulation as we cannot easily determine when the user is analyzing real-world data or drone data.

As for the group_10_obstacle_detector.c/group_10_avoider.c/opencv_wrapper.cpp files; these are only used by the module XML files defined in [paparazzi fork](https://github.com/MAV-Lab23/paparazzi). This means that the offboard simulation attempts to mimic the same simulation steps as would happen in Paparazzi, but cannot exactly due to lacking access to navigation headers among other things.


### Linux:

To install and generate build files:
```
sudo apt install build-essential cmake gcc libopencv-dev
mkdir build
cd build
cmake ..
cmake --build .
```
To make and run the code use:
```
make && ./simulation
```

### Windows MSVC:

Download and install the following dependencies:

- [Visual Studio IDE](https://visualstudio.microsoft.com/thank-you-downloading-visual-studio/?sku=Community&channel=Release&version=VS2022&source=VSLandingPage&cid=3602&passive=false)
- OpenCV for Windows. [MSVC Specific Download Link](https://github.com/opencv/opencv/releases/download/4.9.0/opencv-4.9.0-windows.exe). OpenCV extract location is relevant later. I recommend placing it in `C:/opencv` for simplicity.
- [CMake](<(https://github.com/Kitware/CMake/releases/download/v3.29.0-rc3/cmake-3.29.0-rc3-windows-x86_64.msi)>).

Navigate to where you have cloned the simulation-pipeline repository. Open a terminal and enter the following commands in order.

```
mkdir build
cd build
cmake .. -DOpenCV_DIR="<path/where/you/extracted/opencv/build/x64/vc16/lib>"
```

Note: DOpenCV_DIR is case sensitive.

At this point you should be able to find a MSVC solution (.sln) file in the build directory (i.e. /path/to/simulation-pipeline/build).
Open the solution file and run the program via the green "Local Windows Debugger" button at the top.

When "Debug" is chosen in the configuration dropdown, OpenCV will output some error messages in the console but they can be ignored and do not appear if the configuration is switched to "Release"

If you add new files to the project, ensure you rerun: (terminal inside the build directory)

```
cmake .. -DOpenCV_DIR="<path/where/you/extracted/opencv/build/x64/vc16/lib>"
```

This will regenerate all the necessary project files to recognize the newly added file.

# Windows MSYS

Download and install the following dependencies:

- [Follow the instructions here](https://www.msys2.org/) to download [MSYS](https://github.com/msys2/msys2-installer/releases/download/2024-01-13/msys2-x86_64-20240113.exe) on Windows and afterward install [Ninja build tools](https://packages.msys2.org/package/mingw-w64-x86_64-ninja).
- OpenCV for Windows. [MSYS Specific Download Link](https://github.com/huihut/OpenCV-MinGW-Build/tree/OpenCV-4.5.5-x64). OpenCV extract location is relevant later. I recommend placing it in `C:/opencv` for simplicity.
- [CMake](<(https://github.com/Kitware/CMake/releases/download/v3.29.0-rc3/cmake-3.29.0-rc3-windows-x86_64.msi)>).

Navigate to where you have cloned the simulation-pipeline repository. Open a terminal and enter the following commands in order.

```
mkdir build
cd build
cmake .. -DOpenCV_DIR="<path/where/you/extracted/opencv/build/x64/mingw/lib>"
ninja
```

Note: DOpenCV_DIR is case sensitive.

The built executable should be placed into the build directory.

If you make changes or add new files to the project, ensure you rerun: (terminal inside the build directory)

```
ninja
```

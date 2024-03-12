### Linux:

```
sudo apt install build-essential cmake gcc libopencv-dev
mkdir build
cd build
cmake ..
cmake --build .
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

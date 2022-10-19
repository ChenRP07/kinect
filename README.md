# Kinect Reconstructor
This project extract colored point clouds from mkv videos which are recorded by Microsoft Azure Kinect DK.
## Building
### Windows
We provide all relative dependencies header in `kinect/include/`  and lib in `kinect/lib/`, `kinect/bin/`. This project depends on Azure-Kinect-Sensor-SDK (k4a) and libjpeg-turbo. If you want to use your own dependencies, you can copy the header `k4a/* k4arecord/* turbojpeg.h` into `kinect/include`, and copy the library `k4a.lib k4arecord.lib` into `kinect/lib/`

You can build this project in your Windows Powshell, make sure your PC has CMake >= 3.21.0 and MinGW.

First entry the root directory of this project and create a build directory.

`mkdir build && cd build`

Then generate cmake and mingw environment

`cmake -G "MinGW Makefiles" ..`

`make`

Finally, you can find the executable file `kinect.exe` in `kinect/bin/`.

## Running
Make sure the executable file `kinect.exe` are in the same directory with dynamic link liraries `k4a.dll k4arecord.dll depthengine_2_0.dll libturbojpeg.dll`.

You can run `kinect.exe -h` or `kinect.exe --help` for help.

Generally, this project can be used as followed,

`kinect.exe -t|-b MKV_VIDEO_PATH OUTPUT_DIR_PATH SEQUENCE_NAME`

Parameter `-t` indicates output ply file is ascii format, and `-b` indicates binary_little_endian format. `MKV_VIDEO_PATH` should be the relative path of the input mkv video such as `D:/example.mkv`, and `OUTPUT_DIR_PATH` should be the relative directory path of the output files such as `D:/example/`, and `SEQUENCE_NAME` should be name of the output volumetric video, the ply file will be named as `${SEQUENCE_NAME}_${TIME_STAMP_USEC}.ply`. 
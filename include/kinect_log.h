/*
 * This is a header file of kinect::log.
 * Author : @ChenRP07
 * Date : 2022-10-14
 * */
#ifndef KINECT_LOG_H
#define KINECT_LOG_H

#include <string.h>
#include <ctime>
#include <iostream>

// specific error information
static std::string error_info[100] = {
        "wrong file name format",
        "file not exist",
        "file open fault",
        "read record configuration fault",
        "get k4a calibration from Azure Kinect recording failed",
        "cannot create k4a transformation to generate point clouds",
        "k4a handle is null",
        "cannot get next frame from mkv video",
        "cannot get depth image from video frame",
        "cannot get color image from video frame",
        "cannot create a k4a image",
        "depth image transofrmation fault",
        "input image is empty",
        "cannot create a directory to output",
        "wrong color image format, must be MJPEG",
        "cannot decompress color image by JPEG",
        "cannot seek beginning timestamp",
        "wrong application parameters, try kinect.exe -h|--help for help"
};

// error code
enum kinect_error_code {
    WRONG_FILE_NAME_FORMAT,
    FILE_NOT_EXIST,
    FILE_OPEN_FAULT,
    RECORD_CONFIGURATION_FAULT,
    GET_K4ACALIBRATION_FAILED,
    CREATE_K4ATRANFORMATION_FAILED,
    NO_K4A_HANDLE,
    GET_STREAM_FRAME_FAILED,
    GET_DEPTH_FRAME_FAILED,
    GET_COLOR_FRAME_FAILED,
    CREATE_IMAGE_FAILED,
    IMAGE_TRANSFORMATION_FAULT,
    EMPTY_IMAGE,
    CREATE_OUTPUT_DIR_FAILED,
    WRONG_COLOR_FORMAT,
    JPEG_DECOMPRESSION_FAULT,
    TIMESTAMP_FAULT,
    APP_PARAMETER_FAULT
};

// color format information
static std::string color_info[9] = {"MJPG", "NV12", "YUV2",
                                    "BGRA32", "Depth16", "IR16",
                                    "Custom8", "Custom16", "Custom"};

// image resolution information
static std::string resolution_info[7] = {"Off", "1280x720", "1920x1080",
                                         "2560x1440", "2048x1536", "3840x2160",
                                         "4096x3072"};

// depth mode information
static std::string depth_mode_info[6] = {"Off", "NFov 2x2Binned",
                                         "NFov Unbinned", "WFov 2x2Binned",
                                         "WFov Unbinned", "Passive IR"};

// fps information
static int fps_info[3] = {5, 15, 30};

// external camera sync mode information
static std::string sync_mode_info[3] = {"Standalone", "Master", "Subordinate"};

#define __error__(src) \
    kinect::log::except(__FILE__, __FUNCTION__, __LINE__, error_info[src])

#define __log_time__                                                         \
    time_t raw_time;                                                         \
    struct tm* ptm;                                                          \
    time(&raw_time);                                                         \
    ptm = localtime(&raw_time);                                              \
    printf("\033[34m[%02d-%02d-%02d %02d:%02d:%02d]  \033[0m",               \
           ptm->tm_year + 1900, ptm->tm_mon + 1, ptm->tm_mday, ptm->tm_hour, \
           ptm->tm_min, ptm->tm_sec)

namespace kinect {

    /*
    * Namespace of log information in kinect.
    * */
    namespace log {
        /*
        * Class of exception, gain error information from a "throw" and log the
        * error. Create from C++ macro __FILE__, __FUNCTION__, __LINE__ and a user
        * offered error log. Used by log_error(). e.g. try {
        *    ...
        *    throw kinect::log::except(__FILE__, __FUNCTION__, __LINE__, "Divided
        * by 0.");
        *    ...
        * }
        * catch(kinect::log::except& e) {
        *    e.log_error();
        *    do something to handle this problem.
        * }
        * */
        class except {
        private:
            std::string file_name_;      // file name of this error.
            std::string function_name_;  // fucntion name of this error.
            int line_num_;               // line number of this error.
            std::string error_;          // specific information of this error.

        public:
            /*
             * Default constructor.
             * */
            except() = default;

            /*
             * Default deconstructor.
             * */
            ~except() = default;

            /*
             * Constructor, assign values for all members.
             * @param  : string __file
             * @param  : string __func
             * @param  : int __line
             * @param  : string __error
             * */
            except(const std::string &__file, const std::string &__func,
                   const int &__line, const std::string __error);

            /*
             * Log location and inforamtion of this error.
             * @param  : ----
             * @return : void
             * */
            void log_error() const;
        };

    };  // namespace log
};  // namespace kinect

#endif  // KINECT_LOG_H

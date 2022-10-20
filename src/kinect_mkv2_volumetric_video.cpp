/*
 * Source file of kinect::record::KinectMkv2VolumetricVideo
 * Author : @ChenRP07
 * Time : 2022-10-17
 * */

#include "kinect_log.h"
#include "kinect_record.h"

void kinect::record::KinectMkv2VolumetricVideo::init_video(
        const std::string &__video_path) {
    try {
        if (__video_path.size() < 5) {
            throw __error__(WRONG_FILE_NAME_FORMAT);
        }
        std::string file_type = __video_path.substr(__video_path.size() - 4, 4);
        if (file_type != ".mkv") {
            throw __error__(WRONG_FILE_NAME_FORMAT);
        }
        if (access(__video_path.c_str(), F_OK) == -1) {
            throw __error__(FILE_NOT_EXIST);
        }

        // open *.mkv file
        k4a_result_t result = k4a_playback_open(__video_path.c_str(), &this->k4a_handle_);
        if (result != K4A_RESULT_SUCCEEDED) {
            k4a_playback_close(this->k4a_handle_);
            throw __error__(FILE_OPEN_FAULT);
        }

        // get record configuration from file
        result = k4a_playback_get_record_configuration(
                this->k4a_handle_, &this->k4a_record_config_);

        if (result != K4A_RESULT_SUCCEEDED) {
            k4a_playback_close(this->k4a_handle_);
            throw __error__(RECORD_CONFIGURATION_FAULT);
        }

        // seek beginning timestamp
        result = k4a_playback_seek_timestamp(this->k4a_handle_, this->k4a_record_config_.start_timestamp_offset_usec,
                                             K4A_PLAYBACK_SEEK_BEGIN);
        if (result != K4A_RESULT_SUCCEEDED) {
            k4a_playback_close(this->k4a_handle_);
            throw __error__(TIMESTAMP_FAULT);
        }

        // get calibration from Azure Kinect device
        k4a_calibration_t calibration;
        result = k4a_playback_get_calibration(this->k4a_handle_, &calibration);
        if (result != K4A_RESULT_SUCCEEDED) {
            k4a_playback_close(this->k4a_handle_);
            throw __error__(GET_K4ACALIBRATION_FAILED);
        }

        // create transformation handle
        this->k4a_point_cloud_transformation_handle_ =
                k4a_transformation_create(&calibration);
        if (this->k4a_point_cloud_transformation_handle_ == nullptr) {
            k4a_playback_close(this->k4a_handle_);
            throw __error__(CREATE_K4ATRANFORMATION_FAILED);
        }

        __log_time__;
        std::cout << "\033[36mInitialize mkv video from file " << __video_path
                  << "\033[0m" << std::endl;

    }
    catch (const kinect::log::except &error_log) {
        error_log.log_error();
        this->~KinectMkv2VolumetricVideo();
        exit(1);
    }
}

void kinect::record::KinectMkv2VolumetricVideo::set_name(
        const std::string &__sequence_name) {
    try {
        if (__sequence_name.empty()) {
            throw __error__(WRONG_FILE_NAME_FORMAT);
        }
        this->video_.set_name(__sequence_name);
    }
    catch (const kinect::log::except &error_log) {
        error_log.log_error();
        this->~KinectMkv2VolumetricVideo();
        exit(1);
    }
}

void kinect::record::KinectMkv2VolumetricVideo::log_config() const {
    __log_time__;
    std::cout << "\033[36mConfiguration listed below." << std::endl;
    std::cout << "                       Color format : "
              << color_info[this->k4a_record_config_.color_format] << std::endl;
    std::cout << "                       Color resolution : "
              << resolution_info[this->k4a_record_config_.color_resolution]
              << std::endl;
    std::cout << "                       Depth mode : "
              << depth_mode_info[this->k4a_record_config_.depth_mode]
              << std::endl;
    std::cout << "                       Fps : "
              << fps_info[this->k4a_record_config_.camera_fps] << std::endl;
    std::cout << "                       Color track : ";
    if (this->k4a_record_config_.color_track_enabled) {
        std::cout << "Enabled" << std::endl;
    }
    else {
        std::cout << "Disabled" << std::endl;
    }
    std::cout << "                       Depth track : ";
    if (this->k4a_record_config_.depth_track_enabled) {
        std::cout << "Enabled" << std::endl;
    }
    else {
        std::cout << "Disabled" << std::endl;
    }
    std::cout << "                       IR track : ";
    if (this->k4a_record_config_.ir_track_enabled) {
        std::cout << "Enabled" << std::endl;
    }
    else {
        std::cout << "Disabled" << std::endl;
    }
    std::cout << "                       IMU track : ";
    if (this->k4a_record_config_.imu_track_enabled) {
        std::cout << "Enabled" << std::endl;
    }
    else {
        std::cout << "Disabled" << std::endl;
    }
    std::cout
            << "                       Delay between color and depth images : ";
    printf("%.2fms\n",
           static_cast<float>(
                   this->k4a_record_config_.depth_delay_off_color_usec / 1000.0f));
    std::cout << "                       Wired synchronization mode : "
              << sync_mode_info[this->k4a_record_config_.wired_sync_mode]
              << std::endl;
    std::cout << "                       Delay between recording and "
                 "externally synced master camera : ";
    printf("%.2fms\n",
           static_cast<float>(
                   this->k4a_record_config_.subordinate_delay_off_master_usec /
                   1000.0f));
    std::cout << "                       Timestamp offset : ";
    printf("%.2fms\n",
           static_cast<float>(
                   this->k4a_record_config_.start_timestamp_offset_usec / 1000.0f));
    std::cout << "\033[0m";
}

k4a_image_t kinect::record::KinectMkv2VolumetricVideo::get_point_cloud_image(
        k4a_image_t &__color_image, k4a_image_t &__depth_image) {
    // get color image size, width and height
    int color_image_width = k4a_image_get_width_pixels(__color_image);
    int color_image_height = k4a_image_get_height_pixels(__color_image);

    // create new image
    k4a_image_t transformed_depth_image, point_cloud_image;

    k4a_result_t result = k4a_image_create(
            K4A_IMAGE_FORMAT_DEPTH16, color_image_width, color_image_height,
            color_image_width * sizeof(int16_t), &transformed_depth_image);
    if (result == K4A_RESULT_FAILED) {
        throw __error__(CREATE_IMAGE_FAILED);
    }

    result = k4a_image_create(
            K4A_IMAGE_FORMAT_CUSTOM, color_image_width, color_image_height,
            color_image_width * sizeof(int16_t) * 3, &point_cloud_image);
    if (result == K4A_RESULT_FAILED) {
        throw __error__(CREATE_IMAGE_FAILED);
    }

    // transform depth image to a color image
    result = k4a_transformation_depth_image_to_color_camera(
            this->k4a_point_cloud_transformation_handle_, __depth_image,
            transformed_depth_image);

    if (result == K4A_RESULT_FAILED) {
        throw __error__(IMAGE_TRANSFORMATION_FAULT);
    }

    // transform depth image to point cloud image
    result = k4a_transformation_depth_image_to_point_cloud(
            this->k4a_point_cloud_transformation_handle_, transformed_depth_image,
            K4A_CALIBRATION_TYPE_COLOR, point_cloud_image);
    if (result == K4A_RESULT_FAILED) {
        throw __error__(IMAGE_TRANSFORMATION_FAULT);
    }

    // free memory
    k4a_image_release(transformed_depth_image);
    return point_cloud_image;
}

void kinect::record::KinectMkv2VolumetricVideo::generate_point_cloud(
        k4a_image_t &__point_cloud_image, k4a_image_t &__color_image) {
    try {
        if (__point_cloud_image == nullptr || __color_image == nullptr) {
            throw __error__(EMPTY_IMAGE);
        }

        // get image size
        int width = k4a_image_get_width_pixels(__color_image);
        int height = k4a_image_get_height_pixels(__color_image);

        // get image data
        int16_t *point_cloud_data = static_cast<int16_t *>(
                static_cast<void *>(k4a_image_get_buffer(__point_cloud_image)));

        uint8_t *color_image_data = k4a_image_get_buffer(__color_image);

        // generate points
        std::vector<kinect::type::PointXYZRGB> point_cloud;
        for (int i = 0; i < width * height; ++i) {
            kinect::type::PointXYZRGB point;

            point.x = point_cloud_data[i * 3 + 0];
            point.y = point_cloud_data[i * 3 + 1];
            point.z = point_cloud_data[i * 3 + 2];

            // TODO : Filtering background here.
            if (point.z == 0) {
                continue;
            }

            point.b = color_image_data[i * 4 + 0];
            point.g = color_image_data[i * 4 + 1];
            point.r = color_image_data[i * 4 + 2];

            point_cloud.emplace_back(point);
        }

        this->video_.add_point_cloud(
                point_cloud,
                this->k4a_record_config_.start_timestamp_offset_usec,
                this->k4a_record_config_.camera_fps);
    }
    catch (const kinect::log::except &error_log) {
        error_log.log_error();
        this->~KinectMkv2VolumetricVideo();
        exit(1);
    }
}

bool kinect::record::KinectMkv2VolumetricVideo::get_point_cloud() {
    try {
        timeval time_start, time_end;
        mingw_gettimeofday(&time_start, nullptr);
        if (this->k4a_handle_ == nullptr) {
            throw __error__(NO_K4A_HANDLE);
        }

        k4a_image_t depth_image, color_image, point_cloud_image, uncompressed_color_image;

        // fetch next frame
        k4a_stream_result_t stream_result = k4a_playback_get_next_capture(
                this->k4a_handle_, &this->k4a_capture_);
        if (stream_result == K4A_STREAM_RESULT_EOF) {
            __log_time__;
            std::cout << "\033[36mVideo end.\033[0m" << std::endl;
            return true;
        }
        else if (stream_result == K4A_STREAM_RESULT_FAILED) {
            throw __error__(GET_STREAM_FRAME_FAILED);
        }

        // get depth image
        depth_image = k4a_capture_get_depth_image(this->k4a_capture_);
        if (depth_image == nullptr) {
            throw __error__(GET_DEPTH_FRAME_FAILED);
        }

        // color image
        color_image = k4a_capture_get_color_image(this->k4a_capture_);
        if (color_image == nullptr) {
            throw __error__(GET_COLOR_FRAME_FAILED);
        }

        // check format
        k4a_image_format_t format = k4a_image_get_format(color_image);
        if (format != K4A_IMAGE_FORMAT_COLOR_MJPG) {
            throw __error__(WRONG_COLOR_FORMAT);
        }

        // get color image size
        int color_width = k4a_image_get_width_pixels(color_image);
        int color_height = k4a_image_get_height_pixels(color_image);

        // create uncompressed image
        if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                                                     color_width,
                                                     color_height,
                                                     color_width * 4 * (int) sizeof(uint8_t),
                                                     &uncompressed_color_image)) {
            throw __error__(CREATE_IMAGE_FAILED);
        }

        // JPEG decompression
        tjhandle tjHandle;
        tjHandle = tjInitDecompress();
        if (tjDecompress2(tjHandle,
                          k4a_image_get_buffer(color_image),
                          static_cast<unsigned long>(k4a_image_get_size(color_image)),
                          k4a_image_get_buffer(uncompressed_color_image),
                          color_width,
                          0, // pitch
                          color_height,
                          TJPF_BGRA,
                          TJFLAG_FASTDCT | TJFLAG_FASTUPSAMPLE) != 0) {
            tjDestroy(tjHandle);
            throw __error__(JPEG_DECOMPRESSION_FAULT);
        }
        tjDestroy(tjHandle);

        // align depth image to color image
        point_cloud_image =
                this->get_point_cloud_image(uncompressed_color_image, depth_image);

        if (point_cloud_image == nullptr) {
            throw __error__(IMAGE_TRANSFORMATION_FAULT);
        }

        // generate point cloud
        this->generate_point_cloud(point_cloud_image, uncompressed_color_image);

        mingw_gettimeofday(&time_end, nullptr);
        float time_cost = static_cast<float>(time_end.tv_sec - time_start.tv_sec) * 1000.0f + static_cast<float>
                (time_end.tv_usec - time_start.tv_usec) / 1000.0f;
        __log_time__;
        printf("\033[36mGenerate point cloud from mkv video frame #%u, cost %.3fms.\n\033[0m", this->video_.size(),
               time_cost);

        k4a_image_release(depth_image);
        k4a_image_release(color_image);
        k4a_image_release(uncompressed_color_image);
        k4a_image_release(point_cloud_image);
        return false;

    }
    catch (const kinect::log::except &error_log) {
        error_log.log_error();
        this->~KinectMkv2VolumetricVideo();
        exit(1);
    }
}

void kinect::record::KinectMkv2VolumetricVideo::output_point_cloud_sequence(
        const std::string &__output_sequence_path, bool __binary) {
    try {
        timeval time_start, time_end;
        mingw_gettimeofday(&time_start, nullptr);
        std::string format;
        if (__binary) {
            format = "binary";
        }
        else {
            format = "ascii";
        }

        __log_time__;
        printf("\033[36mWriting volumetric video to %s .ply format file ......\n\033[0m", format.c_str());

        // dir do not exist
        if (opendir(__output_sequence_path.c_str()) == nullptr) {
            // create this dir, mode 0775
            if (mkdir(__output_sequence_path.c_str()) == -1) {
                throw __error__(CREATE_OUTPUT_DIR_FAILED);
            }
        }
        this->video_.output(__output_sequence_path, __binary);
        mingw_gettimeofday(&time_end, nullptr);

        printf("                      \033[36mWriting file done, cost %ds.\n\033[0m", this->video_.size(),
               time_end.tv_sec - time_start.tv_sec);

        // release memory
        if (this->k4a_handle_ != nullptr) {
            k4a_playback_close(this->k4a_handle_);
        }
        if (this->k4a_capture_ != nullptr) {
            k4a_capture_release(this->k4a_capture_);
        }
        if (this->k4a_point_cloud_transformation_handle_ != nullptr) {
            k4a_transformation_destroy(this->k4a_point_cloud_transformation_handle_);
        }
    }
    catch (const kinect::log::except &error_log) {
        error_log.log_error();
        this->~KinectMkv2VolumetricVideo();
        exit(1);
    }
}

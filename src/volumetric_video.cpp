/*
 * Source file of kinect::type::VolumetricVideo .
 * Author : @ChenRP07
 * Date : 2022-10-14
 * */
#include "kinect_log.h"
#include "kinect_type.h"

#include <ostream>

kinect::type::PointCloudFrame::PointCloudFrame(
        std::vector<kinect::type::PointXYZRGB> &__point_cloud, uint64_t __time) {
    this->cloud_.resize(__point_cloud.size());
    for (size_t i = 0; i < this->cloud_.size(); i++) {
        this->cloud_[i] = __point_cloud.at(i);
    }
    this->time_stamp_ = __time;
}

void kinect::type::PointCloudFrame::output_ascii(
        const std::string &__output_path) {
    try {
        std::ofstream outfile;
        outfile.open(__output_path.c_str(), std::ios::out | std::ios::trunc);
        if (!outfile.is_open()) {
            throw __error__(FILE_OPEN_FAULT);
        }
        outfile << "ply" << std::endl;
        outfile << "format ascii 1.0" << std::endl;
        outfile << "comment made by @ChenRP07" << std::endl;
        outfile << "element vertex " << this->cloud_.size() << std::endl;
        outfile << "property float x" << std::endl;
        outfile << "property float y" << std::endl;
        outfile << "property float z" << std::endl;
        outfile << "property uchar red" << std::endl;
        outfile << "property uchar green" << std::endl;
        outfile << "property uchar blue" << std::endl;
        outfile << "end_header" << std::endl;
        for (auto &i: this->cloud_) {
            outfile << i.x << " " << i.y << " " << i.z << " " << i.r << " "
                    << i.g << " " << i.b << std::endl;
        }
        outfile.close();
    }
    catch (const kinect::log::except &error_log) {
        error_log.log_error();
        this->~PointCloudFrame();
        exit(1);
    }
}

void kinect::type::PointCloudFrame::output_binary(const std::string &__output_path) {
    try {
        std::ofstream outfile;
        outfile.open(__output_path.c_str(), std::ios::out | std::ios::trunc);
        if (!outfile.is_open()) {
            throw __error__(FILE_OPEN_FAULT);
        }
        outfile << "ply" << std::endl;
        outfile << "format binary_little_endian 1.0" << std::endl;
        outfile << "comment made by @ChenRP07" << std::endl;
        outfile << "element vertex " << this->cloud_.size() << std::endl;
        outfile << "property float x" << std::endl;
        outfile << "property float y" << std::endl;
        outfile << "property float z" << std::endl;
        outfile << "property uchar red" << std::endl;
        outfile << "property uchar green" << std::endl;
        outfile << "property uchar blue" << std::endl;
        outfile << "end_header" << std::endl;
        outfile.close();

        outfile.open(__output_path.c_str(), std::ios::out | std::ios::app | std::ios::binary);
        for (auto &i: this->cloud_) {
            float coordinates[3] = {i.x, i.y, i.z};
            uint8_t colors[3] = {i.r, i.g, i.b};
            outfile.write((char *) coordinates, sizeof(float) * 3);
            outfile.write((char *) colors, sizeof(uint8_t) * 3);
        }
        outfile.close();
    }
    catch (const kinect::log::except &error_log) {
        error_log.log_error();
        this->~PointCloudFrame();
        exit(1);
    }

}

void kinect::type::PointCloudFrame::output(const std::string &__output_path,
                                           bool __binary) {
    std::string file_name =
            __output_path + "_" + std::to_string(this->time_stamp_) + ".ply";
    if (__binary) {
        this->output_binary(file_name);
    }
    else {
        this->output_ascii(file_name);
    }
}

void kinect::type::VolumetricVideo::set_name(
        const std::string &__sequence_name) {
    try {
        if (__sequence_name.empty()) {
            throw __error__(WRONG_FILE_NAME_FORMAT);
        }
        this->volumetric_video_name_ = __sequence_name;
    }
    catch (const kinect::log::except &error_log) {
        error_log.log_error();
        this->~VolumetricVideo();
        exit(1);
    }
}

void kinect::type::VolumetricVideo::add_point_cloud(
        std::vector<kinect::type::PointXYZRGB> &__point_cloud,
        uint64_t __time_offset, int __fps) {
    // time interval using usec
    uint64_t time_interval = 1e6 / __fps;
    uint64_t time_stamp = __time_offset + this->frames_.size() * time_interval;
    this->frames_.emplace_back(__point_cloud, time_stamp);
}

void kinect::type::VolumetricVideo::output(const std::string &__output_path,
                                           bool __binary) {
    try {
        if (this->volumetric_video_name_.empty()) {
            throw __error__(WRONG_FILE_NAME_FORMAT);
        }

        std::string file_name_prev = __output_path;
        if (file_name_prev.back() != '/') {
            file_name_prev += '/';
        }

        file_name_prev += this->volumetric_video_name_;

        for (auto &i: this->frames_) {
            i.output(file_name_prev, __binary);
        }
    }
    catch (const kinect::log::except &error_log) {
        error_log.log_error();
        this->~VolumetricVideo();
        exit(1);
    }
}

/*
 * This is a header file of kinect_type.
 * Author : @ChenRP07
 * Date : 2022-10-13
 * */
#ifndef KINECT_TYPE_H
#define KINECT_TYPE_H

#include <k4a/k4atypes.h>
#include <k4a/k4a.h>
#include <k4arecord/playback.h>

#include <cstdint>
#include <vector>
#include <fstream>
#include <unistd.h>

namespace kinect {
    namespace type {
        // Point attributes, x/y/z for 3D coordinates, r/g/b for colors
        struct PointXYZRGB {
            float x, y, z;
            uint8_t r, g, b;
        };

        /*
        * Point cloud frame of a volumetric video.
        * It contains a set of PointXYZRGB type points and a relative usec timestamp.
        * */
        class PointCloudFrame {
        private:
            // A static point cloud frame with format coordinates XYZ and colors RGB.
            std::vector<kinect::type::PointXYZRGB> cloud_;
            // time stamp for this point cloud frame
            uint64_t time_stamp_;

            /*
             * Output cloud_ to a binary .ply format file.
             * @param  : const std::string& __output_path -- file path.
             * @return : void
             * */
            void output_binary(const std::string &__output_path);

            /*
             * Output cloud_ to a ascii .ply format file.
             * @param  : const std::string& __output_path -- file path.
             * @return : void
             * */
            void output_ascii(const std::string &__output_path);

        protected:
        public:
            /*
             * Default constructor.
             * */
            PointCloudFrame() = default;

            /*
             * Default deconstructor.
             * */
            ~PointCloudFrame() = default;

            /*
             * Constructor.
             * @param  : std::vector<kinect::type::PointXYZRGB>& __point_cloud -- data
             * @param  : uint64_t __time -- usec timestamp
             * */
            PointCloudFrame(std::vector<kinect::type::PointXYZRGB> &__point_cloud, uint64_t __time);

            /*
             * Output cloud_ to  .ply format file.
             * @param  : const std::string& __output_path -- file path.
             * @param  : bool __binary -- 0 is ascii, 1 is binary
             * @return : void
             * */
            void output(const std::string &__output_path, bool __binary);
        };

        /*
        * Volumetric video.
        * It contains a set of point cloud frames and a video sequence name.
        * */
        class VolumetricVideo {
        private:
            // All point cloud frames.
            std::vector<PointCloudFrame> frames_;
            // Name of this video, using this to name the output file.
            std::string volumetric_video_name_;
        public:
            /*
             * Default constructor.
             * */
            VolumetricVideo() = default;
            /*
             * Default deconstructor.
             * */
            ~VolumetricVideo() = default;

            /*
             * Set sequence name.
             * @param  : const std::string& __sequence_name -- name
             * @return : void
             * */
            void set_name(const std::string &__sequence_name);

            /*
             * Add point cloud frame to frames_;
             * @param  : std::vector<kinect::type::PointXYZRGB> &__point_cloud -- data
             * @param  : uint64_t __time_offset -- usec time offset of whole video
             * @param  : int __fps -- fps of this video
             * @return : void
             * */
            void add_point_cloud(std::vector<kinect::type::PointXYZRGB> &__point_cloud,
                                 uint64_t __time_offset, int __fps);

            /*
             * Output video to  .ply format file.
             * @param  : const std::string& __output_path -- file path.
             * @param  : bool __binary -- 0 is ascii, 1 is binary
             * @return : void
             * */
            void output(const std::string &__output_path, bool __binary);

            /*
             * Size of frames_;
             * @param  : ----
             * @return : size_t -- size
             * */
            const size_t size() const { return this->frames_.size(); }
        protected:
        };
    };  // namespace type
};  // namespace kinect

#endif  // KINECT_TYPE_H

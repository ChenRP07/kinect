/*
 * This is a header file of kinect_record.
 * Author : @ChenRP07
 * Date : 2022-10-13
 * */

#ifndef KINECT_RECORD_H
#define KINECT_RECORD_H

#include <turbojpeg.h>
#include "kinect_type.h"
#include <dirent.h>

namespace kinect {
    namespace record {
        /*
        * KinectMkv2VolumetricVideo is used to convert a mkv video to a point cloud
        * sequence. How to use :
        * ......
        *
        * KinectMkv2VolumetricVideo example;
        * example.init_video(INPUT_VIDEO_PATH);
        * example.log_config();
        * while (true) {
        *     if (example.get_point_cloud()) {
        *         break;
        *     }
        * }
        * example.output_point_cloud_sequence(OUTPUT_SEQUENCE_DIR);
        *
        * ......
        * */
        class KinectMkv2VolumetricVideo {
        private:
            // volumetric video
            kinect::type::VolumetricVideo video_;
            // kinect process handle
            k4a_playback_t k4a_handle_;
            // configuration of this video
            k4a_record_configuration_t k4a_record_config_;
            // kinect transformation handle, used to transform depth image
            k4a_transformation_t k4a_point_cloud_transformation_handle_;
            // kinect capture handle, used to get image from mkv video
            k4a_capture_t k4a_capture_;
            /*
             * Get a point cloud image from a color image and a depth image.
             * @param  : k4a_image_t& __color_image -- color information
             * @param  : k4a_image_t& __depth_image -- depth information
             * @return : k4a_image_t -- result point cloud image
             * */
            k4a_image_t get_point_cloud_image(k4a_image_t &__color_image,
                                              k4a_image_t &__depth_image);

            /*
             * Generate point cloud and add it to video_.
             * @param  : k4a_image_t& __point_cloud_image -- point xyz coordinates
             * @param  : k4a_image_t& __color_image -- point color information
             * @return : void
             * */
            void generate_point_cloud(k4a_image_t &__point_cloud_image,
                                      k4a_image_t &__color_image);

        public:
            /*
             * Default constructor.
             * */
            KinectMkv2VolumetricVideo() = default;

            /*
             * Default deconstructor.
             * */
            ~KinectMkv2VolumetricVideo() = default;

            /*
             * Initialize a video container, input video path.
             * @param  : string __video_path
             * @return : void
             * */
            void init_video(const std::string &__video_path);

            /*
             * Log the information of k4a_record_config_
             * @param  : ----
             * @return : void
             * */
            void log_config() const;

            /*
             * Get a point cloud frame from mkv video, return if this is the last frame.
             * @param  : ----
             * @return : bool -- if no frame 1, else 0
             * */
            bool get_point_cloud();

            /*
             * Set sequence name.
             * @param  : const std::string& __sequence_name -- name
             * @return : void
             * */
            void set_name(const std::string &__sequence_name);

            /*
             * Output point cloud sequence to gived path, named as (SEQUENCE_NAME)_(TIME_STAMP).ply
             * @param  : const std::string& __output_sequence_path -- output dir path
             * @param  : bool __binary -- output format, 0 is ascii, 1 is binary
             * @return : void
             * */
            void output_point_cloud_sequence(const std::string &__output_sequence_path, bool __binary);
        };
    };  // namespace record
};  // namespace kinect

#endif  // KINECT_RECORD_H

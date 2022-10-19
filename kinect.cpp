#include "kinect_log.h"
#include "kinect_record.h"

int main(int argc, char *argv[]) {
    try {
        if (argc < 2) {
            throw __error__(APP_PARAMETER_FAULT);
        }
        else if (argc == 2) {
            std::string parse(argv[1]);
            if (parse == "-h" || parse == "--help") {
                std::cout << "Giving a mkv kinect video, this application will generate point cloud frames."
                          << std::endl;
                std::cout << "Parameters should be given as followed : " << std::endl;
                std::cout << "kinect.exe -t|-b MKV_VIDEO_PATH OUTPUT_DIR_PATH SEQUENCE_NAME" << std::endl;
            }
            else {
                throw __error__(APP_PARAMETER_FAULT);
            }
        }
        else if (argc == 5) {
            std::string format(argv[1]), mkv_path(argv[2]), output_dir(argv[3]), seq_name(argv[4]);
            bool binary;
            if (format == "-t") {
                binary = false;
            }
            else if (format == "-b") {
                binary = true;
            }
            kinect::record::KinectMkv2VolumetricVideo handle;
            handle.init_video(mkv_path);
            handle.set_name(seq_name);
            handle.log_config();
            while (true) {
                if (handle.get_point_cloud()) {
                    break;
                }
            }
            handle.output_point_cloud_sequence(output_dir, binary);
        }
        else {
            throw __error__(APP_PARAMETER_FAULT);
        }
    }
    catch (const kinect::log::except &error_log) {
        error_log.log_error();
        return 0;
    }
    return 0;
}

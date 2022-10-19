/*
 * Source file of kinect::log
 * Author : @ChenRP07
 * Date : 2022-10-19
 * */
#include "kinect_log.h"

kinect::log::except::except(const std::string &__file,
                            const std::string &__func, const int &__line,
                            const std::string __error)
        : file_name_{__file},
          function_name_{__func},
          line_num_{__line},
          error_{__error} {}

void kinect::log::except::log_error() const {
    std::cout << "\033[31mError\033[0m : in file "
              << "\033[32m" << this->file_name_ << "\033[0m, function "
              << "\033[31m" << this->function_name_ << "\033[0m, line "
              << "\033[34m" << this->line_num_ << "\033[0m, " << this->error_
              << "." << std::endl;
    return;
}


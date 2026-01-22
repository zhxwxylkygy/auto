#pragma once
#include <algorithm>
#include <filesystem>
#include <regex>
#include <stdexcept>
#include <thread>

namespace ad {
using namespace std::chrono_literals;

const std::regex RECORD_FILE_SAVE_PATTERN(
    R"(rosbag2_\d{4}_\d{2}_\d{2}-\d{2}_\d{2}_\d{2}_\d+.db3)");

class AutoDelete {
public:
  AutoDelete() = delete;

  AutoDelete(const std::filesystem::path &save_dir_path,
             size_t video_max_num_in_dir) {
    Start(save_dir_path, video_max_num_in_dir);
  };

  void Stop() { this->t.join(); }

  void Start(const std::filesystem::path &save_dir_path,
             size_t video_max_num_in_dir) {

    if (!std::filesystem::exists(save_dir_path)) {
      throw std::logic_error("ERR_MSG_INVALID_SAVE_DIR_PATH");
    }
    std::filesystem::directory_entry entry(save_dir_path);
    if (!entry.is_directory()) {
      throw std::logic_error("ERR_MSG_INVALID_SAVE_DIR_PATH");
    }
    if (!(video_max_num_in_dir >= 1 && video_max_num_in_dir <= 50)) {
      throw std::logic_error("ERR_MSG_INVALID_FILE_MAX_NUM_IN_DIR");
    }

    auto f = [save_dir_path, video_max_num_in_dir]() {
      while (true) {

        std::filesystem::recursive_directory_iterator dir_iter(save_dir_path);

        std::vector<
            std::pair<std::filesystem::file_time_type, std::filesystem::path>>
            file_with_time_vec;

        std::for_each(dir_iter, std::filesystem::recursive_directory_iterator(),
                      [&file_with_time_vec](auto &file_entry) {
                        auto file_path = file_entry.path();
                        std::smatch match_result;
                        auto filename_str = file_path.filename().string();
                        auto c = RECORD_FILE_SAVE_PATTERN;
                        bool a = std::regex_match(filename_str, match_result,
                                                  RECORD_FILE_SAVE_PATTERN);
                        if (std::filesystem::is_regular_file(file_entry) &&
                            std::regex_match(filename_str, match_result,
                                             RECORD_FILE_SAVE_PATTERN)) {
                          file_with_time_vec.emplace_back(std::make_pair(
                              file_entry.last_write_time(), file_path));
                        }
                      });

        std::sort(file_with_time_vec.begin(), file_with_time_vec.end(),
                  [](auto &a, auto &b) { return a.first < b.first; });

        if (file_with_time_vec.size() >= video_max_num_in_dir) {
          int need_to_remove_num = static_cast<int>(file_with_time_vec.size() -
                                                    video_max_num_in_dir + 1);
          auto end_iter = file_with_time_vec.begin() + need_to_remove_num;
          std::for_each(file_with_time_vec.begin(), end_iter,
                        [](auto &file_with_time) {
                          std::filesystem::remove(file_with_time.second);
                        });
          file_with_time_vec.erase(file_with_time_vec.begin(), end_iter);
        }

        std::this_thread::sleep_for(3s);
      }
    };

    t = std::thread(f);
  }

private:
  std::thread t;
};

} // namespace ad

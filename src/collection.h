#pragma once

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace point_lio {

struct Collection {
    using Point = pcl::PointXYZINormal;
    using Cloud = pcl::PointCloud<Point>;
    using Clock = std::chrono::steady_clock;

    auto spin(const Cloud& input) -> void {
        if (timing_started && time_limit.count() > 0) {
            const auto now = Clock::now();
            if (now - started_at >= time_limit) {
                timing_started = false;
                collecting_enabled = false;
            }
        }

        if (!collecting_enabled) {
            return;
        }

        if (!accumulation) {
            accumulation = std::make_shared<Cloud>();
        }
        *accumulation += input;
    }

    auto set_saving_path(const std::filesystem::path& path) -> void {
        if (path.empty()) {
            return;
        }
        saving_path = path;
    }

    auto set_time_limit(std::chrono::seconds limit) -> void {
        time_limit = limit;
        if (time_limit.count() <= 0) {
            collecting_enabled = true;
        }
    }

    auto start_timing() -> void {
        started_at = Clock::now();
        timing_started = true;
        collecting_enabled = true;
    }

    auto save_once() noexcept {
        auto now = std::chrono::system_clock::now();
        auto current_time = std::chrono::system_clock::to_time_t(now);
        auto local_tm = *std::localtime(&current_time);
        auto unix_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
        auto ms_part = unix_ms.count() % 1000;

        auto filename_builder = std::ostringstream{};
        filename_builder << std::put_time(&local_tm, "%Y%m%d_%H%M%S") << "_" << std::setw(3)
                         << std::setfill('0') << ms_part << ".pcd";

        auto filename = saving_path;
        filename /= filename_builder.str();
        save_to(filename);

        return filename;
    }

private:
    std::shared_ptr<Cloud> accumulation;
    std::filesystem::path saving_path{"/tmp/point_lio_collection"};

    std::chrono::seconds time_limit{0};
    Clock::time_point started_at{};

    bool timing_started = false;
    bool collecting_enabled = true;

    auto save_to(const std::filesystem::path& filename) -> void {

        if (!accumulation || accumulation->size() == 0) {
            return;
        }

        std::filesystem::create_directories(filename.parent_path());
        pcl::io::savePCDFileBinary(filename.string(), *accumulation);
    }
};

} // namespace point_lio

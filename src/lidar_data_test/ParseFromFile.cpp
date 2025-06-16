/**
 * @file ParseFromFileToPCD.cpp
 * @author huizeyu (huizeyu@siasun.com)
 * @brief 反序列化Lidar数据并保存为PCD文件 (只保存XYZI)
 * @version 0.1
 * @date 2025-06-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "sensor.pb.h"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

namespace fs = std::filesystem;

int main() {
  GOOGLE_PROTOBUF_VERIFY_VERSION; // 初始化Protobuf库

  // ===== 配置参数 =====
  const std::string input_filename =
      "/home/siasun/code/Test/protocol_buff_test/lidar_data.bin";
  const std::string output_directory =
      "/home/siasun/code/Test/protocol_buff_test/pcd_output";
  const int max_frames_to_process = -1; // -1表示处理所有帧

  // 检查输入文件是否存在
  if (!fs::exists(input_filename)) {
    std::cerr << "Error: Input file does not exist: " << input_filename
              << std::endl;
    return 1;
  }

  // 创建输出目录
  if (!fs::exists(output_directory)) {
    if (!fs::create_directories(output_directory)) {
      std::cerr << "Error: Failed to create output directory: "
                << output_directory << std::endl;
      return 1;
    }
    std::cout << "Created output directory: " << output_directory << std::endl;
  }

  // ===== 读取序列化数据文件 =====
  std::fstream input(input_filename, std::ios::in | std::ios::binary);
  if (!input.is_open()) {
    std::cerr << "Error: Could not open file " << input_filename << std::endl;
    return 1;
  }

  sensor::LidarDataBag data_bag;
  if (!data_bag.ParseFromIstream(&input)) {
    std::cerr << "Error: Failed to parse data from " << input_filename
              << std::endl;
    input.close();
    return 1;
  }
  input.close();

  // 开始处理
  auto start_time = std::chrono::high_resolution_clock::now();
  const size_t total_frame_count = data_bag.frames_size();
  const size_t actual_frame_count =
      (max_frames_to_process > 0)
          ? std::min(static_cast<size_t>(max_frames_to_process),
                     total_frame_count)
          : total_frame_count;

  size_t total_points = 0;
  size_t saved_files = 0;

  std::cout << "Processing " << actual_frame_count << " of "
            << total_frame_count << " frames..." << std::endl;

  // ===== 遍历所有帧并保存为PCD =====
  for (int frame_idx = 0; frame_idx < actual_frame_count; ++frame_idx) {
    const sensor::LidarData &frame = data_bag.frames(frame_idx);
    const size_t point_count = frame.points_size();
    total_points += point_count;

    // 创建PCL点云 (只包含XYZI)
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.width = point_count;
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.resize(cloud.width * cloud.height);

    // 填充点云数据 (只提取XYZ和强度)
    for (int pt_idx = 0; pt_idx < point_count; ++pt_idx) {
      const sensor::LidarPoint &point = frame.points(pt_idx);
      pcl::PointXYZI &pcl_point = cloud.points[pt_idx];
      pcl_point.x = point.x();
      pcl_point.y = point.y();
      pcl_point.z = point.z();
      pcl_point.intensity =
          static_cast<float>(point.intensity()); // 将uint32转换为float
    }

    // 生成文件名 (使用帧索引)
    std::ostringstream filename;
    filename << output_directory << "/frame_" << std::setfill('0')
             << std::setw(6) << frame_idx << ".pcd";

    // 保存为二进制PCD文件
    if (pcl::io::savePCDFileBinary(filename.str(), cloud) == 0) {
      saved_files++;

      // 每保存10个文件或第一/最后一个文件时打印进度
      if (saved_files % 10 == 0 || saved_files == 1 ||
          saved_files == actual_frame_count) {
        auto elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::high_resolution_clock::now() - start_time)
                .count();

        std::cout << "Saved frame " << frame_idx << " (" << point_count
                  << " points)"
                  << " to " << fs::path(filename.str()).filename() << " ["
                  << saved_files << "/" << actual_frame_count << "]"
                  << " (" << elapsed << " ms elapsed)" << std::endl;
      }
    } else {
      std::cerr << "Error: Failed to save PCD file: " << filename.str()
                << std::endl;
    }
  }

  // ===== 性能统计 =====
  auto total_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::high_resolution_clock::now() - start_time)
                        .count();

  std::cout << "\n===== Processing Summary =====" << std::endl;
  std::cout << "Input file: " << input_filename << std::endl;
  std::cout << "Total frames: " << total_frame_count << std::endl;
  std::cout << "Frames processed: " << actual_frame_count << std::endl;
  std::cout << "Total points processed: " << total_points << std::endl;
  std::cout << "PCD files saved: " << saved_files << " to " << output_directory
            << std::endl;
  std::cout << "Processing time: " << total_time << " ms" << std::endl;

  if (total_time > 0) {
    std::cout << "Points per second: " << (total_points * 1000.0) / total_time
              << " pt/ms" << std::endl;
    std::cout << "Frames per second: "
              << (actual_frame_count * 1000.0) / total_time << " fps"
              << std::endl;
  }

  google::protobuf::ShutdownProtobufLibrary(); // 清理Protobuf资源
  return 0;
}
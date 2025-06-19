#include "sensor.pb.h"
#include <chrono>
#include <cmath>
#include <filesystem> // 用于创建目录
#include <fstream>
#include <iomanip> // 用于设置文件名格式
#include <iostream>
#include <random>
#include <string>
#include <thread>
#include <vector>

using namespace std;
using namespace std::chrono;

namespace fs = std::filesystem;

// 生成随机浮点数
float random_float(mt19937 &gen, float min_val, float max_val) {
  uniform_real_distribution<float> dist(min_val, max_val);
  return dist(gen);
}

// 生成随机整数
int random_int(mt19937 &gen, int min_val, int max_val) {
  uniform_int_distribution<int> dist(min_val, max_val);
  return dist(gen);
}

// 生成一帧激光雷达数据
sensor::LivoxPointCloud generate_lidar_frame(mt19937 &gen,
                                             const string &sensor_id,
                                             int64_t timestamp,
                                             int point_count) {
  sensor::LivoxPointCloud lidar_data;
  lidar_data.set_timestamp(timestamp);
  lidar_data.set_sensor_id(sensor_id);
  lidar_data.set_point_num(point_count);

  // 生成点云
  for (int i = 0; i < point_count; i++) {
    auto *point = lidar_data.add_points();
    float angle = 2 * M_PI * i / point_count;
    float distance = random_float(gen, 1.0f, 20.0f);
    float height = random_float(gen, -1.0f, 1.0f);

    sensor::PointXYZI tmp_p_xyzi;

    // 模拟不同形状的点云
    int pattern = i % 4;
    switch (pattern) {
    case 0: // 标准点云
      tmp_p_xyzi.set_x(distance * cos(angle));
      tmp_p_xyzi.set_y(distance * sin(angle));
      tmp_p_xyzi.set_z(height);
      break;
    case 1: // 方型障碍物
      tmp_p_xyzi.set_x(cos(angle) < 0 ? -random_float(gen, 2.0f, 5.0f)
                                      : random_float(gen, 2.0f, 5.0f));
      tmp_p_xyzi.set_y(sin(angle) < 0 ? -random_float(gen, 2.0f, 5.0f)
                                      : random_float(gen, 2.0f, 5.0f));
      tmp_p_xyzi.set_z(0);
      break;
    case 2: // 圆柱形障碍物
      tmp_p_xyzi.set_x(3 * cos(angle));
      tmp_p_xyzi.set_y(3 * sin(angle));
      tmp_p_xyzi.set_z(height);
      break;
    default: // 地面
      tmp_p_xyzi.set_x(distance * cos(angle));
      tmp_p_xyzi.set_y(distance * sin(angle));
      tmp_p_xyzi.set_z(-0.5);
      break;
    }

    tmp_p_xyzi.set_intensity(random_int(gen, 0, 255));
    point->set_allocated_point(&tmp_p_xyzi);
    point->set_tag(random_int(gen, 0, 5));
    point->set_line(random_int(gen, 0, 63));
    point->set_offset_time(i * 100); // 100ns per point
  }

  return lidar_data;
}

// 生成一帧IMU数据
sensor::ImuData generate_imu_frame(mt19937 &gen, const string &sensor_id,
                                   int64_t timestamp, int motion_pattern,
                                   int frame_index) {
  sensor::ImuData imu_data;
  imu_data.set_timestamp(timestamp);
  imu_data.set_sensor_id(sensor_id);

  float acc_x, acc_y, acc_z;
  float gyro_x, gyro_y, gyro_z;

  // 根据运动模式生成不同数据
  switch (motion_pattern) {
  case 0: // 静止状态（只有重力）
    acc_x = 0.0f;
    acc_y = 0.0f;
    acc_z = 1.0f; // Z轴重力
    gyro_x = 0.0f;
    gyro_y = 0.0f;
    gyro_z = 0.0f;
    break;
  case 1: // 加速运动
    acc_x = 0.2f + 0.1f * sin(frame_index * 0.1f);
    acc_y = 0.05f * sin(frame_index * 0.2f);
    acc_z = 1.0f; // Z轴重力
    gyro_x = 0.01f * sin(frame_index * 0.3f);
    gyro_y = 0.02f * sin(frame_index * 0.4f);
    gyro_z = 0.05f;
    break;
  case 2: // 转弯运动
    acc_x = 0.1f * sin(frame_index * 0.15f);
    acc_y = 0.3f * sin(frame_index * 0.1f);
    acc_z = 1.0f; // Z轴重力
    gyro_x = 0.02f;
    gyro_y = 0.01f;
    gyro_z = 0.2f + 0.1f * sin(frame_index * 0.25f);
    break;
  case 3: // 振动状态
    acc_x = 0.5f * sin(frame_index * 0.5f);
    acc_y = 0.3f * cos(frame_index * 0.7f);
    acc_z = 1.0f + 0.2f * sin(frame_index * 0.3f); // 重力变化
    gyro_x = 0.5f * sin(frame_index * 0.6f);
    gyro_y = 0.4f * cos(frame_index * 0.8f);
    gyro_z = 0.3f * sin(frame_index * 0.4f);
    break;
  case 4: // 自由落体（失重状态）
    acc_x = 0.0f;
    acc_y = 0.0f;
    acc_z = 0.0f; // 失重
    gyro_x = 0.02f * sin(frame_index * 0.2f);
    gyro_y = 0.03f * cos(frame_index * 0.3f);
    gyro_z = 0.01f * sin(frame_index * 0.4f);
    break;
  default: // 随机运动（最复杂状态）
    acc_x = random_float(gen, -0.2f, 0.2f);
    acc_y = random_float(gen, -0.2f, 0.2f);
    acc_z = 1.0f + random_float(gen, -0.2f, 0.2f);
    gyro_x = random_float(gen, -0.1f, 0.1f);
    gyro_y = random_float(gen, -0.1f, 0.1f);
    gyro_z = random_float(gen, -0.1f, 0.1f);
    break;
  }

  // 添加高斯噪声（模拟传感器噪声）
  normal_distribution<float> noise_dist(0.0f, 0.005f);
  acc_x += noise_dist(gen);
  acc_y += noise_dist(gen);
  acc_z += noise_dist(gen);
  gyro_x += noise_dist(gen);
  gyro_y += noise_dist(gen);
  gyro_z += noise_dist(gen);

  // 设置传感器数据
  imu_data.set_acc_x(acc_x);
  imu_data.set_acc_y(acc_y);
  imu_data.set_acc_z(acc_z);
  imu_data.set_gyro_x(gyro_x);
  imu_data.set_gyro_y(gyro_y);
  imu_data.set_gyro_z(gyro_z);

  return imu_data;
}

int main() {
  GOOGLE_PROTOBUF_VERIFY_VERSION; // 初始化Protobuf库

  // 创建随机数生成器
  random_device rd;
  mt19937 gen(rd());

  // 传感器参数
  const string lidar_id = "lidar_16";
  const string imu_id = "imu_9axis";

  // 数据集参数
  const int lidar_frames_count = 8530;       // 激光雷达帧数
  const int lidar_points_per_frame = 40000;  // 每帧激光点数
  const int imu_frames_per_lidar_frame = 20; // 每帧激光对应的IMU帧数
  const int frames_per_file = 100;           // 每文件包含的点云帧数

  // 时间参数
  const double lidar_frame_interval = 0.1; // 激光雷达帧间隔100ms (10Hz)
  const double imu_frame_interval =
      lidar_frame_interval / imu_frames_per_lidar_frame; // IMU帧间隔

  // 设置起始时间戳（当前时间）
  auto start_timepoint = high_resolution_clock::now();
  int64_t start_timestamp =
      duration_cast<nanoseconds>(start_timepoint.time_since_epoch()).count();

  // 创建输出目录
  const string output_dir = "/home/siasun/code/Test/protocol_buff_test/bag";
  if (!fs::exists(output_dir)) {
    if (fs::create_directories(output_dir)) {
      cout << "Created output directory: " << output_dir << endl;
    } else {
      cerr << "Failed to create output directory: " << output_dir << endl;
      return 1;
    }
  }

  auto start_time = high_resolution_clock::now();

  cout << "Starting data generation..." << endl;
  cout << "Lidar frames: " << lidar_frames_count
       << " (points per frame: " << lidar_points_per_frame << ")" << endl;
  cout << "IMU frames: " << lidar_frames_count * imu_frames_per_lidar_frame
       << endl;
  cout << "Frames per file: " << frames_per_file << endl;
  cout << "Total files to generate: "
       << ceil(static_cast<double>(lidar_frames_count) / frames_per_file)
       << endl;

  // 计算总文件数
  const int total_files =
      ceil(static_cast<double>(lidar_frames_count) / frames_per_file);

  // 整体进度统计
  int total_points_generated = 0;
  int total_imu_frames_generated = 0;
  vector<double> file_times;

  // 生成数据 - 每10帧点云保存为一个文件
  for (int file_idx = 0; file_idx < total_files; file_idx++) {
    // 创建GroupBag
    sensor::LivoxGroupBag group_bag;

    // 创建LidarDataBag和ImuDataBag
    auto *lidar_data_bag = group_bag.mutable_lidar_data_bag();
    auto *imu_data_bag = group_bag.mutable_imu_data_bag();

    auto file_start_time = high_resolution_clock::now();

    // 计算当前文件处理的帧范围
    int start_frame = file_idx * frames_per_file;
    int end_frame = min((file_idx + 1) * frames_per_file, lidar_frames_count);
    int frames_in_this_file = end_frame - start_frame;

    // 文件内数据计数
    int file_points = 0;
    int file_imu_frames = 0;

    cout << "\nGenerating file " << (file_idx + 1) << "/" << total_files
         << " with frames " << start_frame << " to " << (end_frame - 1) << endl;

    // 生成文件内容
    for (int lidar_idx = start_frame; lidar_idx < end_frame; lidar_idx++) {
      // 计算激光雷达帧时间戳
      int64_t lidar_timestamp =
          start_timestamp +
          static_cast<int64_t>(lidar_idx * lidar_frame_interval * 1e9);

      // 生成一帧激光雷达数据
      sensor::LivoxPointCloud lidar_frame = generate_lidar_frame(
          gen, lidar_id, lidar_timestamp, lidar_points_per_frame);

      // 添加到激光雷达数据包
      lidar_data_bag->add_frames()->CopyFrom(lidar_frame);
      file_points += lidar_points_per_frame;

      // 生成对应的IMU数据帧
      int motion_pattern = (lidar_idx / (lidar_frames_count / 6)) % 6;

      for (int imu_idx = 0; imu_idx < imu_frames_per_lidar_frame; imu_idx++) {
        // 计算IMU帧时间戳（均匀分布在激光帧之间）
        int64_t imu_timestamp =
            start_timestamp +
            static_cast<int64_t>(
                (lidar_idx + (double)imu_idx / imu_frames_per_lidar_frame) *
                lidar_frame_interval * 1e9);

        // 生成一帧IMU数据
        sensor::ImuData imu_frame = generate_imu_frame(
            gen, imu_id, imu_timestamp, motion_pattern,
            imu_idx + lidar_idx * imu_frames_per_lidar_frame);

        // 添加到IMU数据包
        imu_data_bag->add_imu_data()->CopyFrom(imu_frame);
        file_imu_frames++;
      }

      // 文件内进度显示
      if ((lidar_idx - start_frame) % 2 == 0) {
        auto file_elapsed = duration_cast<milliseconds>(
            high_resolution_clock::now() - file_start_time);
        double frame_percent =
            ((lidar_idx - start_frame + 1) * 100.0) / frames_in_this_file;

        cout << "  Frame " << (lidar_idx - start_frame) << "/"
             << (frames_in_this_file - 1) << " (" << fixed << setprecision(1)
             << frame_percent << "%)"
             << " | Motion pattern: " << motion_pattern << " ("
             << file_elapsed.count() << " ms)" << endl;
      }
    }

    // 序列化保存数据
    stringstream filename;
    filename << output_dir << "/sensor_group_" << (file_idx + 1) << "_of_"
             << total_files << ".bin";

    fstream output(filename.str(), ios::out | ios::binary | ios::trunc);
    if (!group_bag.SerializeToOstream(&output)) {
      cerr << "Failed to write sensor group data to " << filename.str() << endl;
      return 1;
    }
    output.close();

    // 更新整体统计
    total_points_generated += file_points;
    total_imu_frames_generated += file_imu_frames;

    // 文件处理时间
    auto file_time = duration_cast<milliseconds>(high_resolution_clock::now() -
                                                 file_start_time);
    file_times.push_back(file_time.count());

    // 文件统计
    double avg_file_time =
        accumulate(file_times.begin(), file_times.end(), 0.0) /
        file_times.size();

    cout << "  Saved file " << filename.str() << endl;
    cout << "  Points: " << file_points << ", IMU frames: " << file_imu_frames
         << endl;
    cout << "  File size: " << fixed << setprecision(1)
         << (fs::file_size(filename.str()) / (1024.0 * 1024.0)) << " MB"
         << endl;
    cout << "  File processing time: " << file_time.count() << " ms" << endl;
    cout << "  Average file time: " << avg_file_time << " ms" << endl;

    // 预估剩余时间
    if (file_idx < total_files - 1) {
      double remaining_files = total_files - file_idx - 1;
      double remaining_ms = remaining_files * avg_file_time;
      int remaining_min = static_cast<int>(remaining_ms / (1000 * 60));
      int remaining_sec = static_cast<int>(fmod(remaining_ms / 1000, 60));

      cout << "  Estimated time remaining: " << remaining_min << " min "
           << remaining_sec << " sec" << endl;
    }
  }

  // 最终性能统计
  auto total_time =
      duration_cast<milliseconds>(high_resolution_clock::now() - start_time);

  cout << "\n\nData generation complete!" << endl;
  cout << "======================================" << endl;
  cout << "Total files generated: " << total_files << endl;
  cout << "Total lidar frames: " << lidar_frames_count << endl;
  cout << "Total lidar points: " << total_points_generated << endl;
  cout << "Total IMU frames: " << total_imu_frames_generated << endl;
  cout << "Lidar-IMU ratio: "
       << (total_imu_frames_generated / (double)lidar_frames_count) << ":1"
       << endl;
  cout << "Total processing time: " << total_time.count() << " ms (" << fixed
       << setprecision(1) << total_time.count() / 1000.0 << " sec)" << endl;
  cout << "Points per second: "
       << (total_points_generated * 1000.0) / total_time.count()
       << " lidar pts/s" << endl;
  cout << "IMU frames per second: "
       << (total_imu_frames_generated * 1000.0) / total_time.count()
       << " IMU frames/s" << endl;

  // 文件大小信息
  double total_size = 0.0;
  for (int i = 0; i < total_files; i++) {
    stringstream filename;
    filename << output_dir << "/sensor_group_" << (i + 1) << "_of_"
             << total_files << ".bin";
    total_size += fs::file_size(filename.str());
  }

  cout << "Total data size: " << fixed << setprecision(1)
       << (total_size / (1024.0 * 1024.0)) << " MB" << endl;
  cout << "Average file size: " << fixed << setprecision(1)
       << (total_size / (1024.0 * 1024.0 * total_files)) << " MB" << endl;

  cout << "All files saved to: " << output_dir << endl;
  cout << "======================================" << endl;

  google::protobuf::ShutdownProtobufLibrary(); // 清理Protobuf

  return 0;
}
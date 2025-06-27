#include "sensor.pb.h" // 包含IMU数据定义的头文件
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <random>

#include "zstd.h"

using namespace std;

int main() {
  GOOGLE_PROTOBUF_VERIFY_VERSION; // 初始化Protobuf库

  // 创建数据容器
  sensor::ImuDataBag data_bag;
  const string sensor_id = "imu_1";
  const int frames_count = 851730; // 帧数
  const double time_step = 0.01;   // 10ms时间间隔(100Hz)

  // 初始化随机数生成器
  random_device rd;
  mt19937 gen(rd());

  // 正常运动状态下的加速度分布（-0.2到0.2 G）
  normal_distribution<float> accel_dist(0.0f, 0.05f);

  // 正常运动状态下的角速度分布（-0.1到0.1 rad/s）
  normal_distribution<float> gyro_dist(0.0f, 0.02f);

  // 设置起始时间戳（当前时间）
  auto start_timepoint = chrono::high_resolution_clock::now();
  int64_t start_timestamp = chrono::duration_cast<chrono::nanoseconds>(
                                start_timepoint.time_since_epoch())
                                .count();

  auto start_time = chrono::high_resolution_clock::now();

  for (int frame_idx = 0; frame_idx < frames_count; frame_idx++) {
    // 计算当前帧时间戳（每帧间隔10ms）
    int64_t timestamp =
        start_timestamp + static_cast<int64_t>(frame_idx * time_step * 1e9);

    // 创建新数据点并填充基本信息
    sensor::ImuData *imu_data = data_bag.add_imu_data();
    imu_data->set_timestamp(timestamp);
    imu_data->set_sensor_id(sensor_id);

    // 模拟不同类型的运动模式
    float acc_x, acc_y, acc_z;
    float gyro_x, gyro_y, gyro_z;

    // 每200帧改变一次运动状态
    int motion_pattern = (frame_idx / 200) % 6;

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
      acc_x = 0.2f + 0.1f * sin(frame_idx * 0.1f);
      acc_y = 0.05f * sin(frame_idx * 0.2f);
      acc_z = 1.0f; // Z轴重力
      gyro_x = 0.01f * sin(frame_idx * 0.3f);
      gyro_y = 0.02f * sin(frame_idx * 0.4f);
      gyro_z = 0.05f;
      break;

    case 2: // 转弯运动
      acc_x = 0.1f * sin(frame_idx * 0.15f);
      acc_y = 0.3f * sin(frame_idx * 0.1f);
      acc_z = 1.0f; // Z轴重力
      gyro_x = 0.02f;
      gyro_y = 0.01f;
      gyro_z = 0.2f + 0.1f * sin(frame_idx * 0.25f);
      break;

    case 3: // 振动状态
      acc_x = 0.5f * sin(frame_idx * 0.5f);
      acc_y = 0.3f * cos(frame_idx * 0.7f);
      acc_z = 1.0f + 0.2f * sin(frame_idx * 0.3f); // 重力变化
      gyro_x = 0.5f * sin(frame_idx * 0.6f);
      gyro_y = 0.4f * cos(frame_idx * 0.8f);
      gyro_z = 0.3f * sin(frame_idx * 0.4f);
      break;

    case 4: // 自由落体（失重状态）
      acc_x = 0.0f;
      acc_y = 0.0f;
      acc_z = 0.0f; // 失重
      gyro_x = 0.02f * sin(frame_idx * 0.2f);
      gyro_y = 0.03f * cos(frame_idx * 0.3f);
      gyro_z = 0.01f * sin(frame_idx * 0.4f);
      break;

    default: // 随机运动（最复杂状态）
      acc_x = accel_dist(gen);
      acc_y = accel_dist(gen);
      acc_z = 1.0f + accel_dist(gen); // 重力加上变化
      gyro_x = gyro_dist(gen);
      gyro_y = gyro_dist(gen);
      gyro_z = gyro_dist(gen);
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
    imu_data->set_acc_x(acc_x);
    imu_data->set_acc_y(acc_y);
    imu_data->set_acc_z(acc_z);
    imu_data->set_gyro_x(gyro_x);
    imu_data->set_gyro_y(gyro_y);
    imu_data->set_gyro_z(gyro_z);

    // 进度显示
    if (frame_idx % 100 == 0) {
      auto elapsed = chrono::duration_cast<chrono::milliseconds>(
                         chrono::high_resolution_clock::now() - start_time)
                         .count();
      cout << "Generated IMU frame " << frame_idx << "/" << frames_count
           << " | Motion pattern: " << motion_pattern << " (" << elapsed
           << " ms)" << endl;

      // 打印部分数据
      cout << "  Sample data: acc(" << acc_x << ", " << acc_y << ", " << acc_z
           << ") gyro(" << gyro_x << ", " << gyro_y << ", " << gyro_z << ")"
           << endl;
    }
  }

  // 序列化保存数据
  const string filename =
      "/home/siasun/code/Test/protocol_buff_test/imu_data.bin";
  fstream output(filename, ios::out | ios::binary | ios::trunc);
  if (!data_bag.SerializeToOstream(&output)) {
    cerr << "Failed to write IMU data to " << filename << endl;
    return 1;
  }
  output.close();

  std::string imu_data_str;
  if (!data_bag.SerializeToString(&imu_data_str)) {
    cerr << "Failed to write IMU data to " << filename << endl;
    return 1;
  }
  const string filename_zstd =
      "/home/siasun/code/Test/protocol_buff_test/imu_data.zst";

  // 3. 压缩数据
  auto start_compress = std::chrono::high_resolution_clock::now();

  // 计算压缩所需的最大空间
  size_t compress_bound = ZSTD_compressBound(imu_data_str.size());
  std::vector<char> compressed_data(compress_bound);

  // 执行压缩（级别3平衡速度和压缩率）
  size_t compressed_size =
      ZSTD_compress(compressed_data.data(), compress_bound, imu_data_str.data(),
                    imu_data_str.size(), 3);

  auto end_compress = std::chrono::high_resolution_clock::now();

  // 检查压缩错误
  if (ZSTD_isError(compressed_size)) {
    std::cerr << "压缩失败: " << ZSTD_getErrorName(compressed_size)
              << std::endl;
    return 1;
  }

  // 调整向量大小到实际压缩大小
  compressed_data.resize(compressed_size);

  // 4. 保存压缩文件
  std::ofstream comp_file(filename_zstd, std::ios::binary);
  comp_file.write(compressed_data.data(), compressed_data.size());
  comp_file.close();
  cout << "Save Compressed File Success" << endl;

  std::cout << "压缩耗时: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(
                   end_compress - start_compress)
                   .count()
            << " ms\n";
  std::cout << "压缩文件大小: " << compressed_size << " 字节\n";

  // 性能统计
  auto total_time = chrono::duration_cast<chrono::milliseconds>(
                        chrono::high_resolution_clock::now() - start_time)
                        .count();

  cout << "\nSuccessfully generated and saved " << data_bag.imu_data_size()
       << " IMU frames to " << filename << endl;
  cout << "Total processing time: " << total_time << " ms" << endl;
  cout << "Frames per second: " << (frames_count * 1000.0) / total_time
       << " fps" << endl;

  // 文件大小信息
  ifstream in_file(filename, ios::binary | ios::ate);
  cout << "File size: " << in_file.tellg() << " bytes ("
       << (in_file.tellg() / 1024.0 / 1024.0) << " MB)" << endl;
  in_file.close();

  google::protobuf::ShutdownProtobufLibrary(); // 清理Protobuf
  return 0;
}
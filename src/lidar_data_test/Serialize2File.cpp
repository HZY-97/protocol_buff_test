#include "sensor.pb.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <random> // 包含随机数生成库
#include <vector>

using namespace std;

int main() {
  GOOGLE_PROTOBUF_VERIFY_VERSION; // 初始化Protobuf库

  // 创建数据容器
  sensor::LivoxPointCloudBag data_bag;
  const string sensor_id = "lidar_1";
  const int frames_count = 2000;      // 减少帧数用于测试
  const int points_per_frame = 20000; // 减少点数用于测试

  // 初始化随机数生成器
  random_device rd;
  mt19937 gen(rd());

  auto start_time = chrono::high_resolution_clock::now();

  for (int frame_idx = 0; frame_idx < frames_count; frame_idx++) {
    auto now = chrono::high_resolution_clock::now();
    auto timestamp =
        chrono::duration_cast<chrono::nanoseconds>(now.time_since_epoch())
            .count();

    // 创建新帧并填充基本信息
    sensor::LivoxPointCloud *frame = data_bag.add_frames();
    frame->set_timestamp(timestamp);
    frame->set_sensor_id(sensor_id);

    // 批量预分配内存 (关键优化!)
    frame->mutable_points()->Reserve(points_per_frame);

    // 计算当前帧的点云范围
    // 范围递增: 0.1 * (frame_idx + 1)
    const float cube_size = 0.1f * (frame_idx + 1);

    // 创建均匀分布生成器
    uniform_real_distribution<float> dist(-cube_size / 2.0f, cube_size / 2.0f);

    // 批量生成点数据
    vector<sensor::LivoxPoint> temp_points;
    temp_points.reserve(points_per_frame);

    for (int pt_idx = 0; pt_idx < points_per_frame; pt_idx++) {
      sensor::PointXYZI point;
      sensor::LivoxPoint livox_point;

      // 在递增的立方体范围内生成随机点

      point.set_x(dist(gen)); // X坐标在[-size/2, size/2]范围内
      point.set_y(dist(gen)); // Y坐标在[-size/2, size/2]范围内
      point.set_z(dist(gen)); // Z坐标在[-size/2, size/2]范围内

      point.set_intensity(static_cast<uint32_t>(
          abs(point.x() * point.y() * point.z()) * 255 * 10 // 基于位置计算强度
          ));
      livox_point.set_allocated_point(&point);

      livox_point.set_offset_time(timestamp + pt_idx * 1000); // 递增时间偏移
      livox_point.set_tag(pt_idx % 10);
      livox_point.set_line(pt_idx % 32);

      temp_points.push_back(livox_point);
    }

    // 批量赋值 (高效方式)
    for (auto &pt : temp_points) {
      *frame->add_points() = std::move(pt); // 使用move语义
    }

    // 设置点数
    frame->set_point_num(frame->points_size());

    // 进度显示
    if (frame_idx % 10 == 0) {
      auto elapsed = chrono::duration_cast<chrono::milliseconds>(
                         chrono::high_resolution_clock::now() - start_time)
                         .count();
      cout << "Processed frame " << frame_idx << "/" << frames_count
           << " | Cube size: " << cube_size << " m³"
           << " (" << elapsed << " ms)" << endl;
    }
  }

  // 序列化保存数据
  const string filename =
      "/home/siasun/code/Test/protocol_buff_test/lidar_data.bin";
  fstream output(filename, ios::out | ios::binary | ios::trunc);
  if (!data_bag.SerializeToOstream(&output)) {
    cerr << "Failed to write data to " << filename << endl;
    return 1;
  }
  output.close();

  // 性能统计
  auto total_time = chrono::duration_cast<chrono::milliseconds>(
                        chrono::high_resolution_clock::now() - start_time)
                        .count();

  size_t total_points = 0;
  for (const auto &frame : data_bag.frames()) {
    total_points += frame.points_size();
  }

  cout << "\nSuccessfully saved " << data_bag.frames_size() << " frames ("
       << total_points << " points) to " << filename << endl;
  cout << "Total processing time: " << total_time << " ms" << endl;
  cout << "Points per second: " << (total_points * 1000.0) / total_time
       << " pt/ms" << endl;

  google::protobuf::ShutdownProtobufLibrary(); // 清理Protobuf
  return 0;
}
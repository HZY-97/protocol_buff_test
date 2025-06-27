#include "sensor.pb.h"
#include "zstd.h"
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
  const int frames_count = 80;        // 减少帧数用于测试
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

    // 修改后的点生成逻辑
    for (int pt_idx = 0; pt_idx < points_per_frame; pt_idx++) {
      sensor::LivoxPoint *new_point = frame->add_points(); // 直接添加到帧中
      sensor::PointXYZI *point = new sensor::PointXYZI(); // 在堆上创建

      // 设置点属性
      point->set_x(dist(gen));
      point->set_y(dist(gen));
      point->set_z(dist(gen));
      point->set_intensity(static_cast<uint32_t>(
          abs(point->x() * point->y() * point->z()) * 255 * 10));

      // 转移所有权给 new_point
      new_point->set_allocated_point(point);

      // 设置其他属性
      new_point->set_offset_time(timestamp + pt_idx * 1000);
      new_point->set_tag(pt_idx % 10);
      new_point->set_line(pt_idx % 32);
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

  std::string lidar_data_str;
  if (!data_bag.SerializeToString(&lidar_data_str)) {
    cerr << "Failed to write data data to " << filename << endl;
    return 1;
  }
  const string filename_zstd =
      "/home/siasun/code/Test/protocol_buff_test/lidar_data.zst";

  // 3. 压缩数据
  auto start_compress = std::chrono::high_resolution_clock::now();

  // 计算压缩所需的最大空间
  size_t compress_bound = ZSTD_compressBound(lidar_data_str.size());
  std::vector<char> compressed_data(compress_bound);

  // 执行压缩（级别3平衡速度和压缩率）
  size_t compressed_size =
      ZSTD_compress(compressed_data.data(), compress_bound,
                    lidar_data_str.data(), lidar_data_str.size(), 1);

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
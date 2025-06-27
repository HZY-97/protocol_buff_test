#include "sensor.pb.h"
#include "zstd.h"
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <random>
#include <string>
#include <thread>
#include <vector>

using namespace std;
using namespace std::chrono;

namespace fs = std::filesystem;

// 全局互斥锁保护输出
mutex g_cout_mutex;

// 线程安全的日志输出
void safe_cout(const string &msg) {
  lock_guard<mutex> lock(g_cout_mutex);
  cout << msg << endl;
}

void safe_cerr(const string &msg) {
  lock_guard<mutex> lock(g_cout_mutex);
  cerr << msg << endl;
}

// 线程池类
class ThreadPool {
public:
  ThreadPool(size_t threads) : stop(false) {
    for (size_t i = 0; i < threads; ++i) {
      workers.emplace_back([this] {
        for (;;) {
          function<void()> task;
          {
            unique_lock<mutex> lock(this->queue_mutex);
            this->condition.wait(
                lock, [this] { return this->stop || !this->tasks.empty(); });
            if (this->stop && this->tasks.empty())
              return;
            task = move(this->tasks.front());
            this->tasks.pop();
          }
          task();
        }
      });
    }
  }

  template <class F, class... Args>
  auto enqueue(F &&f, Args &&...args)
      -> future<typename result_of<F(Args...)>::type> {
    using return_type = typename result_of<F(Args...)>::type;

    auto task = make_shared<packaged_task<return_type()>>(
        bind(forward<F>(f), forward<Args>(args)...));

    future<return_type> res = task->get_future();
    {
      lock_guard<mutex> lock(queue_mutex);
      if (stop)
        throw runtime_error("enqueue on stopped ThreadPool");
      tasks.emplace([task]() { (*task)(); });
    }
    condition.notify_one();
    return res;
  }

  ~ThreadPool() {
    {
      lock_guard<mutex> lock(queue_mutex);
      stop = true;
    }
    condition.notify_all();
    for (thread &worker : workers)
      worker.join();
  }

private:
  vector<thread> workers;
  queue<function<void()>> tasks;

  mutex queue_mutex;
  condition_variable condition;
  bool stop;
};

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

// 生成一帧激光雷达数据 - 使用智能指针避免内存泄漏
unique_ptr<sensor::LivoxPointCloud>
generate_lidar_frame(mt19937 &gen, const string &sensor_id, int64_t timestamp,
                     int point_count) {
  auto lidar_data = make_unique<sensor::LivoxPointCloud>();
  lidar_data->set_timestamp(timestamp);
  lidar_data->set_sensor_id(sensor_id);
  lidar_data->set_point_num(point_count);

  // 预分配内存提高性能
  lidar_data->mutable_points()->Reserve(point_count);

  // 生成点云
  for (int i = 0; i < point_count; i++) {
    auto *point = lidar_data->add_points();
    float angle = 2 * M_PI * i / point_count;
    float distance = random_float(gen, 1.0f, 20.0f);
    float height = random_float(gen, -1.0f, 1.0f);

    // 使用智能指针管理临时对象
    auto tmp_p_xyzi = make_unique<sensor::PointXYZI>();

    // 模拟不同形状的点云
    int pattern = i % 4;
    switch (pattern) {
    case 0: // 标准点云
      tmp_p_xyzi->set_x(distance * cos(angle));
      tmp_p_xyzi->set_y(distance * sin(angle));
      tmp_p_xyzi->set_z(height);
      break;
    case 1: // 方型障碍物
      tmp_p_xyzi->set_x(cos(angle) < 0 ? -random_float(gen, 2.0f, 5.0f)
                                       : random_float(gen, 2.0f, 5.0f));
      tmp_p_xyzi->set_y(sin(angle) < 0 ? -random_float(gen, 2.0f, 5.0f)
                                       : random_float(gen, 2.0f, 5.0f));
      tmp_p_xyzi->set_z(0);
      break;
    case 2: // 圆柱形障碍物
      tmp_p_xyzi->set_x(3 * cos(angle));
      tmp_p_xyzi->set_y(3 * sin(angle));
      tmp_p_xyzi->set_z(height);
      break;
    default: // 地面
      tmp_p_xyzi->set_x(distance * cos(angle));
      tmp_p_xyzi->set_y(distance * sin(angle));
      tmp_p_xyzi->set_z(-0.5);
      break;
    }

    tmp_p_xyzi->set_intensity(random_int(gen, 0, 255));
    point->set_allocated_point(tmp_p_xyzi.release()); // 转移所有权
    point->set_tag(random_int(gen, 0, 5));
    point->set_line(random_int(gen, 0, 63));
    point->set_offset_time(i * 100); // 100ns per point
  }

  return lidar_data;
}

// 生成一帧IMU数据
unique_ptr<sensor::ImuData>
generate_imu_frame(mt19937 &gen, const string &sensor_id, int64_t timestamp,
                   int motion_pattern, int frame_index) {
  auto imu_data = make_unique<sensor::ImuData>();
  imu_data->set_timestamp(timestamp);
  imu_data->set_sensor_id(sensor_id);

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
  imu_data->set_acc_x(acc_x);
  imu_data->set_acc_y(acc_y);
  imu_data->set_acc_z(acc_z);
  imu_data->set_gyro_x(gyro_x);
  imu_data->set_gyro_y(gyro_y);
  imu_data->set_gyro_z(gyro_z);

  return imu_data;
}

// 压缩任务 - 添加内存管理
void compress_and_save(sensor::LivoxGroupBag *group_bag_ptr,
                       const string &filename) {
  // 检查是否为空
  if (!group_bag_ptr) {
    safe_cerr("错误: 空对象传递给压缩任务: " + filename);
    return;
  }

  string bag_group_data_str;
  if (!group_bag_ptr->SerializeToString(&bag_group_data_str)) {
    safe_cerr("序列化失败: " + filename);
    return;
  }

  // 立即释放Protobuf对象
  delete group_bag_ptr;
  group_bag_ptr = nullptr;

  // 检查序列化数据是否为空
  if (bag_group_data_str.empty()) {
    safe_cerr("序列化数据为空: " + filename);
    return;
  }

  // 计算压缩所需的最大空间
  size_t compress_bound = ZSTD_compressBound(bag_group_data_str.size());
  vector<char> compressed_data(compress_bound);

  // 执行压缩
  auto start_compress = high_resolution_clock::now();
  size_t compressed_size =
      ZSTD_compress(compressed_data.data(), compress_bound,
                    bag_group_data_str.data(), bag_group_data_str.size(), 1);
  auto end_compress = high_resolution_clock::now();
  auto compress_time =
      duration_cast<milliseconds>(end_compress - start_compress).count();

  // 清理原始序列化数据
  string().swap(bag_group_data_str); // 显式释放内存

  // 检查压缩错误
  if (ZSTD_isError(compressed_size)) {
    safe_cerr("压缩失败: " + string(ZSTD_getErrorName(compressed_size)) +
              " | 文件名: " + filename);
    return;
  }

  // 保存压缩文件
  {
    ofstream comp_file(filename, ios::binary);
    if (!comp_file) {
      safe_cerr("无法创建文件: " + filename);
      return;
    }
    comp_file.write(compressed_data.data(), compressed_size);
  } // 保证文件关闭

  // 清理压缩数据
  vector<char>().swap(compressed_data); // 显式释放内存

  // 安全输出信息
  stringstream ss;
  ss << "保存压缩文件成功: " << filename << endl;
  ss << "  压缩耗时: " << compress_time << " ms" << endl;
  ss << "  压缩后大小: " << compressed_size << " 字节";
  safe_cout(ss.str());
}

int main() {
  GOOGLE_PROTOBUF_VERIFY_VERSION; // 初始化Protobuf库

  // 创建线程池（核心数-1，保留一个核心给主线程）
  const size_t num_threads = thread::hardware_concurrency() - 1;
  ThreadPool pool(num_threads > 0 ? num_threads : 2);
  vector<future<void>> compression_tasks;

  // 创建随机数生成器
  random_device rd;
  mt19937 gen(rd());

  // 传感器参数
  const string lidar_id = "lidar_16";
  const string imu_id = "imu_9axis";

  // 数据集参数
  const int lidar_frames_count = 8530;       // 激光雷达帧数
  const int lidar_points_per_frame = 20000;  // 每帧激光点数
  const int imu_frames_per_lidar_frame = 20; // 每帧激光对应的IMU帧数
  const int frames_per_file = 80;            // 每文件包含的点云帧数

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
      safe_cout("Created output directory: " + output_dir);
    } else {
      safe_cerr("Failed to create output directory: " + output_dir);
      return 1;
    }
  }

  const string output_zstd_dir =
      "/home/siasun/code/Test/protocol_buff_test/bag_zstd";
  if (!fs::exists(output_zstd_dir)) {
    if (fs::create_directories(output_zstd_dir)) {
      safe_cout("Created output directory: " + output_zstd_dir);
    } else {
      safe_cerr("Failed to create output directory: " + output_zstd_dir);
      return 1;
    }
  }

  auto start_time = high_resolution_clock::now();

  stringstream init_msg;
  init_msg << "Starting data generation..." << endl;
  init_msg << "Threads in pool: " << (num_threads > 0 ? num_threads : 2)
           << endl;
  init_msg << "Lidar frames: " << lidar_frames_count
           << " (points per frame: " << lidar_points_per_frame << ")" << endl;
  init_msg << "IMU frames: " << lidar_frames_count * imu_frames_per_lidar_frame
           << endl;
  init_msg << "Frames per file: " << frames_per_file << endl;
  init_msg << "Total files to generate: "
           << ceil(static_cast<double>(lidar_frames_count) / frames_per_file);
  safe_cout(init_msg.str());

  // 计算总文件数
  const int total_files =
      ceil(static_cast<double>(lidar_frames_count) / frames_per_file);

  // 整体进度统计
  int total_points_generated = 0;
  int total_imu_frames_generated = 0;
  vector<double> file_times;

  // 生成数据
  for (int file_idx = 0; file_idx < total_files; file_idx++) {
    // 创建GroupBag
    auto group_bag = make_unique<sensor::LivoxGroupBag>();

    // 创建LidarDataBag和ImuDataBag
    auto *lidar_data_bag = group_bag->mutable_lidar_data_bag();
    auto *imu_data_bag = group_bag->mutable_imu_data_bag();

    auto file_start_time = high_resolution_clock::now();

    // 计算当前文件处理的帧范围
    int start_frame = file_idx * frames_per_file;
    int end_frame = min((file_idx + 1) * frames_per_file, lidar_frames_count);
    int frames_in_this_file = end_frame - start_frame;

    // 文件内数据计数
    int file_points = 0;
    int file_imu_frames = 0;

    stringstream file_info;
    file_info << "\nGenerating file " << (file_idx + 1) << "/" << total_files
              << " with frames " << start_frame << " to " << (end_frame - 1);
    safe_cout(file_info.str());

    // 生成文件内容
    for (int lidar_idx = start_frame; lidar_idx < end_frame; lidar_idx++) {
      // 计算激光雷达帧时间戳
      int64_t lidar_timestamp =
          start_timestamp +
          static_cast<int64_t>(lidar_idx * lidar_frame_interval * 1e9);

      // 生成一帧激光雷达数据
      auto lidar_frame = generate_lidar_frame(gen, lidar_id, lidar_timestamp,
                                              lidar_points_per_frame);

      // 添加到激光雷达数据包
      lidar_data_bag->add_frames()->CopyFrom(*lidar_frame);
      file_points += lidar_points_per_frame;

      // 立即释放lidar_frame内存
      lidar_frame.reset();

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
        auto imu_frame = generate_imu_frame(
            gen, imu_id, imu_timestamp, motion_pattern,
            imu_idx + lidar_idx * imu_frames_per_lidar_frame);

        // 添加到IMU数据包
        imu_data_bag->add_imu_data()->CopyFrom(*imu_frame);
        file_imu_frames++;
      }

      // 每5帧释放一次内存
      if ((lidar_idx - start_frame) % 5 == 0) {
        google::protobuf::ShutdownProtobufLibrary();
        GOOGLE_PROTOBUF_VERIFY_VERSION;
      }

      // 文件内进度显示
      if ((lidar_idx - start_frame) % 2 == 0) {
        auto file_elapsed = duration_cast<milliseconds>(
            high_resolution_clock::now() - file_start_time);
        double frame_percent =
            ((lidar_idx - start_frame + 1) * 100.0) / frames_in_this_file;

        stringstream progress;
        progress << "  Frame " << (lidar_idx - start_frame) << "/"
                 << (frames_in_this_file - 1) << " (" << fixed
                 << setprecision(1) << frame_percent << "%)"
                 << " | Motion pattern: " << motion_pattern << " ("
                 << file_elapsed.count() << " ms)";
        safe_cout(progress.str());
      }
    }

    // 序列化保存数据
    stringstream filename_str;
    filename_str << output_dir << "/sensor_group_" << file_idx << "_of_"
                 << total_files << ".bin";

    {
      fstream output(filename_str.str(), ios::out | ios::binary | ios::trunc);
      if (!output) {
        safe_cerr("无法打开文件: " + filename_str.str());
        continue;
      }

      if (!group_bag->SerializeToOstream(&output)) {
        safe_cerr("Failed to write sensor group data to " + filename_str.str());
        continue;
      }
    } // 文件自动关闭

    // 准备压缩任务
    stringstream filename_zstd;
    filename_zstd << output_zstd_dir << "/sensor_group_" << file_idx << "_of_"
                  << total_files << ".zstd";

    auto *group_bag_ptr = group_bag.release(); // 释放所有权

    // 使用线程池提交压缩任务
    compression_tasks.emplace_back(
        pool.enqueue([group_bag_ptr, filename = filename_zstd.str()]() {
          compress_and_save(group_bag_ptr, filename);
        }));

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

    stringstream file_stats;
    file_stats << "  Saved file " << filename_str.str() << endl;
    file_stats << "  Points: " << file_points
               << ", IMU frames: " << file_imu_frames << endl;
    file_stats << "  File size: " << fixed << setprecision(1)
               << (fs::file_size(filename_str.str()) / (1024.0 * 1024.0))
               << " MB" << endl;
    file_stats << "  File processing time: " << file_time.count() << " ms";
    safe_cout(file_stats.str());

    // 预估剩余时间
    if (file_idx < total_files - 1) {
      double remaining_files = total_files - file_idx - 1;
      double remaining_ms = remaining_files * avg_file_time;
      int remaining_min = static_cast<int>(remaining_ms / (1000 * 60));
      int remaining_sec = static_cast<int>(fmod(remaining_ms / 1000, 60));

      stringstream eta;
      eta << "  Estimated time remaining: " << remaining_min << " min "
          << remaining_sec << " sec";
      safe_cout(eta.str());
    }
  }

  // 主线程完成数据生成后，等待所有压缩任务完成
  safe_cout("\nWaiting for all compression tasks to complete...");

  // 跟踪内存使用
  size_t peak_memory = 0;
  int completed = 0;
  int total_tasks = compression_tasks.size();

  while (completed < total_tasks) {
    completed = 0;
    for (auto &task : compression_tasks) {
      if (task.wait_for(chrono::seconds(0)) == future_status::ready) {
        completed++;
      }
    }

    // 检查内存使用
    ifstream meminfo("/proc/self/status");
    string line;
    while (getline(meminfo, line)) {
      if (line.find("VmPeak:") != string::npos) {
        size_t kb = stoul(line.substr(7));
        if (kb > peak_memory)
          peak_memory = kb;
        break;
      }
    }

    if (completed < total_tasks) {
      stringstream progress;
      progress << "压缩进度: " << completed << "/" << total_tasks << " ("
               << fixed << setprecision(1) << (100.0 * completed / total_tasks)
               << "%) | "
               << "内存峰值: " << peak_memory / 1024.0 << " MB";
      safe_cout(progress.str());
      this_thread::sleep_for(chrono::seconds(1));
    }
  }

  // 最终性能统计
  auto total_time =
      duration_cast<milliseconds>(high_resolution_clock::now() - start_time);

  stringstream summary;
  summary << "\n\n数据生成完成!" << endl;
  summary << "======================================" << endl;
  summary << "总文件数: " << total_files << endl;
  summary << "激光雷达帧数: " << lidar_frames_count << endl;
  summary << "激光雷达点数: " << total_points_generated << endl;
  summary << "IMU帧数: " << total_imu_frames_generated << endl;
  summary << "耗时: " << total_time.count() << " ms (" << fixed
          << setprecision(1) << total_time.count() / 1000.0 << " 秒)" << endl;
  summary << "内存峰值: " << peak_memory / 1024.0 << " MB" << endl;
  summary << "所有文件保存至: " << output_dir << " 和 " << output_zstd_dir;
  safe_cout(summary.str());

  google::protobuf::ShutdownProtobufLibrary(); // 清理Protobuf
  return 0;
}
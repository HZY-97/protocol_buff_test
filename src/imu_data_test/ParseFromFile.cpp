#include "sensor.pb.h" // 包含IMU数据定义的头文件
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;
using namespace Eigen;

// 四元数操作工具类
class QuaternionUtil {
public:
  // 从旋转向量转换为四元数
  static Quaterniond fromRotationVector(const Vector3d &rv) {
    double angle = rv.norm();
    if (angle < 1e-10) {
      return Quaterniond::Identity();
    }
    Vector3d axis = rv / angle;
    return Quaterniond(AngleAxisd(angle, axis));
  }

  // 将四元数转换为旋转向量
  static Vector3d toRotationVector(const Quaterniond &q) {
    AngleAxisd aa(q);
    return aa.angle() * aa.axis();
  }

  // 四元数导数
  static Quaterniond derivative(const Quaterniond &q, const Vector3d &omega) {
    Quaterniond dq;
    dq.w() = -0.5 * (q.x() * omega.x() + q.y() * omega.y() + q.z() * omega.z());
    dq.x() = 0.5 * (q.w() * omega.x() - q.z() * omega.y() + q.y() * omega.z());
    dq.y() = 0.5 * (q.z() * omega.x() + q.w() * omega.y() - q.x() * omega.z());
    dq.z() = 0.5 * (-q.y() * omega.x() + q.x() * omega.y() + q.w() * omega.z());
    return dq;
  }
};

// IMU积分器类
class ImuIntegrator {
public:
  ImuIntegrator() { reset(); }

  void reset() {
    position = Vector3d::Zero();
    velocity = Vector3d::Zero();
    orientation = Quaterniond::Identity();
    gravity = Vector3d(0, 0, -9.81); // 重力向量 (m/s^2)
    bias_acc = Vector3d::Zero();
    bias_gyro = Vector3d::Zero();
    timestamp = -1.0;
  }

  // 处理IMU数据点
  void processImuData(const sensor::ImuData &imu_data) {
    double current_time =
        static_cast<double>(imu_data.timestamp()) * 1e-9; // ns -> s
    Vector3d acc(imu_data.acc_x(), imu_data.acc_y(), imu_data.acc_z());
    Vector3d gyro(imu_data.gyro_x(), imu_data.gyro_y(), imu_data.gyro_z());

    // 补偿偏置
    acc -= bias_acc;
    gyro -= bias_gyro;

    if (timestamp < 0) {
      // 第一帧 - 初始化
      timestamp = current_time;
      return;
    }

    // 计算时间差 (秒)
    double dt = current_time - timestamp;
    if (dt <= 0) {
      // 无效时间差
      return;
    }

    // 姿态积分 (使用四元数)
    Quaterniond dq = QuaternionUtil::derivative(orientation, gyro);
    Quaterniond q_dot = dq;

    // 更新四元数
    orientation.coeffs() += q_dot.coeffs() * dt;
    orientation.normalize();

    // 加速度积分
    // 将加速度从物体坐标系转换到世界坐标系
    Matrix3d R = orientation.toRotationMatrix();
    Vector3d acc_world = R * acc;

    // 补偿重力
    Vector3d linear_acc = acc_world - gravity;

    // 更新速度和位置 (使用梯形积分)
    Vector3d vel_prev = velocity;
    velocity += linear_acc * dt;
    position += (vel_prev + velocity) * 0.5 * dt;

    // 更新时间戳
    timestamp = current_time;
  }

  // 获取当前位姿
  Vector3d getPosition() const { return position; }
  Quaterniond getOrientation() const { return orientation; }

private:
  Vector3d position;       // 3D位置 (m)
  Vector3d velocity;       // 速度 (m/s)
  Quaterniond orientation; // 姿态 (四元数)
  Vector3d gravity;        // 重力向量 (m/s^2)
  Vector3d bias_acc;       // 加速度计偏置
  Vector3d bias_gyro;      // 陀螺仪偏置
  double timestamp;        // 当前时间戳 (s)
};

int main(int argc, char **argv) {

  const string input_filename =
      "/home/siasun/code/Test/protocol_buff_test/imu_data.bin";
  const string output_filename =
      "/home/siasun/code/Test/protocol_buff_test/imu_trajectory.pcd";

  google::protobuf::ShutdownProtobufLibrary(); // 清理之前的Protobuf实例
  GOOGLE_PROTOBUF_VERIFY_VERSION;              // 初始化Protobuf库

  // 创建数据容器
  sensor::ImuDataBag data_bag;

  // 从文件读取数据
  fstream input(input_filename, ios::in | ios::binary);
  if (!input) {
    cerr << "Error: Failed to open input file " << input_filename << endl;
    return 1;
  }
  if (!data_bag.ParseFromIstream(&input)) {
    cerr << "Error: Failed to parse IMU data from " << input_filename << endl;
    return 1;
  }
  input.close();

  cout << "Successfully loaded " << data_bag.imu_data_size()
       << " IMU frames from " << input_filename << endl;

  // 创建积分器
  ImuIntegrator integrator;

  // 创建点云保存轨迹
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  auto start_time = chrono::high_resolution_clock::now();

  // 处理所有IMU数据
  for (int i = 0; i < data_bag.imu_data_size(); i++) {
    const sensor::ImuData &imu_data = data_bag.imu_data(i);

    // 积分处理
    integrator.processImuData(imu_data);

    // 每100帧打印一次状态
    if (i % 100 == 0) {
      Vector3d pos = integrator.getPosition();
      Quaterniond orient = integrator.getOrientation();
      auto elapsed = chrono::duration_cast<chrono::milliseconds>(
                         chrono::high_resolution_clock::now() - start_time)
                         .count();

      Vector3d euler =
          orient.toRotationMatrix().eulerAngles(2, 1, 0) * (180.0 / M_PI);

      cout << "Processed frame " << i << "/" << data_bag.imu_data_size()
           << " | Position: (" << pos.x() << ", " << pos.y() << ", " << pos.z()
           << ")"
           << " | RPY: (" << euler.x() << "°, " << euler.y() << "°, "
           << euler.z() << "°)"
           << " (" << elapsed << " ms)" << endl;
    }

    // 保存位置到点云
    Vector3d pos = integrator.getPosition();
    cloud->push_back(pcl::PointXYZ(pos.x(), pos.y(), pos.z()));
  }

  // 保存点云
  pcl::io::savePCDFileASCII(output_filename, *cloud);
  cout << "\nSaved trajectory with " << cloud->size() << " points to "
       << output_filename << endl;

  // 性能统计
  auto total_time = chrono::duration_cast<chrono::milliseconds>(
                        chrono::high_resolution_clock::now() - start_time)
                        .count();

  cout << "Total processing time: " << total_time << " ms" << endl;
  cout << "Points per second: " << (cloud->size() * 1000.0) / total_time
       << " pts/s" << endl;
  cout << "Path length: " << cloud->size()
       << " points, saved as PCD file: " << output_filename << endl;

  // 清理Protobuf
  google::protobuf::ShutdownProtobufLibrary();

  return 0;
}
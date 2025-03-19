#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <math.h>
#include <thread>
#include <mutex>
#include <bluesea_m300/CustomMsg.h>
#include <bluesea_m300/CustomPoint.h>
#include <queue>
#include "../sdk/sdk.h"

typedef struct
{
  std::string frame_id;
  std::string imu_frame_id;

  ros::Publisher pub_pointcloud;
  std::string topic_pointcloud;
  bool output_pointcloud;

  ros::Publisher pub_custommsg;
  std::string topic_custommsg;
  bool output_custommsg;

  ros::Publisher pub_imu;
  std::string topic_imu;
  bool output_imu;

  std::string lidar_ip;
  int lidar_port;
  int local_port;
  int ptp_enable;

  int frame_package_num;
  // int window;
  // int min_angle;
  // int max_angle;

  // int continuous_times;
  // double dirty_factor;

} ArgData;

void PointCloudCallback(uint32_t handle, const uint8_t dev_type, LidarPacketData *data, void *client_data)
{
  if (data == nullptr)
  {
    return;
  }
  //  printf("point cloud handle: %u, data_num: %d, data_type: %d, length: %d, frame_counter: %d\n",
  //  handle, data->dot_num, data->data_type, data->length, data->frame_cnt);
  if (data->data_type == LIDARPOINTCLOUD)
  {
    LidarCloudPointData *p_point_data = (LidarCloudPointData *)data->data;
    ArgData *argdata = (ArgData *)client_data;
    if (argdata->output_pointcloud)
    {
      sensor_msgs::PointCloud msg;
      int N = data->dot_num;
      auto timestamp = ros::Time::now();
      msg.header.stamp.sec = timestamp.sec;
      msg.header.stamp.nsec = timestamp.nsec;
      msg.header.frame_id = argdata->frame_id;
      msg.points.resize(N);
      msg.channels.resize(1);
      msg.channels[0].name = "intensities";
      msg.channels[0].values.resize(N);

      for (size_t i = 0; i < N; i++)
      {
        msg.points[i].x = p_point_data[i].x;
        msg.points[i].y = p_point_data[i].y;
        msg.points[i].z = p_point_data[i].z;
        msg.channels[0].values[i] = p_point_data[i].reflectivity;
      }
      sensor_msgs::PointCloud2 laserCloudMsg;
      convertPointCloudToPointCloud2(msg, laserCloudMsg);
      argdata->pub_pointcloud.publish(laserCloudMsg);
    }
    if (argdata->output_custommsg)
    {
      bluesea_m300::CustomMsg msg;
      int N = data->dot_num;
      msg.point_num = N;
      msg.lidar_id = 0;
      msg.header.frame_id = argdata->frame_id;
      msg.header.stamp.sec = data->timestamp / 1000000000;
      msg.header.stamp.nsec = data->timestamp % 1000000000;

      msg.header.seq++;
      msg.rsvd = {0, 0, 0};
      for (size_t i = 0; i < N; i++)
      {
        bluesea_m300::CustomPoint point;
        point.x = p_point_data[i].x;
        point.y = p_point_data[i].y;
        point.z = p_point_data[i].z;
        point.reflectivity = p_point_data[i].reflectivity;
        point.offset_time = p_point_data[i].offset_time;
        point.line = p_point_data[i].line;
        point.tag = p_point_data[i].tag;
        msg.points.push_back(point);
      }
      // printf("%d %d\n", N, msg.points.size());
      argdata->pub_custommsg.publish(msg);
    }
  }
  free(data);
}

void ImuDataCallback(uint32_t handle, const uint8_t dev_type, LidarPacketData *data, void *client_data)
{
  if (data == nullptr)
  {
    return;
  }
  // printf("Imu data callback handle:%u, data_num:%u, data_type:%u, length:%u, frame_counter:%u.\n",
  //        handle, data->dot_num, data->data_type, data->length, data->frame_cnt);
  if (data->data_type == LIDARIMUDATA)
  {
    ArgData *argdata = (ArgData *)client_data;
    if (argdata->output_imu)
    {
      LidarImuPointData *p_imu_data = (LidarImuPointData *)data->data;
      sensor_msgs::Imu imu;
      imu.angular_velocity.x = p_imu_data->gyro_x;
      imu.angular_velocity.y = p_imu_data->gyro_y;
      imu.angular_velocity.z = p_imu_data->gyro_z;

      imu.linear_acceleration.x = p_imu_data->linear_acceleration_x;
      imu.linear_acceleration.y = p_imu_data->linear_acceleration_y;
      imu.linear_acceleration.z = p_imu_data->linear_acceleration_z;

      // uint64_t nanosec = data->timestamp;
      auto timestamp = ros::Time::now();
	  imu.header.frame_id = argdata->imu_frame_id;
      imu.header.stamp.sec = timestamp.sec;
      imu.header.stamp.nsec = timestamp.nsec;
      argdata->pub_imu.publish(imu);
    }
  }

  free(data);
}

void LogDataCallback(uint32_t handle, const uint8_t dev_type, char *data, int len)
{
  if (data == nullptr)
  {
    return;
  }
  printf("ID::%d print level:%d msg:%s\n", handle, dev_type, data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Lidar_M300");
  ros::NodeHandle nh("~");
  ArgData argdata;

  nh.param("frame_id", argdata.frame_id, std::string("lidar_frame"));
  nh.param("imu_frame_id", argdata.imu_frame_id, std::string("imu_frame"));

  // 使用绝对话题名称（以'/'开头）
  std::string topic_prefix = "";  // 将在命名空间内发布
  nh.param("topic_pointcloud", argdata.topic_pointcloud, std::string("pointcloud"));
  nh.param("output_pointcloud", argdata.output_pointcloud, true);

  nh.param("topic_custommsg", argdata.topic_custommsg, std::string("custommsg"));
  nh.param("output_custommsg", argdata.output_custommsg, true);

  nh.param("topic_imu", argdata.topic_imu, std::string("imu"));
  nh.param("output_imu", argdata.output_imu, true);

  nh.param("lidar_ip", argdata.lidar_ip, std::string("192.168.158.98"));

  nh.param("lidar_port", argdata.lidar_port, 6543);
  nh.param("local_port", argdata.local_port, 6668);

  nh.param("ptp_enable", argdata.ptp_enable, -1);

  nh.param("frame_package_num", argdata.frame_package_num, 180);

  ShadowsFilterParam sfp;
  nh.param("sfp_enable", sfp.sfp_enable, 1);
  nh.param("window", sfp.window, 1);
  nh.param("min_angle", sfp.min_angle, 5.0);
  nh.param("max_angle", sfp.max_angle, 175.0);
  nh.param("max_angle", sfp.effective_distance, 5.0);
  DirtyFilterParam dfp;

  nh.param("dfp_enable", dfp.dfp_enable, 1);
  nh.param("continuous_times", dfp.continuous_times, 30);
  nh.param("dirty_factor", dfp.dirty_factor, 0.005);

  // 使用NodeHandle而不是带私有命名空间的NodeHandle来发布话题
  ros::NodeHandle public_nh;  // 这将使用节点的命名空间，但不会添加私有节点名称
  if (argdata.output_pointcloud)
    argdata.pub_pointcloud = public_nh.advertise<sensor_msgs::PointCloud2>(argdata.topic_pointcloud, 10);
  if (argdata.output_custommsg)
    argdata.pub_custommsg = public_nh.advertise<bluesea_m300::CustomMsg>(argdata.topic_custommsg, 10);
  if (argdata.output_imu)
    argdata.pub_imu = public_nh.advertise<sensor_msgs::Imu>(argdata.topic_imu, 10);

  BlueSeaLidarSDK::getInstance()->Init();
  int devID = BlueSeaLidarSDK::getInstance()->AddLidar(argdata.lidar_ip, argdata.lidar_port, argdata.local_port,argdata.ptp_enable,argdata.frame_package_num,sfp,dfp);

  BlueSeaLidarSDK::getInstance()->SetPointCloudCallback(devID, PointCloudCallback, &argdata);
  BlueSeaLidarSDK::getInstance()->SetImuDataCallback(devID, ImuDataCallback, &argdata);
  BlueSeaLidarSDK::getInstance()->SetLogDataCallback(devID, LogDataCallback, nullptr);

  while (BlueSeaLidarSDK::getInstance()->read_calib(argdata.lidar_ip.c_str(), argdata.lidar_port) != 0);

  BlueSeaLidarSDK::getInstance()->ConnectLidar(devID);

  while (ros::ok())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}

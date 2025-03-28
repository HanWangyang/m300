#include "../sdk/sdk.h"

void PointCloudCallback(uint32_t handle, const uint8_t dev_type, LidarPacketData *data, void *client_data)
{
	if (data == nullptr)
	{
		return;
	}
	printf("point cloud handle: %u, data_num: %d, data_type: %d, length: %d, frame_counter: %d\n",
		   handle, data->dot_num, data->data_type, data->length, data->frame_cnt);

	if (data->data_type == LIDARPOINTCLOUD)
	{
		/*LidarCloudPointData *p_point_data = (LidarCloudPointData *)data->data;
		for (uint32_t i = 0; i < data->dot_num; i++) {
			printf("%f %f %f\n", p_point_data[i].x, p_point_data[i].y, p_point_data[i].z);
			p_point_data[i].x;
			p_point_data[i].y;
			p_point_data[i].z;
		}*/
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
	// handle, data->dot_num, data->data_type, data->length, data->frame_cnt);

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

int main()
{
	char lidar_addr[] = "192.168.0.60";
	int lidar_port = 6543;
	int listen_port = 6668;


	int ptp_enable = -1;
	int frame_package_num = 180;


	ShadowsFilterParam sfp;
	sfp.sfp_enable=0;
	sfp.window= 1;
	sfp.min_angle= 5.0;
	sfp.max_angle=175.0;
	sfp.effective_distance= 5.0;

	DirtyFilterParam dfp;
	dfp.dfp_enable=0;
	dfp.continuous_times= 30;
	dfp.dirty_factor=0.01;

	BlueSeaLidarSDK::getInstance()->Init();
	int devID = BlueSeaLidarSDK::getInstance()->AddLidar(lidar_addr, lidar_port, listen_port,ptp_enable,frame_package_num,sfp,dfp);

	BlueSeaLidarSDK::getInstance()->SetPointCloudCallback(devID, PointCloudCallback, nullptr);
	BlueSeaLidarSDK::getInstance()->SetImuDataCallback(devID, ImuDataCallback, nullptr);
	BlueSeaLidarSDK::getInstance()->SetLogDataCallback(devID, LogDataCallback, nullptr);
	while (BlueSeaLidarSDK::getInstance()->read_calib(lidar_addr, lidar_port) != 0)
		;
	BlueSeaLidarSDK::getInstance()->ConnectLidar(devID);

	// multiple lidars  ,please make sure lidar ip and   localport  is must be not same

	// char lidar_addr2[] = "192.168.1.10";
	// int lidar_port2 = 6543;
	// int listen_port2 = 6669;
	// int devID2 = BlueSeaLidarSDK::getInstance()->AddLidar(lidar_addr2, lidar_port2, listen_port2);
	// BlueSeaLidarSDK::getInstance()->SetPointCloudCallback(devID2, PointCloudCallback, nullptr);
	// BlueSeaLidarSDK::getInstance()->SetImuDataCallback(devID2, ImuDataCallback, nullptr);
	// BlueSeaLidarSDK::getInstance()->SetLogDataCallback(devID2, LogDataCallback, nullptr);
	// BlueSeaLidarSDK::getInstance()->ConnectLidar(devID2);

	while (1)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}
	BlueSeaLidarSDK::getInstance()->Uninit();
}

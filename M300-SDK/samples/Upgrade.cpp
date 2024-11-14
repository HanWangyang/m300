#include"../sdk/sdk.h"


//集成使用以下数据需在以下回调函数中加锁同步
void LogDataCallback(uint32_t handle, const uint8_t dev_type, char* data, int len) {
	if (data == nullptr) {
		return;
	}
	printf("ID::%d print level:%d msg:%s\n", handle, dev_type, data);
}

int main()
{
	char lidar_addr[] = "192.168.0.231";
	int lidar_port = 6543;
	int listen_port = 6668;
	//std::string  upgrade_file_path = "C:\\Users\\49535\\Desktop\\LDS-M300-E-20241029-115237.lhl";
    std::string  upgrade_file_path = "/home/pacecat/wangzn/M300-SDK/LDS-M300-E-20241029-115237.lhl";
	BlueSeaLidarSDK::getInstance()->Init();
	int devID = BlueSeaLidarSDK::getInstance()->AddLidar(lidar_addr, lidar_port, listen_port);
	BlueSeaLidarSDK::getInstance()->SetLogDataCallback(devID, LogDataCallback, nullptr);

	BlueSeaLidarSDK::getInstance()->ConnectLidar(devID);


	bool ret = BlueSeaLidarSDK::getInstance()->SetLidarUpgrade(devID, upgrade_file_path);
	printf(" lidar upgrade %d\n", ret);
	while (1)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}
	BlueSeaLidarSDK::getInstance()->Uninit();
}
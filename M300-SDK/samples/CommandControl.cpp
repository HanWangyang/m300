#include"../sdk/sdk.h"

void LogDataCallback(uint32_t handle, const uint8_t dev_type, char* data, int len) {
	if (data == nullptr) {
		return;
	}
	printf("ID::%d print level:%d msg:%s\n", handle, dev_type, data);
}

int main()
{
	char lidar_addr[] = "192.168.0.210";
	int lidar_port = 6543;
	int listen_port = 6668;


	int ptp_enable = -1;
	int frame_package_num = 180;
	ShadowsFilterParam sfp;
	sfp.sfp_enable=0;
	DirtyFilterParam dfp;
	dfp.dfp_enable=0;


	BlueSeaLidarSDK::getInstance()->Init();
	int devID = BlueSeaLidarSDK::getInstance()->AddLidar(lidar_addr, lidar_port, listen_port,ptp_enable,frame_package_num,sfp,dfp);

	BlueSeaLidarSDK::getInstance()->SetLogDataCallback(devID, LogDataCallback, nullptr);

	BlueSeaLidarSDK::getInstance()->ConnectLidar(devID);


	/*****************query lidar base info**************************/
	BaseInfo info;	
	BlueSeaLidarSDK::getInstance()->QueryBaseInfo(devID, info);

	printf(" ID:%d uuid:%s  model:%s\n lidarip:%s lidarmask:%s lidargateway:%s lidarport:%d \n uploadip:%s uploadport:%d  uploadfix:%d\n",
		devID,
		info.uuid.c_str(), info.model.c_str(),
		info.lidarip.c_str(), info.lidarmask.c_str(),info.lidargateway.c_str(), info.lidarport,
		info.uploadip.c_str(),info.uploadport, info.uploadfix);
	/*****************query lidar version**************************/
	VersionInfo  info2;
	BlueSeaLidarSDK::getInstance()->QueryVersion(devID, info2);
	printf("ID:%d mcu_ver:%s  motor_ver:%s\n software_ver:%s \n", devID, info2.mcu_ver.c_str(), info2.motor_ver.c_str(), info2.software_ver.c_str());

	/*****************query  lidar is online**************************/
	int isOnline = BlueSeaLidarSDK::getInstance()->QueryDeviceState(devID);
	printf("ID:%d    isOnline:%d\n", devID, isOnline);

	/*****************set action  work   or not work **************************/
	/*bool isok = BlueSeaLidarSDK::getInstance()->SetLidarAction(devID, STOP);
	std::this_thread::sleep_for(std::chrono::milliseconds(3000));
	bool isok2 = BlueSeaLidarSDK::getInstance()->SetLidarAction(devID, START);
	printf("ID:%d  action:%d  %d\n", devID, isok, isok2);*/


	/*****************set lidar network**************************/
	/*char lidar_addr2[] = "192.168.0.232";
	char mask2[] = "255.255.255.0";
	char gateway2[] = "192.168.0.1";
	int lidar_port2 = 6543;
	int listen_port2 = 6668;
	BlueSeaLidarSDK::getInstance()->SetLidarNetWork(devID,lidar_addr2, mask2, gateway2, lidar_port2);*/

	/*****************set lidar upload  network**************************/
	/*char lidar_addr3[] = "192.168.0.47";
	int lidar_port3 = 6668;
	bool ret3=BlueSeaLidarSDK::getInstance()->SetLidarUploadNetWork(devID, lidar_addr3,lidar_port3);
	bool ret4 = BlueSeaLidarSDK::getInstance()->SetLidarUploadFix(devID,true);
	printf("ID:%d  set upload network:%d  %d\n", devID, ret3, ret4);*/


	while (1)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}
	BlueSeaLidarSDK::getInstance()->Uninit();
}
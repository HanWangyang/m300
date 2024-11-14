#ifndef M300_E_PROTOCOL_H
#define M300_E_PROTOCOL_H

#include"define.h"

#define TRANS_BLOCK	0x200
#define RAW_ANG_BOTTOM	1
#define RAW_ANG_TOP		2
#define RAW_ANG_READY	(RAW_ANG_BOTTOM	| RAW_ANG_TOP)
#define PACK_PREAMLE 0X484C
#define DRIFT_RD_PACK 0x4357
#define DRIFT_MAGIC 0xD81F1CA1
#define BLUESEA_PAC_POINT 64	
#define MAX_FANS 1000*10
#define MAX_LIDARS 8
#define CMD_REPEAT   5     //命令重发的次数
#define BUF_SIZE 8*1024
#define LOCALPORT  6668
#define HEARTPORT  6789


#define GS_PACK 0x4753
#define S_PACK 0x0053
#define T_PACK 0x0054
#define C_PACK 0x0043

typedef struct {
	uint16_t code;
	uint16_t len;
	uint16_t idx;
	uint16_t pad;
	uint8_t data[TRANS_BLOCK];
} TransBuf;


struct RawPoint
{
	uint64_t timestamp;
	int32_t angles[2];
	uint16_t dist;
	uint8_t energy;
	uint8_t mirror;

};

enum ColorMode {
	COLOR_ENERGY,
	COLOR_TIME
};

struct CmdHeader
{
	uint16_t sign;
	uint16_t cmd;
	uint16_t sn;
	uint16_t len;
};

typedef struct
{
	double R[3][3];
	float K[3];
	float B[3];
	double Gyro[3];
} IMUDrift;

typedef struct {
	int32_t top[8];
	int32_t min_pitch;
	int32_t max_pitch;
	int32_t bottom;
	int32_t offset;
	int32_t lookup;
} MirrorDrift;

typedef struct {
	IMUDrift imu;
	MirrorDrift mirror;
} Drifts;

typedef struct
{
	uint32_t code;
	union {
		Drifts drifts;
		uint32_t body[126];
	};
	uint32_t crc;
} DriftCalib;



#pragma pack (push,1)

typedef struct
{
	uint8_t Header;
	int16_t Accel_X;
	int16_t Accel_Y;
	int16_t Accel_Z;
	int16_t Gyro_X;
	int16_t Gyro_Y;
	int16_t Gyro_Z;
	int8_t T;
	uint16_t TS;
	uint64_t timestamp;
}IIM42652_FIFO_PACKET_16_ST;


typedef struct {
	uint32_t depth : 24;
	uint32_t theta_hi : 8;
	uint32_t theta_lo : 12;
	uint32_t phi : 20;
	uint8_t reflectivity;
	uint8_t tag;
} BlueSeaLidarSpherPoint;

typedef struct {
	uint8_t version;
	uint16_t length;
	uint16_t time_interval;      /**< unit: 0.1 us */
	uint16_t dot_num;
	uint16_t udp_cnt;
	uint8_t frame_cnt;
	uint8_t data_type;
	uint8_t time_type;
	uint8_t rsvd[12];
	uint32_t crc32;
	uint64_t timestamp;
	BlueSeaLidarSpherPoint points[BLUESEA_PAC_POINT];
} BlueSeaLidarEthernetPacket;


struct LidarMsgHdr
{
	char sign[4];
	uint32_t proto_version;
	char dev_sn[20];
	uint32_t dev_id;
	uint32_t timestamp;
	uint32_t flags;
	uint32_t events;
	uint16_t id;
	uint16_t extra;
	uint32_t zone_actived;
	uint8_t all_states[32];
	uint32_t reserved[11];
};
#pragma pack (pop)


typedef struct {
	uint8_t version;
	uint32_t length;
	uint16_t time_interval;      /**< unit: 0.1 us */
	uint16_t dot_num;
	uint16_t udp_cnt;
	uint8_t frame_cnt;
	uint8_t data_type;
	uint8_t time_type;
	uint8_t rsvd[12];
	uint32_t crc32;
	uint64_t timestamp;
	uint8_t data[1];             /**< Point cloud data. */
} LidarPacketData;


typedef struct {
	uint32_t offset_time;        // offset time relative to the base time
	float x;              // X axis, unit:m
	float y;              // Y axis, unit:m
	float z;               // Z axis, unit:m
	uint8_t reflectivity;     // reflectivity, 0~255
	uint8_t tag;             // bluesea tag
	uint8_t line;             // laser number in lidar
} LidarCloudPointData;

typedef struct {
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float acc_x;
	float acc_y;
	float acc_z;
	float linear_acceleration_x;
	float linear_acceleration_y;
	float linear_acceleration_z;
} LidarImuPointData;

typedef enum {
	LIDARIMUDATA = 0,
	LIDARPOINTCLOUD = 0x01,
} LidarPointDataType;


typedef struct
{
	uint32_t second;
	uint32_t nano_second;
}TIME_ST;

struct DevHeart
{
	char sign[4];
	uint32_t proto_version;
	uint32_t timestamp[2];
	char dev_sn[20];
	char dev_type[16];
	uint32_t version;
	uint32_t dev_id;
	uint8_t ip[4];
	uint8_t mask[4];
	uint8_t gateway[4];
	uint8_t remote_ip[4];
	uint16_t remote_port;
	uint16_t port;
	char reserve[28];
	uint32_t crc;
};

struct EEpromV101
{
	char label[4];			
	uint16_t pp_ver;
	uint16_t size;			
	uint8_t dev_sn[20];
	uint8_t dev_type[16];
	uint32_t dev_id;
	// network
	uint8_t IPv4[4];
	uint8_t mask[4];
	uint8_t gateway[4];
	uint8_t srv_ip[4];
	uint16_t srv_port;
	uint16_t local_port;
	char reserved[9];
	uint8_t target_fixed;
	uint8_t reserved2[74];

};

struct KeepAlive {
	uint32_t world_clock;
	uint32_t mcu_hz;
	uint32_t arrive;
	uint32_t delay;
	uint32_t reserved[4];
};

#define OP_FLASH_ERASE	0xFE00EEEE
#define OP_WRITE_IAP		0xFE00AAAA
#define OP_FRIMWARE_RESET		0xFE00BBBB

#define PACK_PREAMLE 0X484C
#define F_PACK 0x0046
#define TIMEOUT 3
#define BUFFERSIZE 1024
struct FirmwareFile
{
	int code;
	int len;
	int sent;
	uint32_t crc;
	uint8_t date[4];
	uint8_t unused[120];
	char describe[512];
	uint8_t buffer[0];
};

struct FirmwarePart {
	uint32_t offset;
	uint32_t crc;
	uint32_t buf[128];
};

struct FirmWriteResp {
	uint32_t offset;
	int result;
	char msg[128];
};
struct ResendPack
{
	time_t timeout;
	uint32_t tried;
	uint16_t cmd;
	uint16_t sn;
	uint16_t len;
	char buf[2048];
};




typedef void(*LidarCloudPointCallback) (uint32_t handle, const uint8_t dev_type, LidarPacketData *data, void *client_data);
typedef void(*LidarImuDataCallback)(uint32_t handle, const uint8_t dev_type, LidarPacketData* data, void* client_data);
typedef void(*LidarLogDataCallback)(uint32_t handle, const uint8_t dev_type, char* data, int len);



#endif

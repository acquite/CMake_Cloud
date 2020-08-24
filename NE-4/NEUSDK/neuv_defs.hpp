/* neuv_defs.hpp */
/*
 * Software License Agreement (BSD License)
 *
 *  Neuvition SDK Library - www.neuvition.com
 *  Copyright (c) 2016-2019, Neuvition, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 * 
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef __NEUVDEFS_H__
#define __NEUVDEFS_H__

#include <vector>
#include <cstdlib>
#include <cstdint>
#include <opencv2/imgcodecs.hpp>

namespace neuvition {
#define MAX_LASERS   4
#define TOF_DIM      3
#define USE_TAIWAN_EVK 1
#if USE_TAIWAN_EVK
#define TOF_WIDTH 1800
#define TOF_HEIGHT 720
#else
#define TOF_WIDTH 1280
#define TOF_HEIGHT 520
#endif
#define QP(n) (1.0f/(1<<n))
	typedef unsigned char  uint8_t;
	typedef unsigned short uint16_t;
	typedef unsigned int   uint32_t;
	typedef unsigned long  uint64_t;
	typedef uint32_t nvid_t;
	typedef struct NEUV_UNIT { int x; int y; int z; uint8_t r; uint8_t g; uint8_t b; uint8_t lid; uint8_t apd_id; uint16_t row; uint16_t col; uint32_t tof; uint8_t intensity; uint32_t timestamp; } NeuvUnit;
	typedef std::vector<NeuvUnit> NeuvUnits;
	typedef struct NEUV_POINT { int x; int y; int z; uint8_t r; uint8_t g; uint8_t b; uint32_t tof;} NeuvPoint;
	typedef std::vector<NeuvPoint> NeuvPoints;
	typedef std::vector<double> LaserIncidentAngles;
	typedef struct TOF_DATA { uint16_t line_id; uint16_t pixel_id; uint32_t tof_ns; uint8_t laser_id; uint8_t apd_id; uint8_t r; uint8_t g; uint8_t b; uint8_t intensity; uint32_t timestamp; } TofPoint;
	typedef struct IMU_DATA { int32_t sec; int32_t usec; int32_t roll; int32_t yaw; int32_t pitch; int16_t quat_i; int16_t quat_j; int16_t quat_k; int16_t quat_r; } ImuData;
	typedef struct GPRMC{ unsigned char status; double utc; char lat; char lon; double latitude; double longitude; double speed; double course; unsigned int ddmmyy; double variation; char vardirection; char checksum; } NeuGPRMC;
	typedef struct NEU_POSCOR { short ratio_x; short ratio_y; short xmove; short ymove; short xcoeff1;  short xcoeff2; short ycoeff1; short averz; short zcoeff0; } NeuPosCor;


	enum neuv_cmd_code {
		NEUV_CMD_START_SCAN = 1,
		NEUV_CMD_STOP_SCAN  = 2,
		NEUV_CMD_GET_DEVICE_INFO = 3,
		NEUV_CMD_START_STREAM = 6,
		NEUV_CMD_STOP_STREAM  = 7,
		NEUV_CMD_SHUTDOWN   = 8,
		NEUV_CMD_GET_PARAMS = 13,
		NEUV_CMD_SET_RT_PARAM = 14,
		NEUV_CMD_SET_LASER_KHZLV = 15,
		NEUV_CMD_SET_LASER_COUNT = 17,
		NEUV_CMD_SET_FRAME_COUNT = 18,
		NEUV_CMD_SET_TRECT_LINE  = 19,
		NEUV_CMD_SET_TRECT_PIXEL = 20,
		NEUV_CMD_SET_PWM_VALUE = 22,
		NEUV_CMD_SET_SCAN_MODE = 23,
		NEUV_CMD_SET_DAC_VOLTAGE = 25,
		NEUV_CMD_SET_NOISE_POINTS = 26,
		NEUV_CMD_SET_VOXTEL_PERIOD = 27,
		NEUV_CMD_SET_CAMERA_CROP_WH = 29,
		NEUV_CMD_SET_CAMERA_CROP_XY = 30,
		NEUV_CMD_SET_SYS_MODE = 31,
		NEUV_CMD_SET_LASER_TASK_MODE = 32,
		NEUV_CMD_SAVE_CONFIG_DATA = 33,
		NEUV_CMD_SET_PIXEL_OFFSET = 34,
		NEUV_CMD_SET_DENSITY = 35,
		NEUV_CMD_UPDATE_FPGA_VERSION = 38,
		NEUV_CMD_UPDATE_FPGA_STATUS = 39,
		NEUV_CMD_GET_DEVICE_STATUS = 40,
		NEUV_CMD_CAM_IMAGE_UPDATED = 41,
		NEUV_CMD_SET_CAM_X_DEVIATE = 42,
		NEUV_CMD_SET_CAM_Y_DEVIATE = 43,
		NEUV_CMD_SET_CAM_Z_DEVIATE = 44,
		NEUV_CMD_STILL_ALIVE = 45,
		NEUV_CMD_GPS_UPDATED = 46,
		NEUV_CMD_NIL = 0xffff,

		NEUV_CMD_TEMP0 = 400,
		NEUV_CMD_TEMP1 = 401,
		NEUV_CMD_TEMP2 = 402,
		NEUV_CMD_TEMP3 = 403,
		NEUV_CMD_TEMP4 = 404,
		NEUV_CMD_TEMP5 = 405,
		NEUV_CMD_TEMP6 = 406,
		NEUV_CMD_TEMP7 = 407,
		NEUV_CMD_TEMP8 = 408,
		NEUV_CMD_TEMP9 = 409,
		NEUV_CMD_TEMP10 = 410,
		NEUV_CMD_TEMP_RSP = 499,

	NEUV_CMD_LIVE_PUB = 0x10001, // publish
	NEUV_CMD_LIVE_SUB = 0x10002, // subscribe
	NEUV_CMD_LIVE_FRH = 0x10003, // frame head
	NEUV_CMD_LIVE_FRT = 0x10004, // frame tail
	NEUV_CMD_LIVE_PCZ = 0x10005, // frame payload
	NEUV_CMD_LIVE_CLP = 0x10006, // client-side ping-pong
	NEUV_CMD_LIVE_SLP = 0x10007, // server-side ping-pong
	NEUV_CMD_LIVE_PS1 = 0x10008, // start push stream
	NEUV_CMD_LIVE_PS2 = 0x10009, // stop push stream
	NEUV_CMD_LIVE_LS0 = 0x1000a, // get stream info
	NEUV_CMD_LIVE_LS1 = 0x1000b, // start pull stream
	NEUV_CMD_LIVE_LS2 = 0x1000c, // stop pull stream
	NEUV_CMD_LIVE_NIL = 0x1ffff
};

class INeuvEvent {
public:
	virtual ~INeuvEvent() {}
	virtual void on_connect(int,const char*) = 0;
	virtual void on_disconnect(int) = 0;
	virtual void on_response(int,enum neuv_cmd_code) = 0;
	virtual void on_framedata(int,int64_t,const NeuvUnits&, const nvid_t&) = 0;
	virtual void on_imudata(int,int64_t,const NeuvUnits&,const ImuData&) = 0;
	virtual void on_mjpgdata(int, int64_t, cv::Mat) = 0;
	virtual void on_pczdata(bool) = 0;
};

int compute_tof2xyz_table(const double angle_x, const double angle_y, const double bias_y,const int device_type, const NeuPosCor& pos_cor);
void compute_tof2xyz_table_with_multi_lasers(const LaserIncidentAngles& angles,const int device_type, const NeuPosCor& pos_cor);
uint64_t ntohll(uint64_t val);
uint64_t htonll(uint64_t val);

int setup_client(const char* host, const int port, INeuvEvent* handler, const bool flag);
int teardown_client();
int start_scan();
int stop_scan();
int start_stream();
int stop_stream();
int shutdown_device();
int query_device_status();
int query_device_params();
int save_device_params();

int set_event_handler(INeuvEvent* handler);
int set_reconnect_params(const bool enabled,const size_t seconds);
int set_flip_axis(const bool flip_x,const bool flip_y);
int set_laser_power(const int percent);
int set_laser_interval(const int index);
int set_scan_mode(const int index);
int set_frame_frequency(const int fps);
int set_camera_status(const bool enabled);
int get_laser_power();
int get_laser_interval();
LaserIncidentAngles get_laser_angles();
NeuGPRMC get_gps_details();
NeuPosCor get_poscor_params();
int get_frame_frequency();
int get_scan_mode();
int get_device_type();
bool is_camera_on();
int set_data_save(const bool enabled);
int set_mjpg_curl(const bool enabled);
double get_hfov();
double get_vfov();

bool is_connected();
bool is_scanning();
bool is_streaming();
size_t get_lidar_frame_count();
size_t get_lidar_transferred_bytes();


}

#endif /* __NEUVDEFS_H__ */

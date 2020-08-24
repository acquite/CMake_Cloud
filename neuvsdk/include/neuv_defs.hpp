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
		NEUV_CMD_GET_NOISY_GRADE = 45,
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

class DeviceParams {
public:
	DeviceParams() : chroma_depth_(false), print_stats_(false), task_routine_(false),
	camera_enabled_(false), imu_enabled_(false),
	pulsew_enabled_(false),
	sram_reads_per_frame_(50),
	laser_khzlv_(1),
	scan_fr_line_(0), scan_fr_pixel_(0), scan_to_line_(TOF_HEIGHT-1), scan_to_pixel_(TOF_WIDTH-1),
	pwm_voltage_(0), scan_mode_(0), laser_count_(1),
	sac_tpr_val_(1), sac_frq_min_(5), sac_frq_max_(5), sac_pwm_min_(23), sac_pwm_max_(23), dac_voltage_(400), noise_points_(0), vox_period_(0),
	crop_width_(0), crop_height_(0), crop_x_(0), crop_y_(0), sys_mode_(0), laser_mode_(0), offset_evk1_(0), offset_evk2_(0), density_line_(0), density_pixel_(0),
	fpga_ver_(0),laser_probe1temp_(0.0),laser_probe2temp_(0.0), laser_fpgatemp_(0.0) {}

	DeviceParams& operator=(const DeviceParams& p) {
		if (this == &p) { return *this; }
		chroma_depth_=p.chroma_depth_;
		print_stats_=p.print_stats_;
		task_routine_=p.task_routine_;
		camera_enabled_=p.camera_enabled_;
		imu_enabled_=p.imu_enabled_;
		manual_enabled_=p.manual_enabled_;
		pulsew_enabled_=p.pulsew_enabled_;
		sram_reads_per_frame_=p.sram_reads_per_frame_;
		laser_khzlv_=p.laser_khzlv_;
		scan_fr_line_=p.scan_fr_line_;
		scan_fr_pixel_=p.scan_fr_pixel_;
		scan_to_line_=p.scan_to_line_;
		scan_to_pixel_=p.scan_to_pixel_;
		pwm_voltage_=p.pwm_voltage_;
		scan_mode_=p.scan_mode_;
		laser_count_=p.laser_count_;
		sac_tpr_val_=p.sac_tpr_val_;
		sac_frq_min_=p.sac_frq_min_;
		sac_frq_max_=p.sac_frq_max_;
		sac_pwm_min_=p.sac_pwm_min_;
		sac_pwm_max_=p.sac_pwm_max_;
		dac_voltage_=p.dac_voltage_;
		noise_points_=p.noise_points_;
		vox_period_=p.vox_period_;
		crop_width_=p.crop_width_;
		crop_height_=p.crop_height_;
		crop_x_=p.crop_x_;
		crop_y_=p.crop_y_;
		sys_mode_=p.sys_mode_;
		laser_mode_=p.laser_mode_;
		offset_evk1_=p.offset_evk1_;
		offset_evk2_=p.offset_evk2_;
		density_line_=p.density_line_;
		density_pixel_=p.density_pixel_;
		fpga_ver_=p.fpga_ver_;
		laser_onoff_=p.laser_onoff_;
		laser_temperature_=p.laser_temperature_;
		laser_pulserate_=p.laser_pulserate_;
		laser_powervoltage_=p.laser_powervoltage_;
		laser_firstcurrent_=p.laser_firstcurrent_;
		laser_secondcurrent_=p.laser_secondcurrent_;
		laser_pulsewidth_=p.laser_pulsewidth_;
		laser_boardtemp_=p.laser_boardtemp_;
		laser_fpgatemp_=p.laser_fpgatemp_;
		laser_probe1temp_=p.laser_probe1temp_;
		laser_probe2temp_=p.laser_probe2temp_;
		laser_launchtimes_=p.laser_launchtimes_;
		noisy_grade_=p.noisy_grade_;
		gps_gprmc_=p.gps_gprmc_;
		angle_x_=p.angle_x_;
		angle_y_=p.angle_y_;

		spi_status_=p.spi_status_;
		imu_status_=p.imu_status_;
		deviate_x_=p.deviate_x_;
		deviate_y_=p.deviate_y_;
		deviate_z_=p.deviate_z_;
		temp0_=p.temp0_;
		temp1_=p.temp1_;
		temp2_=p.temp2_;
		temp3_=p.temp3_;
		temp4_=p.temp4_;
		temp5_=p.temp5_;
		temp6_=p.temp6_;
		temp7_=p.temp7_;
		temp8_=p.temp8_;
		temp9_=p.temp9_;
		temp10_=p.temp10_;

		return *this;
	}
public:
	inline bool chroma_depth() const { return chroma_depth_; }
	inline bool print_stats() const { return print_stats_; }
	inline bool task_routine() const { return task_routine_; }
	inline bool camera_enabled() const { return camera_enabled_; }
	inline bool imu_enabled() const { return imu_enabled_; }
	inline bool manual_enabled() const { return manual_enabled_; }
	inline bool pulsew_enabled() const { return pulsew_enabled_; }
	inline void chroma_depth(const bool val) { chroma_depth_ = val; }
	inline void print_stats(const bool val) { print_stats_ = val; }
	inline void task_routine(const bool val) { task_routine_ = val; }
	inline void camera_enabled(const bool val) { camera_enabled_ = val; }
	inline void imu_enabled(const bool val) { imu_enabled_ = val; }
	inline void manual_enabled(const bool val) { manual_enabled_ = val; }
	inline void pulsew_enabled(const bool val) { pulsew_enabled_ = val; }

	inline uint16_t sram_reads_per_frame() const { return sram_reads_per_frame_; }
	inline uint16_t laser_khzlv() const { return laser_khzlv_; }
	inline uint16_t scan_fr_line() const { return scan_fr_line_; }
	inline uint16_t scan_fr_pixel() const { return scan_fr_pixel_; }
	inline uint16_t scan_to_line() const { return scan_to_line_; }
	inline uint16_t scan_to_pixel() const { return scan_to_pixel_; }
	inline uint16_t pwm_voltage() const { return pwm_voltage_; }
	inline uint8_t scan_mode() const { return scan_mode_; }
	inline uint8_t laser_count() const { return laser_count_; }
	inline uint8_t sac_tpr_val() const { return sac_tpr_val_; }
	inline uint8_t sac_frq_min() const { return sac_frq_min_; }
	inline uint8_t sac_frq_max() const { return sac_frq_max_; }
	inline uint8_t sac_pwm_min() const { return sac_pwm_min_; }
	inline uint8_t sac_pwm_max() const { return sac_pwm_max_; }
	inline uint16_t dac_voltage() const { return dac_voltage_; }
	inline uint32_t noise_points() const { return noise_points_; }
	inline uint8_t vox_period() const { return vox_period_; }
	inline uint16_t crop_width() const { return crop_width_; }
	inline uint16_t crop_height() const { return crop_height_; }
	inline uint16_t crop_x() const { return crop_x_; }
	inline uint16_t crop_y() const { return crop_y_; }
	inline uint8_t  sys_mode() const { return sys_mode_; }
	inline uint8_t laser_mode() const { return laser_mode_; }
	inline uint8_t density_line() const { return density_line_; }
	inline uint8_t density_pixel() const { return density_pixel_; }
	inline short offset_evk1() const { return offset_evk1_; }
	inline short offset_evk2() const { return offset_evk2_; }
	inline uint32_t fpga_ver() const { return fpga_ver_; }
	inline uint32_t fpga_status() const { return fpga_status_; }

	inline uint8_t  laser_onoff() const { return laser_onoff_; }
	inline float    laser_temperature() const { return laser_temperature_; }
	inline uint16_t laser_pulserate() const { return laser_pulserate_; }
	inline uint16_t laser_powervoltage() const { return laser_powervoltage_; }
	inline uint16_t laser_firstcurrent() const { return laser_firstcurrent_; }
	inline uint16_t laser_secondcurrent() const { return laser_secondcurrent_; }
	inline float    laser_pulsewidth() const { return laser_pulsewidth_; }
	inline uint16_t laser_laserpower() const { return laser_laserpower_; }
	inline float    laser_boardtemp() const { return laser_boardtemp_; }
	inline float    laser_probe1temp() const { return laser_probe1temp_; }
	inline float    laser_probe2temp() const { return laser_probe2temp_; }
	inline float    laser_fpgatemp() const { return laser_fpgatemp_; }
	inline uint8_t  laser_evk1status() const { return laser_evk1status_; }
	inline uint8_t  laser_evk2status() const { return laser_evk2status_; }
	inline uint32_t laser_launchtimes() const { return laser_launchtimes_; }
	inline uint8_t  imu_status() const { return imu_status_; }
	inline uint8_t  spi_status() const { return spi_status_; }
	inline int32_t  deviate_x() const { return deviate_x_; }
	inline int32_t  deviate_y() const { return deviate_y_; }
	inline int32_t  deviate_z() const { return deviate_z_; }
	inline uint8_t  device_type() const { return device_type_; }
	inline uint8_t  laser_type() const { return laser_type_; }
	inline int      noisy_grade() const { return noisy_grade_; }
	inline NeuGPRMC gps_gprmc() const { return gps_gprmc_; }
	inline LaserIncidentAngles laser_angles() const { return laser_angles_; }
	inline void laserangles_clear() { laser_angles_.clear(); };
	inline double angle_x() const { return angle_x_; }
	inline double angle_y() const { return angle_y_; }


	//temp value for debug
	inline int temp0() const { return temp0_; }
	inline int temp1() const { return temp1_; }
	inline int temp2() const { return temp2_; }
	inline int temp3() const { return temp3_; }
	inline int temp4() const { return temp4_; }
	inline int temp5() const { return temp5_; }
	inline int temp6() const { return temp6_; }
	inline int temp7() const { return temp7_; }
	inline int temp8() const { return temp8_; }
	inline int temp9() const { return temp9_; }
	inline int temp10() const { return temp10_; }

	inline int sram_reads_per_frame(const uint16_t val) { sram_reads_per_frame_=val; return 0; }
	inline int laser_khzlv(const uint16_t val) { if (val>6) return -1; laser_khzlv_=val; return 0; }
	inline int scan_fr_line(const uint16_t val) { if (val>719) return -1; scan_fr_line_=val; return 0; }
	inline int scan_fr_pixel(const uint16_t val) { if (val>1779) return -1; scan_fr_pixel_=val; return 0; }
	inline int scan_to_line(const uint16_t val) { if (val>719) return -1; scan_to_line_=val; return 0; }
	inline int scan_to_pixel(const uint16_t val) { if (val>1779) return -1; scan_to_pixel_=val; return 0; }
	inline int pwm_voltage(const uint16_t val) { pwm_voltage_=val; return 0; }
	inline int scan_mode(const uint8_t val) { if (val>2) return -1; scan_mode_=val; return 0; }
	inline int laser_count(const uint8_t val) { if (val>6) return -1; laser_count_=val; return 0; }
	inline int sac_tpr_val(const uint8_t val) { if (val==0) return -1; if (val>30) return -1; sac_tpr_val_=val; return 0; }
	inline int sac_frq_min(const uint8_t val) { if (val==0) return -1; if (val>15) return -1; sac_frq_min_=val; return 0; }
	inline int sac_frq_max(const uint8_t val) { if (val==0) return -1; if (val>15) return -1; sac_frq_max_=val; return 0; }
	inline int sac_pwm_min(const uint8_t val) { if (val>185) return -1; sac_pwm_min_=val; return 0; }
	inline int sac_pwm_max(const uint8_t val) { if (val>185) return -1; sac_pwm_max_=val; return 0; }
	inline int dac_voltage(const uint16_t val) {if (val>1000) return -1; if (val<400) return -1;  dac_voltage_=val; return 0; }
	inline int noise_points(const uint32_t val) { noise_points_=val; return 0; }
	inline int vox_period(const uint8_t val) { vox_period_=val; return 0; }
	inline int crop_width(const uint16_t val) { crop_width_=val; return 0; }
	inline int crop_height(const uint16_t val) { crop_height_=val; return 0; }
	inline int crop_x(const uint16_t val) { crop_x_=val; return 0; }
	inline int crop_y(const uint16_t val) { crop_y_=val; return 0; }
	inline int sys_mode(const uint8_t val) { sys_mode_=val; return 0; }
	inline int laser_mode(const uint8_t val) { laser_mode_=val; return 0; }
	inline int density_line(const uint8_t val) { density_line_=val; return 0; }
	inline int density_pixel(const uint8_t val) { density_pixel_=val; return 0; }
	inline int offset_evk1(const short val) { offset_evk1_=val; return 0; }
	inline int offset_evk2(const short val) { offset_evk2_=val; return 0; }
	inline int fpga_ver(const uint32_t val) { fpga_ver_=val; return 0; }
	inline int fpga_status(const uint32_t val) { fpga_status_=val; return 0; }

	inline int laser_onoff(const uint8_t val) { laser_onoff_ = val; return 0; }
	inline int laser_temperature(const float val) { laser_temperature_=val; return 0; }
	inline int laser_pulserate(const uint16_t val) { laser_pulserate_=val; return 0; }
	inline int laser_powervoltage(const uint16_t val) { laser_powervoltage_=val; return 0; }
	inline int laser_firstcurrent(const uint16_t val) { laser_firstcurrent_=val; return 0; }
	inline int laser_secondcurrent(const uint16_t val) { laser_secondcurrent_=val; return 0; }
	inline int laser_pulsewidth(const float val) { laser_pulsewidth_=val; return 0; }
	inline int laser_laserpower(const uint16_t val) { laser_laserpower_=val; return 0; }
	inline int laser_boardtemp(const float val) { laser_boardtemp_=val; return 0; }
	inline int laser_probe1temp(const float val) { laser_probe1temp_=val; return 0; }
	inline int laser_probe2temp(const float val) { laser_probe2temp_=val; return 0; }
	inline int laser_fpgatemp(const float val) { laser_fpgatemp_=val; return 0; }
	inline int laser_evk1status(const uint8_t val) { laser_evk1status_=val; return 0; }
	inline int laser_evk2status(const uint8_t val) { laser_evk2status_=val; return 0; }
	inline int laser_launchtimes(const uint32_t val) { laser_launchtimes_=val; return 0; }
	inline int imu_status(const uint8_t val) { imu_status_=val; return 0; }
	inline int spi_status(const uint8_t val) { spi_status_=val; return 0; }
	inline int deviate_x(const int32_t val) { deviate_x_=val; return 0; }
	inline int deviate_y(const int32_t val) { deviate_y_=val; return 0; }
	inline int deviate_z(const int32_t val) { deviate_z_=val; return 0; }
	inline int device_type(const uint8_t val) { device_type_=val; return 0; }
	inline int laser_type(const uint8_t val) { laser_type_=val; return 0; }
	inline int noisy_grade(const int val) { noisy_grade_=val; return 0; }
	inline int gps_gprmc(const NeuGPRMC val) { gps_gprmc_=val; return 0; }
	inline int laser_angles(const double val) { laser_angles_.push_back(val*0.1); return 0; }
	inline int angle_x(const double val) { angle_x_=val*0.05; return 0; }
	inline int angle_y(const double val) { angle_y_=val*0.05; return 0; }



	//temp value for debug
	inline int temp0(const int val) { temp0_=val; return 0; }
	inline int temp1(const int val) { temp1_=val; return 0; }
	inline int temp2(const int val) { temp2_=val; return 0; }
	inline int temp3(const int val) { temp3_=val; return 0; }
	inline int temp4(const int val) { temp4_=val; return 0; }
	inline int temp5(const int val) { temp5_=val; return 0; }
	inline int temp6(const int val) { temp6_=val; return 0; }
	inline int temp7(const int val) { temp7_=val; return 0; }
	inline int temp8(const int val) { temp8_=val; return 0; }
	inline int temp9(const int val) { temp9_=val; return 0; }
	inline int temp10(const int val) { temp10_=val; return 0; }

private:
	bool chroma_depth_;
	bool print_stats_;
	bool task_routine_;
	bool camera_enabled_;
	bool imu_enabled_;
	bool manual_enabled_;
	bool pulsew_enabled_;
	uint16_t sram_reads_per_frame_;
	uint16_t laser_khzlv_;
	uint16_t scan_fr_line_;
	uint16_t scan_fr_pixel_;
	uint16_t scan_to_line_;
	uint16_t scan_to_pixel_;
	uint16_t pwm_voltage_;
	uint8_t  scan_mode_;
	uint8_t  laser_count_;
	uint8_t  sac_tpr_val_;
	uint8_t  sac_frq_min_;
	uint8_t  sac_frq_max_;
	uint8_t  sac_pwm_min_;
	uint8_t  sac_pwm_max_;
	uint16_t dac_voltage_;
	uint32_t noise_points_;
	uint8_t  vox_period_;
	uint16_t crop_width_;
	uint16_t crop_height_;
	uint16_t crop_x_;
	uint16_t crop_y_;
	uint8_t  sys_mode_;
	uint8_t  laser_mode_;
	short    offset_evk1_;
	short    offset_evk2_;
	uint8_t  density_line_;
	uint8_t  density_pixel_;
	uint32_t fpga_ver_;
	uint32_t fpga_status_;
	uint8_t  laser_onoff_;
	float    laser_temperature_;
	uint16_t laser_pulserate_;
	uint16_t laser_powervoltage_;
	uint16_t laser_firstcurrent_;  //first  electric current
	uint16_t laser_secondcurrent_; //second  electric current
	float    laser_pulsewidth_;
	uint16_t laser_laserpower_;
	float    laser_boardtemp_;     //board temperature
	float    laser_probe1temp_;    //probe temperature
	float    laser_probe2temp_;    //probe temperature
	float    laser_fpgatemp_;      //fpga core temperature
	uint8_t  laser_evk1status_;
	uint8_t  laser_evk2status_;
	uint8_t  imu_status_;
	uint8_t  spi_status_;
	int32_t  deviate_x_;
	int32_t  deviate_y_;
	int32_t  deviate_z_;
	uint8_t  device_type_;
	uint8_t  laser_type_;
	uint32_t laser_launchtimes_;
	int      noisy_grade_;
	NeuGPRMC gps_gprmc_;
	LaserIncidentAngles laser_angles_;
	double	 angle_x_;
	double 	 angle_y_;


	//temp value for debug
	int temp0_;
	int temp1_;
	int temp2_;
	int temp3_;
	int temp4_;
	int temp5_;
	int temp6_;
	int temp7_;
	int temp8_;
	int temp9_;
	int temp10_;
};

class INeuvEvent {
public:
	virtual ~INeuvEvent() {}
	virtual void on_connect(int,const char*) = 0;
	virtual void on_disconnect(int) = 0;
	virtual void on_response(int,enum neuv_cmd_code) = 0;
	//virtual void on_framedata(int,int64_t,const NeuvUnits&) = 0;
	virtual void on_framedata(int,int64_t,const NeuvUnits&, const nvid_t&) = 0;
	virtual void on_imudata(int,int64_t,const NeuvUnits&,const ImuData&) = 0;
	virtual void on_mjpgdata(int, int64_t, cv::Mat) = 0;
	virtual void on_pczdata(bool) = 0;
};

int compute_tof2xyz_table(const double angle_x, const double angle_y);
void compute_tof2xyz_table_with_multi_lasers(const LaserIncidentAngles& angles);
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

void set_tlog_time(const nvid_t frameid, const int index, const int64_t microsec);
void set_tlog_size(const nvid_t frameid, const int index, const size_t cloudsize);
void reset_tlog(const nvid_t frameid);
void print_tlog(const nvid_t frameid);
const DeviceParams* get_device_params();
DeviceParams* get_device_params_localcopy();
int apply_params_to_device(const enum neuv_cmd_code code);
int laser_status_update();
int set_debug_temp_data(neuv_cmd_code cmd, int data);
int set_frank_filter_enabled(bool enabled);
int set_log_output_enabled(bool enabled);
bool is_task_mode_on();
int set_npvt_value(int value);
int set_task_mode(bool enabled);
int get_scan_line_start();
int get_scan_line_end();
int get_scan_pixel_start();
int get_scan_pixel_end();
int set_scan_line_pixel(int startLine, int endLine, int startPixel, int endPixel);
}

#endif /* __NEUVDEFS_H__ */

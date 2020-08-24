#include "neuv_defs.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <unistd.h>
#include <sys/time.h>

// 获取时间戳
double get_timestamp(void) {
    struct timeval now;
    gettimeofday(&now,0);
    return (double)(now.tv_sec) + (double)(now.tv_usec)/1000000.0;
}

void showretval(int ret) { if(ret == 0) return; std::cout<<"ret:"<<ret<<std::endl;}

// 对INeuvEvent虚类的实现
class myeventh : public neuvition::INeuvEvent
{
public:  
    virtual void on_connect(int code,const char* msg) {
        if (code == 0) { std::cout<<"[NEUVITION]| Connect..." << std::endl; }
    }

    virtual void on_disconnect(int code) {
        if (code == 0) { std::cout<<"[NEUVITION]| Disconnect..." << std::endl; }
    }

    virtual void on_response(int code,enum neuvition::neuv_cmd_code cmd) {

        switch (cmd) {
            case neuvition::NEUV_CMD_START_SCAN: {
                if (code == 0) { std::cout<<"[NEUVITION]| Start scanning..." << std::endl; }
                break;
            }

            case neuvition::NEUV_CMD_STOP_SCAN: {
                if (code == 0) { std::cout<<"[NEUVITION]| Stop scanning..." << std::endl; }
                break;
            }

            case neuvition::NEUV_CMD_START_STREAM: {
                if (code == 0) { std::cout<<"[NEUVITION]| Start data streaming..." << std::endl; }
                break;
            }

            case neuvition::NEUV_CMD_STOP_STREAM: {
                if (code == 0) { std::cout<<"[NEUVITION]| Stop data streaming..." << std::endl; }
                break;
            }

            // 上传底层设备配置参数
            case neuvition::NEUV_CMD_GET_PARAMS: {
                if (code == 0) { std::cout<<"[NEUVITION]| Device parameters synced..." << std::endl; }
                break;
            }

        }
    }

    void on_framedata(int code, int64_t microsec, const neuvition::NeuvUnits& data, const neuvition::nvid_t& frame_id) {
	
	// 通过回调函数on_framedata()返回点云数据
        // 在此函数中进行点云处理
        std::cout<<"[NEUVITION]| On framedata... | Size | "<< data.size()<<std::endl;

        if (data.size() == 0) return;
   

        for (neuvition::NeuvUnits::const_iterator iter = data.begin(); iter != data.end(); iter++) {
            const neuvition::NeuvUnit& np = (*iter);

            float x = np.x*0.001;
            float y = np.y*0.001;
            float z = np.z*0.001;
            printf("%f, %f, %f ",x,y,z);
	    // 如果设置摄像头开启，并开启图像获取，则RGB为真实颜色，否则为128
            uint8_t r = np.r; 
            uint8_t g = np.g; 
            uint8_t b = np.b;
            uint8_t intensity = np.intensity;	
            uint32_t timestamp = np.timestamp;
        }
    }

    virtual void on_imudata(int code,int64_t microsec, const neuvition::NeuvUnits& data,const neuvition::ImuData& imu) {}

    virtual void on_pczdata(bool status) {}

    virtual void on_mjpgdata(int code, int64_t microsec, cv::Mat Mat) {}
};

int main() {
    int ret=0;

    // 可选: X/Y轴翻转
    ret = neuvition::set_flip_axis(false,true);
    showretval(ret);

    neuvition::INeuvEvent* phandler = new myeventh();
    // 建立与底层设备的连接
    ret = neuvition::setup_client("192.168.183.101",6668,phandler/*event-handler*/,false/*auto-reconnect*/);
    sleep(1);

    // 查询状态
    if (neuvition::is_connected()) {
        // 获取设备视场角、设备类型、俯仰角、位置参数
        double hfov = neuvition::get_hfov();
        double vfov = neuvition::get_vfov();
        int device_type = neuvition::get_device_type();
        double bias_y = 0.0;    //设备俯仰角，默认为0
        neuvition::LaserIncidentAngles laserangles = neuvition::get_laser_angles();
        neuvition::NeuPosCor pos_cor = neuvition::get_poscor_params();
        neuvition::compute_tof2xyz_table(hfov, vfov, bias_y, device_type, pos_cor);   
        neuvition::compute_tof2xyz_table_with_multi_lasers(laserangles, device_type, pos_cor);


        // 设置激光功率%
        // 范围值：0~65%
        ret = neuvition::set_laser_power(60);
        usleep(20000);//20ms

        // 设置激光发射频率档位
        // 可选值：1:340KHZ 3:500KHZ
        ret = neuvition::set_laser_interval(1);
        usleep(20000);//20ms

        // 设置点云帧率FPS
        // 可选值：10,15
        ret = neuvition::set_frame_frequency(10);
        usleep(20000);//20ms

        // 可选: 设置摄像头开启，并开启图像获取
        // 如果不使用，直接注释
        ret = neuvition::set_camera_status(true);
        ret = neuvition::set_mjpg_curl(true);

        // 请求底层设备开始扫描
        ret=neuvition::start_scan();
        sleep(1); //1s

        // 请求底层设备开始送点云流
        ret=neuvition::start_stream();
	

        // 等待100秒,从底层设备获取TOF数据并回调
        // 等待时间可自行定义
        sleep(100);

        // 请求底层设备停止送点云流
        ret=neuvition::stop_stream();
        usleep(20000);//20ms

        // 请求底层设备停止扫描
        ret=neuvition::stop_scan();
        usleep(20000);//20ms

        // 断开与底层设备的连接
        ret=neuvition::teardown_client();
        usleep(200000);//20ms

    }


    delete phandler;
    return 0;

}


#pragma once

// ros
#include <ros/ros.h>
// opencv
#include "opencv2/opencv.hpp"
// picam
#include "camera.h"
//std 
#include <chrono>
#include <thread>
#include <mutex>
// messages
#include <communication/MicroPhoneControl.h>
#include <communication/MicroPhoneData.h>
#include <communication/CameraControl.h>
#include <communication/LEDControl.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int32.h>
// common definitions
#include "CommonDefinitions.hpp"

#include <everloop_image.h>
#include <everloop.h>
#include <wishbone_bus.h>

#define WIDTH 640
#define HEIGHT 480

using namespace std;
using namespace cv;
namespace hal = matrix_hal;

struct COLOR{
	COLOR(uint8_t r,uint8_t g,uint8_t b, uint8_t white):r(r),g(g),b(b),white(white){};
	COLOR(){};
	uint8_t r = 0,g = 0,b = 0, white = 0;
};

class MatrixNode{
public:
	MatrixNode();
    ~MatrixNode();

private:
    void camera_control(const communication::CameraControl::ConstPtr& msg);
    void microphone_control(const communication::MicroPhoneControl::ConstPtr& msg);
    void led_control(const communication::LEDControl::ConstPtr& msg);
    void led_lets_do_this();
    static void CameraCallback(CCamera* cam, const void* buffer, int buffer_length);
    static int threshold_value;
    static std::chrono::high_resolution_clock::time_point t1;
    static std::chrono::high_resolution_clock::time_point t2;
    static std::chrono::duration<double> time_span;
    static int ID;
    Mat cameraMatrix, distCoeffs;
    static Mat map1, map2;
    Mat myuv;
    ros::NodeHandle nh;
    static ros::Publisher *video_pub, *cameraID_pub;
    ros::Subscriber camera_control_sub, mic_control_sub, led_control_sub;
    ros::AsyncSpinner *spinner;
    static Mat img, img_rectified, img_gray;
    static unsigned char *img_data, *img_rectified_data, *img_gray_data;
    static bool publish_video_flag;
    vector<COLOR> leds;
    thread *led_thread;
    bool online = true;

    hal::WishboneBus bus;
    hal::Everloop everloop;
    hal::EverloopImage image1d;
    mutex m_lockMutex;
};

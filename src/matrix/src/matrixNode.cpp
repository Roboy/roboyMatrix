#include "matrixNode.hpp"

int MatrixNode::ID = 0;
int MatrixNode::threshold_value = 240;
std::chrono::high_resolution_clock::time_point MatrixNode::t1;
std::chrono::high_resolution_clock::time_point MatrixNode::t2;
std::chrono::duration<double> MatrixNode::time_span;
Mat MatrixNode::img = Mat(HEIGHT, WIDTH, CV_8UC4, MatrixNode::img_data);
Mat MatrixNode::img_rectified = Mat(HEIGHT, WIDTH, CV_8UC4, MatrixNode::img_rectified_data);
Mat MatrixNode::img_gray = Mat(HEIGHT, WIDTH, CV_8UC1, MatrixNode::img_gray_data);
ros::Publisher* MatrixNode::video_pub = NULL;
ros::Publisher* MatrixNode::cameraID_pub = NULL;
unsigned char* MatrixNode::img_data = new unsigned char[WIDTH*HEIGHT*4];
unsigned char* MatrixNode::img_rectified_data = new unsigned char[WIDTH*HEIGHT*4];
unsigned char* MatrixNode::img_gray_data = new unsigned char[WIDTH*HEIGHT];
Mat MatrixNode::map1;
Mat MatrixNode::map2;
bool MatrixNode::publish_video_flag = false;

MatrixNode::MatrixNode() {
    cv::FileStorage fs("/home/roboy/workspace/roboyMatrix/src/intrinsics.xml", cv::FileStorage::READ);
    if (!fs.isOpened()) {
        ROS_ERROR("could not open intrinsics.xml");
        return;
    }
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();

    ID = 0;

    // calculate undistortion mapping
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
                            getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, cv::Size(WIDTH, HEIGHT), 1,
                                                      cv::Size(WIDTH, HEIGHT), 0),
                            cv::Size(WIDTH, HEIGHT), CV_16SC2, map1, map2);

    video_pub = new ros::Publisher;
    *video_pub = nh.advertise<sensor_msgs::Image>("/roboyMatrix/video", 1);

    camera_control_sub = nh.subscribe("/roboyMatrix/camera_control", 100, &MatrixNode::camera_control, this);

    mic_control_sub = nh.subscribe("/roboyMatrix/mic_control", 100, &MatrixNode::microphone_control, this);

    led_control_sub = nh.subscribe("/roboyMatrix/led_control", 100, &MatrixNode::led_control, this);

    cameraID_pub = new ros::Publisher;
    *cameraID_pub = nh.advertise<std_msgs::Int32>("/roboyMatrix/cameraID", 100);

    spinner = new ros::AsyncSpinner(1);
    spinner->start();

    std_msgs::Int32 msg;
    msg.data = ID;
    cameraID_pub->publish(msg);

    img = cv::Mat(HEIGHT, WIDTH, CV_8UC4, img_data);
    img_rectified = cv::Mat(HEIGHT, WIDTH, CV_8UC4, img_rectified_data);

    t1 = std::chrono::high_resolution_clock::now();

    StartCamera(WIDTH, HEIGHT, 90, CameraCallback);

    bus.SpiInit();
    everloop.Setup(&bus);

    leds.resize(36);
    led_lets_do_this();
}

MatrixNode::~MatrixNode() {
    spinner->stop();
    delete spinner;
    delete video_pub;
    delete[] img_data;
    delete[] img_rectified_data;
    delete[] img_gray_data;
    online = false;
    if(led_thread->joinable()) {
            ROS_INFO("waiting for led thread to shut down");
            led_thread->join();
    }
}

void MatrixNode::camera_control(const communication::CameraControl::ConstPtr& msg){
    if(msg->cameraID == ID){ // only react to control for this camera
        switch(msg->control) {
            case toggleVideoStream:
                publish_video_flag = msg->boolValue;
                break;
            case changeThreshold:
                threshold_value = msg->intValue;
                break;
        }
    }
}

void MatrixNode::microphone_control(const communication::MicroPhoneControl::ConstPtr& msg){

}

void MatrixNode::led_control(const communication::LEDControl::ConstPtr& msg){
	lock_guard<std::mutex> lock(m_lockMutex);

	for(uint i=0;i<msg->ledID.size();i++){
		leds[msg->ledID[i]].r = 100;//msg->r[i];
		leds[msg->ledID[i]].g = 200;//msg->g[i];
		leds[msg->ledID[i]].b = 300;//msg->b[i];
		leds[msg->ledID[i]].white = 400;//msg->white[i];
	}
}

void MatrixNode::led_lets_do_this(){
	ros::Rate rate(100);
	while(online){
		uint i=0;
		for (hal::LedValue& led : image1d.leds) {
			led.red = (uint8_t)((float)rand()/RAND_MAX*255.0f);
			led.green = (uint8_t)((float)rand()/RAND_MAX*255.0f);
			led.blue = (uint8_t)((float)rand()/RAND_MAX*255.0f);
			led.white = (uint8_t)((float)rand()/RAND_MAX*255.0f);
			i++;
		}

		everloop.Write(&image1d);
		rate.sleep();
	}
}

void MatrixNode::CameraCallback(CCamera *cam, const void *buffer, int buffer_length) {
    cv::Mat myuv(HEIGHT + HEIGHT / 2, WIDTH, CV_8UC1, (unsigned char *) buffer);
    cv::cvtColor(myuv, img, CV_YUV2RGBA_NV21);

//    imshow("camera",img);
//    waitKey(1);

    static uint counter = 0;
    t2 = std::chrono::high_resolution_clock::now();
    time_span = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
    counter++;

    if(time_span.count()>30){ // reset every 30 seconds
        counter = 0;
        t1 = std::chrono::high_resolution_clock::now();
        std_msgs::Int32 msg;
        msg.data = ID;
        cameraID_pub->publish(msg);
    }

    if(publish_video_flag && counter%3==0){
        cv_bridge::CvImage cvImage;
        img_gray.copyTo(cvImage.image);
        sensor_msgs::Image msg;
        cvImage.toImageMsg(msg);
        msg.encoding = "mono8";
        msg.header.stamp = ros::Time::now();
		static uint next_id = 0;
		msg.header.seq = next_id++;
        video_pub->publish(msg);
   }
}

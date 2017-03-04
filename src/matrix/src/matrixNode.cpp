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

    record_srv =  nh.advertiseService("/roboyMatrix/mic_record", &MatrixNode::record, this);

    led_control_sub = nh.subscribe("/roboyMatrix/led_control", 100, &MatrixNode::led_control, this);

    led_pattern_sub = nh.subscribe("/roboyMatrix/led_pattern", 100, &MatrixNode::led_pattern, this);

    cameraID_pub = new ros::Publisher;
    *cameraID_pub = nh.advertise<std_msgs::Int32>("/roboyMatrix/cameraID", 100);

    spinner = new ros::AsyncSpinner(10);
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
    mics.Setup(&bus);
    mics.CalculateDelays(0, 0, 1000, 320 * 1000);

    leds.resize(36);
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
    // TODO
}

bool MatrixNode::record(communication::Record::Request  &req, communication::Record::Response &res){
    res.recording_raw.resize((req.mics.size() + 1)*req.seconds_to_record * mics.SamplingRate());
    uint32_t step = 0;
    while (true) {
        mics.Read(); /* Reading 8-mics buffer from de FPGA */   
        for (uint32_t s = 0; s < mics.NumberOfSamples(); s++) {
          for (uint16_t i = 0; i < req.mics.size(); i++) { /* mics.Channels()=8 */
            res.recording_raw[i*req.seconds_to_record*mics.SamplingRate()+s] = mics.At(s, req.mics[i]);
          }
          res.recording_raw[req.mics.size()*req.seconds_to_record*mics.SamplingRate()+step] = mics.Beam(s);
          step++;
        }
    if (step == req.seconds_to_record * mics.SamplingRate()) break;
    }
    res.samples = step;
    return true;
}


void MatrixNode::led_control(const communication::LEDControl::ConstPtr& msg){
	lock_guard<std::mutex> lock(m_lockMutex);

	for(uint i=0;i<msg->ledID.size();i++){
		image1d.leds[msg->ledID[i]].red = msg->r[i];
		image1d.leds[msg->ledID[i]].green = msg->g[i];
		image1d.leds[msg->ledID[i]].blue = msg->b[i];
		image1d.leds[msg->ledID[i]].white = msg->white[i];
	}

    everloop.Write(&image1d);
}

void MatrixNode::led_pattern(const communication::LEDPattern::ConstPtr& msg){
    lock_guard<std::mutex> lock(m_lockMutex);
    switch(msg->pattern){
        case FLASH:{
            ros::Rate rate(msg->rate);
            for(uint rep=0; rep<msg->repetitions;rep++){
                for(uint i=0;i<msg->leds.ledID.size();i++){
                    image1d.leds[msg->leds.ledID[i]].red = msg->leds.r[i];
                    image1d.leds[msg->leds.ledID[i]].green = msg->leds.g[i];
                    image1d.leds[msg->leds.ledID[i]].blue = msg->leds.b[i];
                    image1d.leds[msg->leds.ledID[i]].white = msg->leds.white[i];
                }
                everloop.Write(&image1d);
                rate.sleep();
                for(uint i=0;i<msg->leds.ledID.size();i++){
                    image1d.leds[msg->leds.ledID[i]].red = 0;
                    image1d.leds[msg->leds.ledID[i]].green = 0;
                    image1d.leds[msg->leds.ledID[i]].blue = 0;
                    image1d.leds[msg->leds.ledID[i]].white = 0;
                }
                everloop.Write(&image1d);
                rate.sleep();
            }
            break;
        }
        case FLASH_ALL:{
            ros::Rate rate(msg->rate);
            for(uint rep=0; rep<msg->repetitions;rep++){
                for(uint i=0;i<image1d.leds.size();i++){
                    image1d.leds[i].red = msg->leds.r[0];
                    image1d.leds[i].green = msg->leds.g[0];
                    image1d.leds[i].blue = msg->leds.b[0];
                    image1d.leds[i].white = msg->leds.white[0];
                }
                everloop.Write(&image1d);
                rate.sleep();
                for(uint i=0;i<image1d.leds.size();i++){
                    image1d.leds[i].red = 0;
                    image1d.leds[i].green = 0;
                    image1d.leds[i].blue = 0;
                    image1d.leds[i].white = 0;
                }
                everloop.Write(&image1d);
                rate.sleep();
            }
            break;
        }
        case RUN:{
            ros::Rate rate(msg->rate);
            uint rep=0; 
            if(!msg->leds.r.size() && !msg->leds.g.size() && !msg->leds.b.size() && !msg->leds.white.size())
                return;
            while(rep<msg->repetitions){
                if(msg->leds.r.size())
                    image1d.leds[rep%36].red = msg->leds.r[0];
                if(msg->leds.g.size())
                    image1d.leds[rep%36].green = msg->leds.g[0];
                if(msg->leds.b.size())
                    image1d.leds[rep%36].blue = msg->leds.b[0];
                if(msg->leds.white.size())
                    image1d.leds[rep%36].white = msg->leds.white[0];
                
                everloop.Write(&image1d);
                rate.sleep();
                if(msg->leds.r.size()>1)
                    image1d.leds[rep%36].red = msg->leds.r[1];
                else
                    image1d.leds[rep%36].red = 0;
                if(msg->leds.g.size()>1)
                    image1d.leds[rep%36].green = msg->leds.g[1];
                else
                    image1d.leds[rep%36].green = 0;
                if(msg->leds.b.size()>1)
                    image1d.leds[rep%36].blue = msg->leds.b[1];
                else
                    image1d.leds[rep%36].blue = 0;
                if(msg->leds.white.size()>1)
                    image1d.leds[rep%36].white = msg->leds.white[1];
                else
                    image1d.leds[rep%36].white = 0;
                everloop.Write(&image1d);
                rate.sleep();
                rep++;
            }
            break;
        }
        case PULSE:{
            ros::Rate rate(msg->rate*255);
            for(uint rep=0; rep<msg->repetitions;rep++){
                for(uint step=0;step<255;step++){
                    for(uint i=0;i<image1d.leds.size();i++){
                        if(msg->leds.r.size())
                            image1d.leds[i].red = step;
                        if(msg->leds.g.size())
                            image1d.leds[i].green = step;
                        if(msg->leds.b.size())
                            image1d.leds[i].blue = step;
                        if(msg->leds.white.size())
                            image1d.leds[i].white = step;
                    }
                    everloop.Write(&image1d);
                    rate.sleep();
                }
                for(int step=254;step>=0;step--){
                    for(uint i=0;i<image1d.leds.size();i++){
                       if(msg->leds.r.size())
                            image1d.leds[i].red = step;
                        if(msg->leds.g.size())
                            image1d.leds[i].green = step;
                        if(msg->leds.b.size())
                            image1d.leds[i].blue = step;
                        if(msg->leds.white.size())
                            image1d.leds[i].white = step;
                    }
                    everloop.Write(&image1d);
                    rate.sleep();
                }
            }
            break;
        }
        case WAVE:

            break;

    }
}

void MatrixNode::run(){
	ros::Rate rate(100);
	while(ros::ok()){
		// uint i=0;
		// for (hal::LedValue& led : image1d.leds) {
		// 	led.red = leds[i].r;
		// 	led.green = leds[i].g;
		// 	led.blue = leds[i].b;
		// 	led.white = leds[i].white;
		// 	i++;
		// }

		// everloop.Write(&image1d);



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

/**
 * @file /include/interface/main_window.hpp
 *
 * @brief Qt based gui for interface.
 *
 * @date November 2010
 **/
#ifndef interface_MAIN_WINDOW_H
#define interface_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN
#include <QtGui/QMainWindow>
#include <QFileSystemModel>
#include "ui_main_window.h"
#include "ros/ros.h"
#include "communication/CameraControl.h"
#include "communication/LEDControl.h"
#include "communication/LEDPattern.h"
#include "communication/MicroPhoneControl.h"
#include "communication/MicroPhoneData.h"
#include <tinyxml.h>
#include <fstream>

#endif

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace interface {

class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	void on_actionAbout_triggered();
    void cameraControl();
    void ledControl();
	void ledPattern();
    void microPhoneControl();
    void microPhoneData();
Q_SIGNALS:
    void newData(int id);
private:
	Ui::MainWindowDesign ui;
    ros::NodeHandlePtr nh;
    ros::Publisher ledControl_pub, ledPattern_pub, microPhoneControl_pub;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    QFileSystemModel *model;
};

}  // namespace interface

#endif // interface_MAIN_WINDOW_H

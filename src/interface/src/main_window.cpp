/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/interface/interface/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace interface {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));

    QObject::connect(ui.send, SIGNAL(clicked()), this, SLOT(ledPattern()));

    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "interface",
                  ros::init_options::NoSigintHandler |
                          ros::init_options::AnonymousName|
                          ros::init_options::NoRosout);
    }
        ledControl_pub = nh->advertise<communication::LEDControl>("/roboyMatrix/led_control", 1);
        ledPattern_pub = nh->advertise<communication::LEDPattern>("/roboyMatrix/led_pattern", 1);
        microPhoneControl_pub = nh->advertise<communication::MicroPhoneControl>("/roboyMatrix/mic_control", 1);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    model = new QFileSystemModel;
}

MainWindow::~MainWindow() {}

    void MainWindow::cameraControl(){

    }

    void MainWindow::ledControl(){

    }
    void MainWindow::ledPattern(){
        communication::LEDPattern msg;
        msg.rate = atoi(ui.rate->text().toStdString().c_str());
        msg.repetitions = atoi(ui.repetitions->text().toStdString().c_str());
        msg.pattern = atoi(ui.pattern->text().toStdString().c_str());
        communication::LEDControl led_msg;
        if(ui.checkBox->isChecked()){
            led_msg.ledID.push_back(0);
            led_msg.r.push_back(atoi(ui.red->text().toStdString().c_str()));
            led_msg.g.push_back(atoi(ui.green->text().toStdString().c_str()));
            led_msg.b.push_back(atoi(ui.blue->text().toStdString().c_str()));
            led_msg.white.push_back(atoi(ui.white->text().toStdString().c_str()));
        }
        if(ui.checkBox->isChecked()){
            led_msg.ledID.push_back(1);
            led_msg.r.push_back(atoi(ui.red->text().toStdString().c_str()));
            led_msg.g.push_back(atoi(ui.green->text().toStdString().c_str()));
            led_msg.b.push_back(atoi(ui.blue->text().toStdString().c_str()));
            led_msg.white.push_back(atoi(ui.white->text().toStdString().c_str()));
        }

        if(ui.checkBox->isChecked()){
            led_msg.ledID.push_back(2);
            led_msg.r.push_back(atoi(ui.red->text().toStdString().c_str()));
            led_msg.g.push_back(atoi(ui.green->text().toStdString().c_str()));
            led_msg.b.push_back(atoi(ui.blue->text().toStdString().c_str()));
            led_msg.white.push_back(atoi(ui.white->text().toStdString().c_str()));
        }

        if(ui.checkBox->isChecked()){
            led_msg.ledID.push_back(3);
            led_msg.r.push_back(atoi(ui.red->text().toStdString().c_str()));
            led_msg.g.push_back(atoi(ui.green->text().toStdString().c_str()));
            led_msg.b.push_back(atoi(ui.blue->text().toStdString().c_str()));
            led_msg.white.push_back(atoi(ui.white->text().toStdString().c_str()));
        }

        if(ui.checkBox->isChecked()){
            led_msg.ledID.push_back(4);
            led_msg.r.push_back(atoi(ui.red->text().toStdString().c_str()));
            led_msg.g.push_back(atoi(ui.green->text().toStdString().c_str()));
            led_msg.b.push_back(atoi(ui.blue->text().toStdString().c_str()));
            led_msg.white.push_back(atoi(ui.white->text().toStdString().c_str()));
        }

        if(ui.checkBox->isChecked()){
            led_msg.ledID.push_back(5);
            led_msg.r.push_back(atoi(ui.red->text().toStdString().c_str()));
            led_msg.g.push_back(atoi(ui.green->text().toStdString().c_str()));
            led_msg.b.push_back(atoi(ui.blue->text().toStdString().c_str()));
            led_msg.white.push_back(atoi(ui.white->text().toStdString().c_str()));
        }

        if(ui.checkBox->isChecked()){
            led_msg.ledID.push_back(6);
            led_msg.r.push_back(atoi(ui.red->text().toStdString().c_str()));
            led_msg.g.push_back(atoi(ui.green->text().toStdString().c_str()));
            led_msg.b.push_back(atoi(ui.blue->text().toStdString().c_str()));
            led_msg.white.push_back(atoi(ui.white->text().toStdString().c_str()));
        }

        if(ui.checkBox->isChecked()){
            led_msg.ledID.push_back(7);
            led_msg.r.push_back(atoi(ui.red->text().toStdString().c_str()));
            led_msg.g.push_back(atoi(ui.green->text().toStdString().c_str()));
            led_msg.b.push_back(atoi(ui.blue->text().toStdString().c_str()));
            led_msg.white.push_back(atoi(ui.white->text().toStdString().c_str()));
        }

        if(ui.checkBox->isChecked()){
            led_msg.ledID.push_back(8);
            led_msg.r.push_back(atoi(ui.red->text().toStdString().c_str()));
            led_msg.g.push_back(atoi(ui.green->text().toStdString().c_str()));
            led_msg.b.push_back(atoi(ui.blue->text().toStdString().c_str()));
            led_msg.white.push_back(atoi(ui.white->text().toStdString().c_str()));
        }

        if(ui.checkBox->isChecked()){
            led_msg.ledID.push_back(9);
            led_msg.r.push_back(atoi(ui.red->text().toStdString().c_str()));
            led_msg.g.push_back(atoi(ui.green->text().toStdString().c_str()));
            led_msg.b.push_back(atoi(ui.blue->text().toStdString().c_str()));
            led_msg.white.push_back(atoi(ui.white->text().toStdString().c_str()));
        }

        if(ui.checkBox->isChecked()){
            led_msg.ledID.push_back(10);
            led_msg.r.push_back(atoi(ui.red->text().toStdString().c_str()));
            led_msg.g.push_back(atoi(ui.green->text().toStdString().c_str()));
            led_msg.b.push_back(atoi(ui.blue->text().toStdString().c_str()));
            led_msg.white.push_back(atoi(ui.white->text().toStdString().c_str()));
        }

        if(ui.checkBox->isChecked()){
            led_msg.ledID.push_back(11);
            led_msg.r.push_back(atoi(ui.red->text().toStdString().c_str()));
            led_msg.g.push_back(atoi(ui.green->text().toStdString().c_str()));
            led_msg.b.push_back(atoi(ui.blue->text().toStdString().c_str()));
            led_msg.white.push_back(atoi(ui.white->text().toStdString().c_str()));
        }

        if(ui.checkBox->isChecked()){
            led_msg.ledID.push_back(12);
            led_msg.r.push_back(atoi(ui.red->text().toStdString().c_str()));
            led_msg.g.push_back(atoi(ui.green->text().toStdString().c_str()));
            led_msg.b.push_back(atoi(ui.blue->text().toStdString().c_str()));
            led_msg.white.push_back(atoi(ui.white->text().toStdString().c_str()));
        }

        if(ui.checkBox->isChecked()){
            led_msg.ledID.push_back(13);
            led_msg.r.push_back(atoi(ui.red->text().toStdString().c_str()));
            led_msg.g.push_back(atoi(ui.green->text().toStdString().c_str()));
            led_msg.b.push_back(atoi(ui.blue->text().toStdString().c_str()));
            led_msg.white.push_back(atoi(ui.white->text().toStdString().c_str()));
        }

        if(ui.checkBox->isChecked()){
            led_msg.ledID.push_back(14);
            led_msg.r.push_back(atoi(ui.red->text().toStdString().c_str()));
            led_msg.g.push_back(atoi(ui.green->text().toStdString().c_str()));
            led_msg.b.push_back(atoi(ui.blue->text().toStdString().c_str()));
            led_msg.white.push_back(atoi(ui.white->text().toStdString().c_str()));
        }

        if(ui.checkBox->isChecked()){
            led_msg.ledID.push_back(15);
            led_msg.r.push_back(atoi(ui.red->text().toStdString().c_str()));
            led_msg.g.push_back(atoi(ui.green->text().toStdString().c_str()));
            led_msg.b.push_back(atoi(ui.blue->text().toStdString().c_str()));
            led_msg.white.push_back(atoi(ui.white->text().toStdString().c_str()));
        }

        if(ui.checkBox->isChecked()){
            led_msg.ledID.push_back(16);
            led_msg.r.push_back(atoi(ui.red->text().toStdString().c_str()));
            led_msg.g.push_back(atoi(ui.green->text().toStdString().c_str()));
            led_msg.b.push_back(atoi(ui.blue->text().toStdString().c_str()));
            led_msg.white.push_back(atoi(ui.white->text().toStdString().c_str()));
        }

        if(ui.checkBox->isChecked()){
            led_msg.ledID.push_back(17);
            led_msg.r.push_back(atoi(ui.red->text().toStdString().c_str()));
            led_msg.g.push_back(atoi(ui.green->text().toStdString().c_str()));
            led_msg.b.push_back(atoi(ui.blue->text().toStdString().c_str()));
            led_msg.white.push_back(atoi(ui.white->text().toStdString().c_str()));
        }

        if(ui.checkBox->isChecked()){
            led_msg.ledID.push_back(18);
            led_msg.r.push_back(atoi(ui.red->text().toStdString().c_str()));
            led_msg.g.push_back(atoi(ui.green->text().toStdString().c_str()));
            led_msg.b.push_back(atoi(ui.blue->text().toStdString().c_str()));
            led_msg.white.push_back(atoi(ui.white->text().toStdString().c_str()));
        }

        if(ui.checkBox->isChecked()){
            led_msg.ledID.push_back(19);
            led_msg.r.push_back(atoi(ui.red->text().toStdString().c_str()));
            led_msg.g.push_back(atoi(ui.green->text().toStdString().c_str()));
            led_msg.b.push_back(atoi(ui.blue->text().toStdString().c_str()));
            led_msg.white.push_back(atoi(ui.white->text().toStdString().c_str()));
        }

        if(ui.checkBox->isChecked()){
            led_msg.ledID.push_back(20);
            led_msg.r.push_back(atoi(ui.red->text().toStdString().c_str()));
            led_msg.g.push_back(atoi(ui.green->text().toStdString().c_str()));
            led_msg.b.push_back(atoi(ui.blue->text().toStdString().c_str()));
            led_msg.white.push_back(atoi(ui.white->text().toStdString().c_str()));
        }

        if(ui.checkBox->isChecked()){
            led_msg.ledID.push_back(21);
            led_msg.r.push_back(atoi(ui.red->text().toStdString().c_str()));
            led_msg.g.push_back(atoi(ui.green->text().toStdString().c_str()));
            led_msg.b.push_back(atoi(ui.blue->text().toStdString().c_str()));
            led_msg.white.push_back(atoi(ui.white->text().toStdString().c_str()));
        }

        if(ui.checkBox->isChecked()){
            led_msg.ledID.push_back(22);
            led_msg.r.push_back(atoi(ui.red->text().toStdString().c_str()));
            led_msg.g.push_back(atoi(ui.green->text().toStdString().c_str()));
            led_msg.b.push_back(atoi(ui.blue->text().toStdString().c_str()));
            led_msg.white.push_back(atoi(ui.white->text().toStdString().c_str()));
        }

        if(ui.checkBox->isChecked()){
            led_msg.ledID.push_back(23);
            led_msg.r.push_back(atoi(ui.red->text().toStdString().c_str()));
            led_msg.g.push_back(atoi(ui.green->text().toStdString().c_str()));
            led_msg.b.push_back(atoi(ui.blue->text().toStdString().c_str()));
            led_msg.white.push_back(atoi(ui.white->text().toStdString().c_str()));
        }

        if(ui.checkBox->isChecked()){
            led_msg.ledID.push_back(24);
            led_msg.r.push_back(atoi(ui.red->text().toStdString().c_str()));
            led_msg.g.push_back(atoi(ui.green->text().toStdString().c_str()));
            led_msg.b.push_back(atoi(ui.blue->text().toStdString().c_str()));
            led_msg.white.push_back(atoi(ui.white->text().toStdString().c_str()));
        }

        if(ui.checkBox->isChecked()){
            led_msg.ledID.push_back(25);
            led_msg.r.push_back(atoi(ui.red->text().toStdString().c_str()));
            led_msg.g.push_back(atoi(ui.green->text().toStdString().c_str()));
            led_msg.b.push_back(atoi(ui.blue->text().toStdString().c_str()));
            led_msg.white.push_back(atoi(ui.white->text().toStdString().c_str()));
        }

        if(ui.checkBox->isChecked()){
            led_msg.ledID.push_back(26);
            led_msg.r.push_back(atoi(ui.red->text().toStdString().c_str()));
            led_msg.g.push_back(atoi(ui.green->text().toStdString().c_str()));
            led_msg.b.push_back(atoi(ui.blue->text().toStdString().c_str()));
            led_msg.white.push_back(atoi(ui.white->text().toStdString().c_str()));
        }

        if(ui.checkBox->isChecked()){
            led_msg.ledID.push_back(27);
            led_msg.r.push_back(atoi(ui.red->text().toStdString().c_str()));
            led_msg.g.push_back(atoi(ui.green->text().toStdString().c_str()));
            led_msg.b.push_back(atoi(ui.blue->text().toStdString().c_str()));
            led_msg.white.push_back(atoi(ui.white->text().toStdString().c_str()));
        }

        if(ui.checkBox->isChecked()){
            led_msg.ledID.push_back(28);
            led_msg.r.push_back(atoi(ui.red->text().toStdString().c_str()));
            led_msg.g.push_back(atoi(ui.green->text().toStdString().c_str()));
            led_msg.b.push_back(atoi(ui.blue->text().toStdString().c_str()));
            led_msg.white.push_back(atoi(ui.white->text().toStdString().c_str()));
        }

        if(ui.checkBox->isChecked()){
            led_msg.ledID.push_back(29);
            led_msg.r.push_back(atoi(ui.red->text().toStdString().c_str()));
            led_msg.g.push_back(atoi(ui.green->text().toStdString().c_str()));
            led_msg.b.push_back(atoi(ui.blue->text().toStdString().c_str()));
            led_msg.white.push_back(atoi(ui.white->text().toStdString().c_str()));
        }

        if(ui.checkBox->isChecked()){
            led_msg.ledID.push_back(30);
            led_msg.r.push_back(atoi(ui.red->text().toStdString().c_str()));
            led_msg.g.push_back(atoi(ui.green->text().toStdString().c_str()));
            led_msg.b.push_back(atoi(ui.blue->text().toStdString().c_str()));
            led_msg.white.push_back(atoi(ui.white->text().toStdString().c_str()));
        }

        if(ui.checkBox->isChecked()){
            led_msg.ledID.push_back(31);
            led_msg.r.push_back(atoi(ui.red->text().toStdString().c_str()));
            led_msg.g.push_back(atoi(ui.green->text().toStdString().c_str()));
            led_msg.b.push_back(atoi(ui.blue->text().toStdString().c_str()));
            led_msg.white.push_back(atoi(ui.white->text().toStdString().c_str()));
        }

        if(ui.checkBox->isChecked()){
            led_msg.ledID.push_back(32);
            led_msg.r.push_back(atoi(ui.red->text().toStdString().c_str()));
            led_msg.g.push_back(atoi(ui.green->text().toStdString().c_str()));
            led_msg.b.push_back(atoi(ui.blue->text().toStdString().c_str()));
            led_msg.white.push_back(atoi(ui.white->text().toStdString().c_str()));
        }

        if(ui.checkBox->isChecked()){
            led_msg.ledID.push_back(33);
            led_msg.r.push_back(atoi(ui.red->text().toStdString().c_str()));
            led_msg.g.push_back(atoi(ui.green->text().toStdString().c_str()));
            led_msg.b.push_back(atoi(ui.blue->text().toStdString().c_str()));
            led_msg.white.push_back(atoi(ui.white->text().toStdString().c_str()));
        }

        if(ui.checkBox->isChecked()){
            led_msg.ledID.push_back(34);
            led_msg.r.push_back(atoi(ui.red->text().toStdString().c_str()));
            led_msg.g.push_back(atoi(ui.green->text().toStdString().c_str()));
            led_msg.b.push_back(atoi(ui.blue->text().toStdString().c_str()));
            led_msg.white.push_back(atoi(ui.white->text().toStdString().c_str()));
        }

        if(ui.checkBox->isChecked()){
            led_msg.ledID.push_back(35);
            led_msg.r.push_back(atoi(ui.red->text().toStdString().c_str()));
            led_msg.g.push_back(atoi(ui.green->text().toStdString().c_str()));
            led_msg.b.push_back(atoi(ui.blue->text().toStdString().c_str()));
            led_msg.white.push_back(atoi(ui.white->text().toStdString().c_str()));
        }
        msg.leds = led_msg;
        ledPattern_pub.publish(msg);
    }
    void MainWindow::microPhoneControl(){

    }
    void MainWindow::microPhoneData(){

    }

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
//    QSettings settings("Qt-Ros Package", "interface");
//    restoreGeometry(settings.value("geometry").toByteArray());
//    restoreState(settings.value("windowState").toByteArray());
//    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
//    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
//    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
//    ui.line_edit_master->setText(master_url);
//    ui.line_edit_host->setText(host_url);
//    //ui.line_edit_topic->setText(topic_name);
//    bool remember = settings.value("remember_settings", false).toBool();
//    ui.checkbox_remember_settings->setChecked(remember);
//    bool checked = settings.value("use_environment_variables", false).toBool();
//    ui.checkbox_use_environment->setChecked(checked);
//    if ( checked ) {
//    	ui.line_edit_master->setEnabled(false);
//    	ui.line_edit_host->setEnabled(false);
//    	//ui.line_edit_topic->setEnabled(false);
//    }
}

void MainWindow::WriteSettings() {
//    QSettings settings("Qt-Ros Package", "interface");
//    settings.setValue("master_url",ui.line_edit_master->text());
//    settings.setValue("host_url",ui.line_edit_host->text());
//    //settings.setValue("topic_name",ui.line_edit_topic->text());
//    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
//    settings.setValue("geometry", saveGeometry());
//    settings.setValue("windowState", saveState());
//    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace interface


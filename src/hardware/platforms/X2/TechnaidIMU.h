/**
 *
 * \file TechnaidIMU.h
 * \author Technaid and Emek Baris Kucuktabak
 * \version 0.2
 * \date 2021-01-28
 * \copyright Copyright (c) 2021
 *
 * @brief A example class that implment methods and data structs for communicate with Tech IMU
 *
 */

#ifndef SRC_IMUV4_H
#define SRC_IMUV4_H

#define BROADCAST_ID 0x3F
#define START_DIGITAL_DATA_CAPTURE 'D'
#define START_PHYSICAL_DATA_CAPTURE 'P'
#define START_DCM_ORIENTATION_CAPTURE 'O'
#define START_DCM_ORTIENTATION_PHYSICAL_DATA_CAPTURE 'U'
#define START_QUATERNION_DATA_CAPTURE 'A'
#define START_QUATERNION_PHY_DATA_CAPTURE 'B'
#define START_START_ACCELEROMETER_GYR_PHY_DATA_CAPTURE 'I'
#define START_ACCELEROMETER_PHYSICAL_DATA_CAPTURE 'E'
#define START_GYR_PHY_DATA_CAPTURE 'G'
#define START_MAGNETOMETER_PHYSICAL_DATA_CAPTURE 'H'
#define START_QUATERNION_ACCELEROMETER_GYR_PHYSICAL_DATA_CAPTURE 'L'
#define START_QUATERNION_ACCELEROMETER_PHY_DATA_CAPTURE 'J'
#define START_QUATERNION_GYR_PHYSICAL_DATA_CAPTURE 'K'
#define STOP_DATA_CAPTURE 's'
#define CHECK_COMMUNICATION 'c'
#define IMU_VERSION 'v'

// basics
#include <stdio.h>
#include <string.h>
#include <csignal>

// Eigen
#include <Eigen/Dense>

// Logger
#include "spdlog/helper/LogHelper.h"

// CAN
#include <fcntl.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

// Input Device
#include "InputDevice.h"

static volatile sig_atomic_t exitSignalReceived = 0;

struct IMUParameters {
    bool useIMU;
    std::string canChannel;
    std::vector<int> serialNo;
    std::vector<int> networkId;
    std::vector<std::string> outputMode;
    std::vector<int> dataSize;

};

class TechnaidIMU : public InputDevice {
public:
    /*!
     * Constructor.
     * @param nodeHandle the ROS node handle.
     */
    TechnaidIMU(IMUParameters imuParameters);

    bool initialize();
    void updateInput();
    void exit();
    Eigen::MatrixXd& getAcceleration();

private:
    int numberOfIMUs_;

    std::string canChannel_;
    int canSocket_;
    struct can_frame canFrame_;

    bool isInitialized_;

    // vectors to keep parameters of each imu. Read from yaml file
    std::vector<int> serialNo_, networkId_, dataSize_;
    std::vector<std::string> outputMode_;

    Eigen::MatrixXd acceleration_; // rows are the x y z axes of acceleration measurements, columns are different sensors

    bool setOutputMode(int networkId, unsigned char mode);
    bool validateParameters();
    bool canConfiguration();
    bool checkCommunication();

    static void signalHandler(int signum);
};


#endif //SRC_IMUV4_H

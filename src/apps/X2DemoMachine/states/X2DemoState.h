/**
 * /file X2DemoState.h
 * /author Emek Baris Kucuktabak
 * /brief Concrete implementation of DemoState
 * /version 0.1
 * /date 2020-07-06
 *
 * @copyright Copyright (c) 2020
 *
 */
#ifndef SRC_X2DEMOSTATE_H
#define SRC_X2DEMOSTATE_H

#define IMC_REF_ORDER 3
#define IMC_DIST_ORDER 3
#define IMC_DELAY_LENGTH 400

#define ACC_ERROR_ARRAY_SIZE 400

#include "State.h"
#include "X2Robot.h"
#include <ctime>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <math.h>

#include "X2DemoMachineROS.h"

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <CORC/dynamic_paramsConfig.h>

/**
 * \brief Demo State for the X2DemoMachine
 *
 *
 */
class X2DemoState : public State {
    X2Robot *robot_;
    X2DemoMachineROS *x2DemoMachineRos_;

public:
    void entry(void);
    void during(void);
    void exit(void);
    X2DemoState(StateMachine *m, X2Robot *exo, X2DemoMachineROS *x2DemoMachineRos, const char *name = NULL);

    Eigen::VectorXd& getDesiredJointTorques();
    int controller_mode_;
    double virtualMassRatio_;
    double desiredInteractionForce_;
    double desiredJointAcceleration_;
    Eigen::VectorXd desiredJointVelocities_;
    Eigen::VectorXd desiredJointTorques_;
private:
    std::chrono::steady_clock::time_point time0;

    dynamic_reconfigure::Server<CORC::dynamic_paramsConfig> server_;
    void dynReconfCallback(CORC::dynamic_paramsConfig &config, uint32_t level);



    double admittanceInputHistory_[2] = {0,0};
    double admittanceOutputHistory_[2] = {0,0};
    double t_step_ = 0.0025; // todo: get from main

    double mAdmittance_ = 5;
    double bAdmittance_ = 2;

public:
    double kp_acc_;
    double ki_acc_;
    double A_pos_;
    double f_pos_;
    double accErrorArray_[ACC_ERROR_ARRAY_SIZE];

public:
    // variables for imc controller
    double Td_;
    double lambda_r_;
    double lambda_d_;
    double desiredJointAccelerationHistory_[IMC_REF_ORDER];
    double filteredDesiredJointAccelerationHistory_[IMC_REF_ORDER];
    double disturbanceHistory_[IMC_DIST_ORDER];
    double filteredDisturbanceHistory_[IMC_DIST_ORDER];
    double correctedDesiredJointAccelerationHistory_[IMC_DELAY_LENGTH];
    double estimatedJointAcceleration_;

    double checkInput_;
    double checkOutput_;

    double k_interaction_;
    double shiftPos_;


};

#endif
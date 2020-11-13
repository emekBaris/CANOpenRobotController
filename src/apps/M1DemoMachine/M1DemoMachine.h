/**
 * \file M1DemoMachine.h
 * \author Yue Wen adapted from Vincent Crocher
 * \version 0.1
 * \date 2020-08-25
 * \copyright Copyright (c) 2020
 *
 * /brief The <code>M1DemoMachine</code> class represents an example implementation of an M1 state machine.
 *
 */
#ifndef M1_SM_H
#define M1_SM_H

#include <sys/time.h>

#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <csignal> //For raise()

#include "RobotM1.h"
#include "StateMachine.h"

// State Classes
#include "states/M1DemoStates.h"
#include "states/MultiControllerState.h"
#include "spdlog/helper/LogHelper.h"
#include "logging.h"

#include "M1DemoMachineROS.h"

/**
 * @brief Example implementation of a StateMachine for the M1Robot class. States should implemented M1DemoState
 *
 */
class M1DemoMachine : public StateMachine {
   public:
    bool running = false;
    /**
     *  \todo Pilot Parameters would be set in constructor here
     *
     */
    M1DemoMachine();
    ~M1DemoMachine();
    void init(int argc, char *argv[]);
    void end();

    void hwStateUpdate();

    /**
     * Pointers to the relevant states - initialised in init
     *
     */
    MultiControllerState *multiControllerState_;
//    IdleState *idleState;
//    M1DemoState *demoState;
//    Monitoring *monitorState;
//    M1PositionTracking *positionTracking;

   protected:
    RobotM1 *robot_; /*<!Pointer to the Robot*/
    M1DemoMachineROS *m1DemoMachineRos_; /*<!Pointer to the ROS Class*/

   private:
    std::chrono::steady_clock::time_point time0_; // initial time that machine started
    double time_; // time passed after tim0 in [s]
//    EventObject(Event2Demo) * event2Demo;
//    EventObject(Event2Monitor) * event2Monitor;
//    EventObject(Event2Idle) * event2Idle;
//    EventObject(Event2Pos) * event2Pos;
//    EventObject(EndCalib) * endCalib;
};

#endif /*M1_SM_H*/
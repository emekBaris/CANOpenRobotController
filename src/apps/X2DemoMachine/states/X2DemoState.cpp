#include "X2DemoState.h"

X2DemoState::X2DemoState(StateMachine *m, X2Robot *exo, X2DemoMachineROS *x2DemoMachineRos, const char *name) :
        State(m, name), robot_(exo), x2DemoMachineRos_(x2DemoMachineRos) {
    desiredJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    desiredJointVelocities_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    kp_acc_ = 0;
    ki_acc_ = 0;

    // initializing control variable histories to 0
    for(int i = 0; i<IMC_REF_ORDER-1; i++){
        desiredJointAccelerationHistory_[i] = 0.0;
        filteredDesiredJointAccelerationHistory_[i] = 0.0;
    }
    for(int i = 0; i<IMC_REF_ORDER-1;i++){
        disturbanceHistory_[i] = 0.0;
        filteredDisturbanceHistory_[i] = 0.0;
    }
    for(int i = 0; i<IMC_DELAY_LENGTH-1; i++){
        correctedDesiredJointAccelerationHistory_[i] = 0.0;
    }
    for(int i = 0; i<ACC_ERROR_ARRAY_SIZE-1; i++){
        accErrorArray_[i] = 0.0;
    }

}

void X2DemoState::entry(void) {
    std::cout << "Example State Entered " << std::endl
              << "===================" << std::endl
              << "===================" << std::endl;
    robot_->updateRobot();
    // set dynamic parameter server
    dynamic_reconfigure::Server<CORC::dynamic_paramsConfig>::CallbackType f;
    f = boost::bind(&X2DemoState::dynReconfCallback, this, _1, _2);
    server_.setCallback(f);

    robot_->calibrateForceSensors();

    robot_->homingWithImu();

//    robot_->setBackpackIMUMode(IMUOutputMode::QUATERNION);
    robot_->setContactIMUMode(IMUOutputMode::ACCELERATION);

//    std::vector<int> homingDirection{ -1, 1, 0, 0 };
//    robot_->homing(homingDirection);

    time0 = std::chrono::steady_clock::now();
    robot_->initTorqueControl();

}

void X2DemoState::during(void) {
    if(robot_->getBackPackAngleOnMedianPlane() - robot_->getPosition()[0] + robot_->getPosition()[1] >deg2rad(100.0)
    || robot_->getBackPackAngleOnMedianPlane() - robot_->getPosition()[0] + robot_->getPosition()[1] <= deg2rad(-40.0)){

        spdlog::error("EMERGENCY ACTIVATED!!!!!!");
        robot_->initVelocityControl();
        sleep(0.2);
        robot_->initTorqueControl();
        controller_mode_ = 99; //emergency
    }

    if(controller_mode_ == 1){ // zero torque mode
        desiredJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
        robot_->setTorque(desiredJointTorques_);
//        std::cout<<robot_->getInteractionForce()[1]<<std::endl<<"**********"<<std::endl;

    } else if(controller_mode_ == 2){ // zero velocity mode
        desiredJointVelocities_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
        double A = 0.0*M_PI/180.0; //60
        double T = 2.0;
        double time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time0).count()/1000.0;
        desiredJointVelocities_[1] = 0.0;
        robot_->setVelocity(desiredJointVelocities_);

    } else if(controller_mode_ == 3){ // feedforward model compensation
        int motionIntend;
        if(robot_->getPosition()[1]>M_PI/4.0) motionIntend = -1;
        else motionIntend = 1;

        desiredJointTorques_ = robot_->getFeedForwardTorque(motionIntend);
//        std::cout<<desiredJointTorques_<<std::endl;
//        std::cout<<robot_->getInteractionForce()[1]<<std::endl;
        robot_->setTorque(desiredJointTorques_);

    } else if(controller_mode_ == 4){ // virtual mass controller

        Eigen::VectorXd feedBackTorque = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
        double J = 0.1; // distance between knee joint and force sensor

        int motionIntend;
        if(robot_->getPosition()[1]>M_PI/4.0) motionIntend = -1;
        else motionIntend = 1;

//        desiredInteractionForce_ = x2DemoMachineRos_->interactionForceCommand_[1]; //todo: uncomment in muliti robot control

        desiredJointAcceleration_ = (1/(robot_->getRobotParameters().m[1]*virtualMassRatio_))*J*(robot_->getInteractionForce()[1] - desiredInteractionForce_);
        robot_->desiredJointAcceleration_ = desiredJointAcceleration_;

//        feedBackTorque[1] = (1.0/virtualMassRatio_-1)*J*robot_->getInteractionForce()[1] -
//                            (1.0/virtualMassRatio_)*J*desiredInteractionForce_;

        feedBackTorque[1] = robot_->getRobotParameters().m[1]*desiredJointAcceleration_ - J*robot_->getInteractionForce()[1];
        desiredJointTorques_ = robot_->getFeedForwardTorque(motionIntend) + feedBackTorque;
        robot_->setTorque(desiredJointTorques_);

        std::cout<<"robot: "<< robot_->getRobotName()<<std::endl;
        std::cout<<"desired: "<<desiredInteractionForce_<<std::endl;
        std::cout<<"force: "<<robot_->getInteractionForce()[1]<<std::endl;
        std::cout<<"ff: "<<robot_->getFeedForwardTorque(motionIntend)[1]<<std::endl;
        std::cout<<"fb: "<<feedBackTorque[1]<<std::endl;
        std::cout<<"total: "<<desiredJointTorques_[1]<<std::endl;
        std::cout<<"***************"<<std::endl;
    } else if(controller_mode_ == 5){ // Admittance control

        double b0, b1, a1;
        admittanceInputHistory_[0] = robot_->getInteractionForce()[1];

        b0 = t_step_/(mAdmittance_*2.0+bAdmittance_*t_step_);
        b1 = t_step_/(mAdmittance_*2.0+bAdmittance_*t_step_);

        a1 = (mAdmittance_*-4.0)/(mAdmittance_*2.0+bAdmittance_*t_step_)+1.0;

        admittanceOutputHistory_[0] = + b0*admittanceInputHistory_[0] + b1*admittanceInputHistory_[1] - a1*admittanceOutputHistory_[1];

        for(int k =  1 ; k > 0 ; k--) {
            admittanceInputHistory_[k] = admittanceInputHistory_[k-1];
        }

        for(int k =  1 ; k > 0 ; k--) {
            admittanceOutputHistory_[k] = admittanceOutputHistory_[k-1];
        }

        desiredJointVelocities_ << 0, admittanceOutputHistory_[0], 0, 0;

        robot_->setVelocity(desiredJointVelocities_);
//        std::cout<<"Force: "<<robot_->getInteractionForce()[1]<<std::endl;
//        std::cout << "Output vel: " << desiredJointVelocities_[1] * 180.0 / M_PI << std::endl << "****************" << std::endl;

    } else if(controller_mode_ == 6){ // Parameter estimation with velocity control

        double desiredVel = 5.0*M_PI/180.0;

        if(robot_->getPosition()[1]<M_PI/2.0) {
            desiredJointVelocities_ << 0, desiredVel, 0, 0;
        }else {
            desiredJointVelocities_ << 0, 0, 0, 0;
        }

        robot_->setVelocity(desiredJointVelocities_);
//        std::cout<<"Force: "<<robot_->getInteractionForce()[1]<<std::endl;
//        std::cout << "Output vel: " << desiredJointVelocities_[1] * 180.0 / M_PI << std::endl << "****************" << std::endl;

    } else if(controller_mode_ == 7){ // Chirp torque

        double T=15; //chirp time in seconds
        double a=6.; //Amplitude in N.m
        double fi=0; //initial frequency
        double fn=5; //final frequency

        double time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time0).count()/1000.0;
        double f = 0;
        if(time<T) {
            double f = fi + (fn-fi)*time/T;
            desiredJointTorques_[1] = a*sin(2.*M_PI*f*time);
        }
        else {
            desiredJointTorques_[1] = 0;
            std::cout<<"done"<<std::endl;
            controller_mode_ = 0;
        }
        robot_->setTorque(desiredJointTorques_);
    } else if(controller_mode_ == 8){ // IMU monitor mode

        desiredJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
        robot_->setTorque(desiredJointTorques_);
        std::cout<<"backpack: "<< rad2deg(robot_->getBackPackAngleOnMedianPlane())<<std::endl;
        std::cout<<"contact: "<< rad2deg(robot_->getBackPackAngleOnMedianPlane() - robot_->getPosition()[0] + robot_->getPosition()[1])<<std::endl;
//        std::cout<<"Joint 0: "<< rad2deg(robot_->getPosition()[0])<<std::endl;
//        std::cout<<"Joint 1: "<< rad2deg(robot_->getPosition()[1])<<std::endl;
//        std::cout<<"Joint 2: "<< rad2deg(robot_->getPosition()[2])<<std::endl;
//        std::cout<<"Joint 3: "<< rad2deg(robot_->getPosition()[3])<<std::endl;
//        std::cout<<180.0/M_PI*(robot_->getBackPackAngleOnMedianPlane() - robot_->getPosition()[0] + robot_->getPosition()[1])<<std::endl<<"*****"<<std::endl;
//        std::cout<<"**************"<<std::endl;

    } else if(controller_mode_ == 9){ // chirp velocity

        double T = 10; //chirp time in seconds
        double a = deg2rad(45); //Amplitude in rad/s
        double fi = 0.5; //initial frequency
        double fn = 0.5; //final frequency

        double time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time0).count()/1000.0;
        double f = 0;
        if(time<T) {
            double f = fi + (fn-fi)*time/T;
            desiredJointVelocities_ << 0, a*sin(2.*M_PI*f*time), 0, 0;
        }
        else {
            desiredJointVelocities_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
            std::cout<<"done"<<std::endl;
            controller_mode_ = 0;
        }
        robot_->setVelocity(desiredJointVelocities_);
    } else if(controller_mode_ == 10){ // chirp acc

        desiredJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
        double T = 10; //chirp time in seconds
        double a = 1; //Amplitude in rad/s^2
        double fi = 0.5; //initial frequency
        double fn = 0.5; //final frequency

        double time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time0).count()/1000.0;
        double f = 0;
        double motionIntend;
        if(time<T) {
            double f = fi + (fn-fi)*time/T;
            desiredJointAcceleration_ = a*sin(2.*M_PI*f*time);
            motionIntend = desiredJointAcceleration_ / abs(desiredJointAcceleration_);
            desiredJointTorques_[1] = robot_->getRobotParameters().m[1]*desiredJointAcceleration_ + robot_->getFeedForwardTorque(motionIntend)[1];
        }
        else {
            desiredJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
            std::cout<<"done"<<std::endl;
            controller_mode_ = 0;
        }

        robot_->setTorque(desiredJointTorques_);
    } else if(controller_mode_ == 11){ // acceleration control
        // x: input
        // y: output

        double A = 2;
        double T = 1;
        double J = 0.1; // distance between knee joint and force sensor
        double time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time0).count()/1000.0;

        desiredJointAcceleration_ = A*sin(2*M_PI/T*time);
//        desiredJointAcceleration_ = 10.0;
//        desiredJointAcceleration_ = (1/(robot_->getRobotParameters().m[1]*virtualMassRatio_))*J*(robot_->getInteractionForce()[1] - desiredInteractionForce_);
        robot_->desiredJointAcceleration_ = desiredJointAcceleration_;

        desiredJointAccelerationHistory_[0] = robot_->desiredJointAcceleration_;
        double b0_r, b1_r, b2_r, a1_r, a2_r;
        double b0_d, b1_d, b2_d, a1_d, a2_d;

        b0_r = (Td_*2.0+t_step_)*1.0/pow(lambda_r_*2.0+t_step_,2.0)*(lambda_r_*4.0+t_step_);
        b1_r = -1.0/pow(lambda_r_*2.0+t_step_,2.0)*(Td_*lambda_r_*1.6E+1-(t_step_*t_step_)*2.0);
        b2_r = 1.0/pow(lambda_r_*2.0+t_step_,2.0)*(Td_*2.0-t_step_)*(lambda_r_*4.0-t_step_);

        a1_r = (t_step_*4.0)/(lambda_r_*2.0+t_step_)-2.0;
        a2_r = 1.0/pow(lambda_r_*2.0+t_step_,2.0)*pow(lambda_r_*2.0-t_step_,2.0);

//        std::cout<<"b0_r: "<<b0_r<<std::endl;
//        std::cout<<"b1_r: "<<b1_r<<std::endl;
//        std::cout<<"b2_r: "<<b2_r<<std::endl;
//        std::cout<<"a1_r: "<<a1_r<<std::endl;
//        std::cout<<"a2_r: "<<a2_r<<std::endl;
//        std::cout<<"***"<<std::endl;

        filteredDesiredJointAccelerationHistory_[0] = + b0_r*desiredJointAccelerationHistory_[0] +
                b1_r*desiredJointAccelerationHistory_[1] + b2_r*desiredJointAccelerationHistory_[2]
                - a1_r*filteredDesiredJointAccelerationHistory_[1] - a2_r*filteredDesiredJointAccelerationHistory_[2];

//        std::cout<<"filteredDesiredJointAccelerationHistory_[0]: "<<filteredDesiredJointAccelerationHistory_[0]<<std::endl;
//        std::cout<<"filteredDesiredJointAccelerationHistory_[1]: "<<filteredDesiredJointAccelerationHistory_[1]<<std::endl;
//        std::cout<<"filteredDesiredJointAccelerationHistory_[2]: "<<filteredDesiredJointAccelerationHistory_[2]<<std::endl;
//        std::cout<<"desiredJointAccelerationHistory_[0]: "<<desiredJointAccelerationHistory_[0]<<std::endl;
//        std::cout<<"desiredJointAccelerationHistory_[1]: "<<desiredJointAccelerationHistory_[1]<<std::endl;
//        std::cout<<"desiredJointAccelerationHistory_[2]: "<<desiredJointAccelerationHistory_[2]<<std::endl;
//        std::cout<<"---------------------"<<std::endl;
//
//        std::cout<<"filteredDesiredJointAccelerationHistory_: "<<filteredDesiredJointAccelerationHistory_[0]<<std::endl;
//        std::cout<<"filteredDisturbanceHistory_: "<<filteredDisturbanceHistory_[0]<<std::endl;

        correctedDesiredJointAccelerationHistory_[0] = filteredDesiredJointAccelerationHistory_[0] - filteredDisturbanceHistory_[0];
        std::cout<<"corrected acc: "<<correctedDesiredJointAccelerationHistory_[0]<<std::endl;
        estimatedJointAcceleration_ = correctedDesiredJointAccelerationHistory_[(int)(Td_/t_step_)];

        disturbanceHistory_[0] = robot_->getFilteredCorrectedContactAccelerationsZ_()/IMU_DISTANCE - estimatedJointAcceleration_; // UNCOMMENT

        b0_d = (Td_*2.0+t_step_)*1.0/pow(lambda_d_*2.0+t_step_,2.0)*(lambda_d_*4.0+t_step_);
        b1_d = -1.0/pow(lambda_d_*2.0+t_step_,2.0)*(Td_*lambda_d_*1.6E+1-(t_step_*t_step_)*2.0);
        b2_d = 1.0/pow(lambda_d_*2.0+t_step_,2.0)*(Td_*2.0-t_step_)*(lambda_d_*4.0-t_step_);

        a1_d = (t_step_*4.0)/(lambda_d_*2.0+t_step_)-2.0;
        a2_d = 1.0/pow(lambda_d_*2.0+t_step_,2.0)*pow(lambda_d_*2.0-t_step_,2.0);

        filteredDisturbanceHistory_[0] = + b0_d*disturbanceHistory_[0] + b1_d*disturbanceHistory_[1] + b2_d*disturbanceHistory_[2]
                -a1_d*filteredDisturbanceHistory_[1] - a2_d*filteredDisturbanceHistory_[2];

        robot_->correctedDesiredJointAcceleration_ = correctedDesiredJointAccelerationHistory_[0];

        int motionIntend;
        if(robot_->getPosition()[1]>M_PI/4.0) motionIntend = -1;
        else motionIntend = 1;
//        if(desiredJointAcceleration_>0) motionIntend = -1;
//        else motionIntend = 1;

        desiredJointTorques_[1] = robot_->getRobotParameters().m[1]*correctedDesiredJointAccelerationHistory_[0] - 0*J*robot_->getInteractionForce()[1]
                + robot_->getFeedForwardTorque(motionIntend)[1];

        robot_->setTorque(desiredJointTorques_);

        std::cout<<"force: "<<robot_->getInteractionForce()[1]<<std::endl;
        std::cout<<"ref acc: "<<desiredJointAccelerationHistory_[0]<<std::endl;
        std::cout<<"corrected acc: "<<correctedDesiredJointAccelerationHistory_[0]<<std::endl;
//        std::cout<<"fb: "<<robot_->getRobotParameters().m[1]*correctedDesiredJointAccelerationHistory_[0]- 0*J*robot_->getInteractionForce()[1]<<std::endl;
//        std::cout<<"ff: "<<robot_->getFeedForwardTorque(motionIntend)[1]<<std::endl;
        std::cout<<"tau: "<<desiredJointTorques_[1]<<std::endl;
        std::cout<<"***************************"<<std::endl;

        for(int k =  IMC_REF_ORDER-1 ; k > 0 ; k--) {
            desiredJointAccelerationHistory_[k] = desiredJointAccelerationHistory_[k-1];
            filteredDesiredJointAccelerationHistory_[k] = filteredDesiredJointAccelerationHistory_[k-1];
        }
        for(int k =  IMC_DIST_ORDER-1 ; k > 0 ; k--) {
            disturbanceHistory_[k] = disturbanceHistory_[k-1];
            filteredDisturbanceHistory_[k] = filteredDisturbanceHistory_[k-1];
        }
        for(int k = IMC_DELAY_LENGTH-1; k > 0; k--){
            correctedDesiredJointAccelerationHistory_[k] = correctedDesiredJointAccelerationHistory_[k-1];
        }


//        std::cout<<"AFTER filteredDesiredJointAccelerationHistory_[0]: "<<filteredDesiredJointAccelerationHistory_[0]<<std::endl;
//        std::cout<<"AFTER filteredDesiredJointAccelerationHistory_[1]: "<<filteredDesiredJointAccelerationHistory_[1]<<std::endl;
//        std::cout<<"AFTER filteredDesiredJointAccelerationHistory_[2]: "<<filteredDesiredJointAccelerationHistory_[2]<<std::endl;
//        std::cout<<"AFTER desiredJointAccelerationHistory_[0]: "<<desiredJointAccelerationHistory_[0]<<std::endl;
//        std::cout<<"AFTER desiredJointAccelerationHistory_[1]: "<<desiredJointAccelerationHistory_[1]<<std::endl;
//        std::cout<<"AFTER desiredJointAccelerationHistory_[2]: "<<desiredJointAccelerationHistory_[2]<<std::endl;
//        std::cout<<"******************************"<<std::endl;

    } else if (controller_mode_ == 12){ // acc control _v2 (simpler)

        double J = 0.1; // distance between knee joint and force sensor
        double time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time0).count()/1000.0;

//        desiredJointAcceleration_ = A_acc_*sin(2*M_PI*f_acc_*time);
//        desiredJointAcceleration_ = 10.0;
        desiredJointAcceleration_ = (1/(robot_->getRobotParameters().m[1]*virtualMassRatio_))*J*(robot_->getInteractionForce()[1] - desiredInteractionForce_);
        robot_->desiredJointAcceleration_ = desiredJointAcceleration_;

//        double error = desiredJointAcceleration_ - robot_->getFilteredCorrectedContactAccelerationsZ_()/IMU_DISTANCE;
        double error = desiredJointAcceleration_ - robot_->mergedAcc_/IMU_DISTANCE;
        accErrorArray_[0] = error;

        double accErrorSum = 0;
        for (int i = 0; i< ACC_ERROR_ARRAY_SIZE; i++){
            accErrorSum += accErrorArray_[i]*t_step_;
        }

        robot_->correctedDesiredJointAcceleration_ = desiredJointAcceleration_ + kp_acc_*error + ki_acc_*accErrorSum;

        int motionIntend;
//        if(robot_->getPosition()[1]>M_PI/4.0) motionIntend = -1;
//        else motionIntend = 1;
        if(desiredJointAcceleration_>0) motionIntend = -1;
        else motionIntend = 1;

        desiredJointTorques_[1] = robot_->getRobotParameters().m[1]*robot_->correctedDesiredJointAcceleration_ - 0*J*robot_->getInteractionForce()[1]
                                  + robot_->getFeedForwardTorque(motionIntend)[1];

        robot_->setTorque(desiredJointTorques_);

        for(int k = ACC_ERROR_ARRAY_SIZE - 1; k > 0; k--){
            accErrorArray_[k] = accErrorArray_[k-1];
        }



        std::cout<<"force: "<<robot_->getInteractionForce()[1]<<std::endl;
        std::cout<<"ref acc: "<<desiredJointAcceleration_<<std::endl;
        std::cout<<"corrected acc: "<<robot_->correctedDesiredJointAcceleration_<<std::endl;
//        std::cout<<"fb: "<<robot_->getRobotParameters().m[1]*correctedDesiredJointAccelerationHistory_[0]- 0*J*robot_->getInteractionForce()[1]<<std::endl;
//        std::cout<<"ff: "<<robot_->getFeedForwardTorque(motionIntend)[1]<<std::endl;
        std::cout<<"tau: "<<desiredJointTorques_[1]<<std::endl;
        std::cout<<"***************************"<<std::endl;

    }




    else if(controller_mode_ == 99){

        desiredJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
        robot_->setTorque(desiredJointTorques_);

    }
}

void X2DemoState::exit(void) {
    robot_->initTorqueControl();
    // setting 0 torque for safety. Not required for X2(2018) but for some reason, in X2(2019), after exit() it takes around 2-3 second to drives to tunr off.
    robot_->setTorque(Eigen::VectorXd::Zero(X2_NUM_JOINTS));
    std::cout << "Example State Exited" << std::endl;
}

Eigen::VectorXd &X2DemoState::getDesiredJointTorques() {
    return desiredJointTorques_;
}

//todo: move to DemoMachine
void X2DemoState::dynReconfCallback(CORC::dynamic_paramsConfig &config, uint32_t level) {

    if(controller_mode_ == 99){
        spdlog::error("EMERGENCY. CANT CHANGE CONTROLLER. RESTART THE PROGRAM");
        return; // if emergency doesnt let change stuff
    }

    if(config.controller_mode == 1) robot_->initTorqueControl();
    if(config.controller_mode == 2 && controller_mode_ !=2 ) {
        robot_->initVelocityControl();
        time0 = std::chrono::steady_clock::now();
    }
    if(config.controller_mode == 3) robot_->initTorqueControl();
    if(config.controller_mode == 4) robot_->initTorqueControl();
    if(config.controller_mode == 5) robot_->initVelocityControl();
    if(config.controller_mode == 6) robot_->initVelocityControl();
    if(config.controller_mode == 7) {
        robot_->initTorqueControl();
        time0 = std::chrono::steady_clock::now();
    }
    if(config.controller_mode == 8) robot_->initTorqueControl();
    if(config.controller_mode == 9) {
        robot_->initVelocityControl();
        time0 = std::chrono::steady_clock::now();
    }
    if(config.controller_mode == 10) {
        robot_->initTorqueControl();
        time0 = std::chrono::steady_clock::now();
    }
    if(config.controller_mode == 11 && controller_mode_ != 11) {
        robot_->initTorqueControl();
        time0 = std::chrono::steady_clock::now();
    }

    controller_mode_ = config.controller_mode;
    virtualMassRatio_ = config.virtual_mass_ratio;
    desiredInteractionForce_ = config.desired_interaction_force;
    mAdmittance_ = config.m_admittance;
    bAdmittance_ = config.b_admittance;
    Td_ = config.Td/1000.0; // convert to [sec]
    lambda_r_ = config.lambda_r;
    lambda_d_ = config.lambda_d;
    kp_acc_ = config.kp_acc;
    ki_acc_ = config.ki_acc;
    A_acc_ = config.A_acc;
    f_acc_ = config.f_acc;
    robot_->accCutoffFreq = config.acc_cutoff_freq;
    robot_->accBiasCutoffFreq = config.acc_bias_cutoff_freq;


    return;

}
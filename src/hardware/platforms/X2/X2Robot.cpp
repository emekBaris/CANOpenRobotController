#include "X2Robot.h"

/**
 * An enum type.
 * Joint Index for the 4 joints (note, CANopen NODEID = this + 1)
 */
enum X2Joints {
    X2_LEFT_HIP = 0,   /**< Left Hip*/
    X2_LEFT_KNEE = 1,  /**< Left Knee*/
    X2_RIGHT_HIP = 2,  /**< Right Hip*/
    X2_RIGHT_KNEE = 3, /**< Right Knee*/
};
/**
 * Paramater definitions: Hip motor reading and corresponding angle. Used for mapping between degree and motor values.
 */
JointDrivePairs hipJDP{
    250880,       // drivePosA
    0,            // drivePosB
    deg2rad(90),  //jointPosA
    deg2rad(0)    //jointPosB
};
/**
 * Paramater definitions: Knee motor reading and corresponding angle. Used for mapping between degree and motor values.
 */
JointDrivePairs kneeJDP{
    250880,       // drivePosA
    0,            //drivePosB
    deg2rad(90),  //jointPosA
    deg2rad(0)    //jointPosB
};

/**
 * Defines the Joint Limits of the X2 Exoskeleton
 *
 */
ExoJointLimits X2JointLimits = {deg2rad(120), deg2rad(-30), deg2rad(120), deg2rad(0)};

static volatile sig_atomic_t exitHoming = 0;

X2Robot::X2Robot() : Robot() {

    // This is the default name accessed from the MACRO. If ROS is used, under demo machine robot name can be set
    // by setRobotName() to the ros node name. See X2DemoMachine::init()
    robotName_ = XSTR(X2_NAME);

    interactionForces_ = Eigen::VectorXd::Zero(forceSensors.size());

    // Initializing the parameters to zero
    x2Parameters.m = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    x2Parameters.l = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    x2Parameters.s = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    x2Parameters.I = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    x2Parameters.c0 = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    x2Parameters.c1 = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    x2Parameters.c2 = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    x2Parameters.cuffWeights = Eigen::VectorXd::Zero(X2_NUM_FORCE_SENSORS);
    x2Parameters.forceSensorScaleFactor = Eigen::VectorXd::Zero(X2_NUM_FORCE_SENSORS);

    #ifdef NOROBOT
        simJointPositions_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
        simJointVelocities_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
        simJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    #endif
}

X2Robot::~X2Robot() {

    technaidIMUs->exit();
    freeMemory();
    spdlog::debug("X2Robot deleted");
}

void X2Robot::signalHandler(int signum) {
    exitHoming = 1;
    std::raise(SIGTERM); //Clean exit
}

#ifdef SIM
void X2Robot::initialiseROS() {
    controllerSwitchClient_ = nodeHandle_->serviceClient<controller_manager_msgs::SwitchController>("controller_manager/switch_controller");

    positionCommandPublisher_ = nodeHandle_->advertise<std_msgs::Float64MultiArray>("position_controller/command", 10);
    velocityCommandPublisher_ = nodeHandle_->advertise<std_msgs::Float64MultiArray>("velocity_controller/command", 10);
    torqueCommandPublisher_ = nodeHandle_->advertise<std_msgs::Float64MultiArray>("torque_controller/command", 10);

    jointStateSubscriber_ = nodeHandle_->subscribe("joint_states", 1, &X2Robot::jointStateCallback, this);
}
#endif
bool X2Robot::initPositionControl() {
    spdlog::debug("Initialising Position Control on all joints ");
    bool returnValue = true;
    for (auto p : joints) {
        if (p->setMode(CM_POSITION_CONTROL, posControlMotorProfile) != CM_POSITION_CONTROL) {
            // Something back happened if were are here
            spdlog::error("Something bad happened");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        p->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        p->enable();
    }

#ifdef SIM
    controllerSwitchMsg_.request.start_controllers = {"position_controller"};
    controllerSwitchMsg_.request.stop_controllers = {"velocity_controller", "torque_controller"};
    controllerSwitchMsg_.request.strictness = 1;
    controllerSwitchMsg_.request.start_asap = true;
    controllerSwitchMsg_.request.timeout = 0.0;

    if (controllerSwitchClient_.call(controllerSwitchMsg_)) {
        spdlog::info("Switched to position controller");
    } else {
        spdlog::error("Failed switching to position controller");
        returnValue = false;
    }
#endif

    return returnValue;
}

bool X2Robot::initVelocityControl() {
    spdlog::debug("Initialising Velocity Control on all joints ");
    bool returnValue = true;
    for (auto p : joints) {
        if (p->setMode(CM_VELOCITY_CONTROL, velControlMotorProfile) != CM_VELOCITY_CONTROL) {
            // Something back happened if were are here
            spdlog::error("Something bad happened");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        p->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        p->enable();
    }

#ifdef SIM
    controllerSwitchMsg_.request.start_controllers = {"velocity_controller"};
    controllerSwitchMsg_.request.stop_controllers = {"position_controller", "torque_controller"};
    controllerSwitchMsg_.request.strictness = 1;
    controllerSwitchMsg_.request.start_asap = true;
    controllerSwitchMsg_.request.timeout = 0.0;

    if (controllerSwitchClient_.call(controllerSwitchMsg_)) {
        spdlog::info("Switched to velocity controller");
    } else {
        spdlog::error("Failed switching to velocity controller");
        returnValue = false;
    }
#endif

    return returnValue;
}

bool X2Robot::initTorqueControl() {
    spdlog::debug("Initialising Torque Control on all joints ");
    bool returnValue = true;
    for (auto p : joints) {
        if (p->setMode(CM_TORQUE_CONTROL) != CM_TORQUE_CONTROL) {
            // Something back happened if were are here
            spdlog::error("Something bad happened");
            returnValue = false;
        }
        // Put into ReadyToSwitchOn()
        p->readyToSwitchOn();
    }

    // Pause for a bit to let commands go
    usleep(2000);
    for (auto p : joints) {
        p->enable();
    }

#ifdef SIM
    controllerSwitchMsg_.request.start_controllers = {"torque_controller"};
    controllerSwitchMsg_.request.stop_controllers = {"position_controller", "velocity_controller"};
    controllerSwitchMsg_.request.strictness = 1;
    controllerSwitchMsg_.request.start_asap = true;
    controllerSwitchMsg_.request.timeout = 0.0;

    if (controllerSwitchClient_.call(controllerSwitchMsg_)) {
        spdlog::info("Switched to torque controller");
    } else {
        spdlog::error("Failed switching to torque controller");
        returnValue = false;
    }
#endif

    return returnValue;
}

setMovementReturnCode_t X2Robot::setPosition(Eigen::VectorXd positions) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;
    for (auto p : joints) {
        setMovementReturnCode_t setPosCode = ((X2Joint *)p)->setPosition(positions[i]);
        if (setPosCode == INCORRECT_MODE) {
            spdlog::error("Joint {} is not in Position Control ", p->getId());
            returnValue = INCORRECT_MODE;
        } else if (setPosCode != SUCCESS) {
            // Something bad happened
            spdlog::error("Joint {} Unknown Error", p->getId());
            returnValue = UNKNOWN_ERROR;
        }
        i++;
    }

#ifdef SIM
    std::vector<double> positionVector(X2_NUM_JOINTS);

    for (int i = 0; i < X2_NUM_JOINTS; i++) {
        positionVector[i] = positions[i];
    }

    positionCommandMsg_.data = positionVector;
    positionCommandPublisher_.publish(positionCommandMsg_);
#elif NOROBOT
    simJointPositions_ = positions;
#endif

    return returnValue;
}

setMovementReturnCode_t X2Robot::setVelocity(Eigen::VectorXd velocities) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;
    for (auto p : joints) {
        setMovementReturnCode_t setPosCode = ((X2Joint *)p)->setVelocity(velocities[i]);
        if (setPosCode == INCORRECT_MODE) {
            spdlog::error("Joint {} is not in Velocity Control", p->getId());
            returnValue = INCORRECT_MODE;
        } else if (setPosCode != SUCCESS) {
            // Something bad happened
            spdlog::error("Joint {} Unknown Error", p->getId());
            returnValue = UNKNOWN_ERROR;
        }
        i++;
    }

#ifdef SIM
    std::vector<double> velocityVector(X2_NUM_JOINTS);

    for (int i = 0; i < X2_NUM_JOINTS; i++) {
        velocityVector[i] = velocities[i];
    }

    velocityCommandMsg_.data = velocityVector;
    velocityCommandPublisher_.publish(velocityCommandMsg_);
#endif

    return returnValue;
}

setMovementReturnCode_t X2Robot::setTorque(Eigen::VectorXd torques) {
    int i = 0;
    setMovementReturnCode_t returnValue = SUCCESS;
    for (auto p : joints) {
        setMovementReturnCode_t setPosCode = ((X2Joint *)p)->setTorque(torques[i]);
        if (setPosCode == INCORRECT_MODE) {
            spdlog::error("Joint {} is not in Torque Control", p->getId());
            returnValue = INCORRECT_MODE;
        } else if (setPosCode != SUCCESS) {
            // Something bad happened
            spdlog::error("Joint {} Unknown Error", p->getId());
            returnValue = UNKNOWN_ERROR;
        }
        i++;
    }

#ifdef SIM
    std::vector<double> torqueVector(X2_NUM_JOINTS);

    for (int i = 0; i < X2_NUM_JOINTS; i++) {
        torqueVector[i] = torques[i];
    }

    torqueCommandMsg_.data = torqueVector;
    torqueCommandPublisher_.publish(torqueCommandMsg_);
#endif

    return returnValue;
}

Eigen::VectorXd &X2Robot::getPosition() {
#ifndef NOROBOT
    return Robot::getPosition();
#else
    return simJointPositions_;
#endif
}

Eigen::VectorXd &X2Robot::getVelocity() {
#ifndef NOROBOT
    return Robot::getVelocity();
#else
    return simJointVelocities_;
#endif
}

Eigen::VectorXd &X2Robot::getTorque() {
#ifndef NOROBOT
    return Robot::getTorque();
#else
    return simJointTorques_;
#endif
}

Eigen::VectorXd &X2Robot::getInteractionForce() {
    //TODO: generalise sensors
    //Initialise vector if not already done
    if((unsigned int)interactionForces_.size()!=forceSensors.size()) {
        interactionForces_ = Eigen::VectorXd::Zero(forceSensors.size());
    }

    //todo: add backpack angle
    Eigen::VectorXd cuffCompensation = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    cuffCompensation[0] = x2Parameters.cuffWeights[0] * sin(getPosition()[0]);
    cuffCompensation[1] = x2Parameters.cuffWeights[1] * sin(getPosition()[1] - getPosition()[0]);
    cuffCompensation[2] = x2Parameters.cuffWeights[2] * sin(getPosition()[2]);
    cuffCompensation[3] = x2Parameters.cuffWeights[3] * sin(getPosition()[3] - getPosition()[2]);

    //Update values
    for (int i = 0; i < X2_NUM_FORCE_SENSORS; i++) {
        interactionForces_[i] = forceSensors[i]->getForce() + cuffCompensation[i];
    }
    return interactionForces_;
}

bool X2Robot::calibrateForceSensors() {
    int numberOfSuccess = 0;
    for (int i = 0; i < X2_NUM_FORCE_SENSORS; i++) {
        if (forceSensors[i]->calibrate()) numberOfSuccess++;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    if (numberOfSuccess == X2_NUM_FORCE_SENSORS) {
        spdlog::info("[X2Robot::calibrateForceSensors]: Zeroing of force sensors are successfully completed.");
        return true;
    } else {
        spdlog::error("[X2Robot::calibrateForceSensors]: Zeroing failed.");
        return false;
    }
}

Eigen::MatrixXd & X2Robot::getContactAccelerations() {

    // todo: use backpack angle for compensation
    int contactIndex = 0;
    for(int imuIndex = 0; imuIndex<technaidIMUs->getNumberOfIMUs_(); imuIndex++){
        if(x2Parameters.imuParameters.location[imuIndex] == 'c'){

            if(technaidIMUs->getOutputMode_(imuIndex).name != "acc"){
                spdlog::warn("Contact IMU mode is not acceleration for imu no: . Returns 0", imuIndex);
                contactAccelerations_.col(contactIndex) = Eigen::MatrixXd::Zero(3,1);
            }
            contactAccelerations_.col(contactIndex) = technaidIMUs->getAcceleration().col(imuIndex);
        }
    }
    return contactAccelerations_;
}

Eigen::MatrixXd & X2Robot::getContactQuaternions() {

    // todo: use backpack angle for compensation
    int contactIndex = 0;
    for(int imuIndex = 0; imuIndex<technaidIMUs->getNumberOfIMUs_(); imuIndex++){
        if(x2Parameters.imuParameters.location[imuIndex] == 'c'){

            if(technaidIMUs->getOutputMode_(imuIndex).name != "quat"){
                spdlog::warn("Contact IMU mode is not quaternion for imu no: . Returns 0", imuIndex);
                contactQuaternions_.col(contactIndex) = Eigen::MatrixXd::Zero(4,1);
            }
            contactQuaternions_.col(contactIndex) = technaidIMUs->getQuaternion().col(imuIndex);
        }
    }
    return contactQuaternions_;
}

Eigen::MatrixXd & X2Robot::getBackpackAccelerations() {

    // todo: use backpack angle for compensation
    int backpackIndex = 0;
    for(int imuIndex = 0; imuIndex<technaidIMUs->getNumberOfIMUs_(); imuIndex++){
        if(x2Parameters.imuParameters.location[imuIndex] == 'b'){

            if(technaidIMUs->getOutputMode_(imuIndex).name != "acc"){
                spdlog::warn("Backpack IMU mode is not acceleration for imu no: . Returns 0", imuIndex);
                backpackAccelerations_.col(backpackIndex) = Eigen::MatrixXd::Zero(3,1);
            }
            backpackAccelerations_.col(backpackIndex) = technaidIMUs->getAcceleration().col(imuIndex);
        }
    }
    return backpackAccelerations_;

}

Eigen::MatrixXd & X2Robot::getBackpackQuaternions() {
    // todo: use backpack angle for compensation
    int backpackIndex = 0;
    for(int imuIndex = 0; imuIndex<technaidIMUs->getNumberOfIMUs_(); imuIndex++){
        if(x2Parameters.imuParameters.location[imuIndex] == 'b'){
            if(technaidIMUs->getOutputMode_(imuIndex).name != "quat"){
                spdlog::warn("Backpack IMU mode is not quaternion for imu no: . Returns 0", imuIndex);
                backpackQuaternions_.col(backpackIndex) = Eigen::MatrixXd::Zero(3,1);
            }
            backpackQuaternions_.col(backpackIndex) = technaidIMUs->getQuaternion().col(imuIndex);
        }
    }
    return backpackQuaternions_;
}

RobotParameters & X2Robot::getRobotParameters() {

    return x2Parameters;
}

bool X2Robot::homing(std::vector<int> homingDirection, float thresholdTorque, float delayTime,
                     float homingSpeed, float maxTime) {
    std::vector<bool> success(X2_NUM_JOINTS, false);
    std::chrono::steady_clock::time_point time0;
    this->initVelocityControl();
    signal(SIGINT, signalHandler); // check if ctrl + c is pressed

    for (int i = 0; i < X2_NUM_JOINTS; i++) {
        if (homingDirection[i] == 0) continue;  // skip the joint if it is not asked to do homing

        Eigen::VectorXd desiredVelocity(X2_NUM_JOINTS);
        std::chrono::steady_clock::time_point firstTimeHighTorque;  // time at the first time joint exceed thresholdTorque
        bool highTorqueReached = false;

        desiredVelocity[i] = homingSpeed * homingDirection[i] / std::abs(homingDirection[i]);  // setting the desired velocity by using the direction
        time0 = std::chrono::steady_clock::now();

        spdlog::debug("Homing Joint {} ...", i);

        while (success[i] == false &&
                exitHoming == 0 &&
               std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time0).count() < maxTime * 1000) {
            this->updateRobot();  // because this function has its own loops, updateRobot needs to be called
            this->setVelocity(desiredVelocity);
            if (std::abs(this->getTorque()[i]) >= thresholdTorque) {  // if high torque is reached
                highTorqueReached = true;
                firstTimeHighTorque = std::chrono::steady_clock::now();
                while (std::chrono::duration_cast<std::chrono::milliseconds>  // high torque should be measured for delayTime
                       (std::chrono::steady_clock::now() - firstTimeHighTorque).count() < delayTime * 1000 &&
                        exitHoming == 0) {
                    this->updateRobot();
                    if (std::abs(this->getTorque()[i]) < thresholdTorque) {  // if torque value reach below thresholdTorque, goes back
                        highTorqueReached = false;
                        break;
                    }
                }
            }
            success[i] = highTorqueReached;
        }

        if (success[i]) {
            spdlog::debug("Homing Succeeded for Joint {} .", i);
            if (i == X2_LEFT_HIP || i == X2_RIGHT_HIP) {  // if it is a hip joint

                // zeroing is done depending on the limits on the homing direction
                if (homingDirection[i] > 0)
                    ((X2Joint *)this->joints[i])->setPositionOffset(X2JointLimits.hipMax);
                else
                    ((X2Joint *)this->joints[i])->setPositionOffset(X2JointLimits.hipMin);
            } else if (i == X2_LEFT_KNEE || i == X2_RIGHT_KNEE) {  // if it is a knee joint

                // zeroing is done depending on the limits on the homing direction
                if (homingDirection[i] > 0)
                    ((X2Joint *)this->joints[i])->setPositionOffset(X2JointLimits.kneeMax);
                else
                    ((X2Joint *)this->joints[i])->setPositionOffset(X2JointLimits.kneeMin);
            }

        } else {
            spdlog::error("Homing Failed for Joint {} .", i);
        }
    }
    // Checking if all commanded joint successfully homed
    for (int i = 0; i < X2_NUM_JOINTS; i++) {
        if (homingDirection[i] == 0) continue;  // skip the joint if it is not asked to do homing
        if (success[i] == false) return false;
    }
    return true;  // will come here if all joints successfully homed
}

bool X2Robot::initialiseJoints() {

    initializeRobotParams(robotName_);

    for (int id = 0; id < X2_NUM_JOINTS; id++) {
        motorDrives.push_back(new CopleyDrive(id + 1));
        // The X2 has 2 Hips and 2 Knees, by default configured as 2 hips, then 2 legs int jointID, double jointMin, double jointMax, JointDrivePairs jdp, Drive *drive
        if (id == X2_LEFT_HIP || id == X2_RIGHT_HIP) {
            joints.push_back(new X2Joint(id, X2JointLimits.hipMin, X2JointLimits.hipMax, hipJDP, motorDrives[id]));
        } else if (id == X2_LEFT_KNEE || id == X2_RIGHT_KNEE) {
            joints.push_back(new X2Joint(id, X2JointLimits.kneeMin, X2JointLimits.kneeMax, kneeJDP, motorDrives[id]));
        }
    }

    return true;
}

bool X2Robot::initialiseNetwork() {
    spdlog::debug("X2Robot::initialiseNetwork()");

    bool status;
    for (auto joint : joints) {
        status = joint->initNetwork();
        if (!status)
            return false;
    }

#ifdef SIM
    initialiseROS();
#endif

    return true;
}
bool X2Robot::initialiseInputs() {
    inputs.push_back(keyboard = new Keyboard());

    for (int id = 0; id < X2_NUM_FORCE_SENSORS; id++) {
        forceSensors.push_back(new X2ForceSensor(id, x2Parameters.forceSensorScaleFactor[id]));
        inputs.push_back(forceSensors[id]);
    }

    if(x2Parameters.imuParameters.useIMU){
        technaidIMUs = new TechnaidIMU(x2Parameters.imuParameters);
//        inputs.push_back(technaidIMUs); // commented because techIMU class starts its own thread for data update
        technaidIMUs->initialize();
    }

    return true;
}

bool X2Robot::initializeRobotParams(std::string robotName) {

    // need to use address of base directory because when run with ROS, working directory is ~/.ros
    std::string baseDirectory = XSTR(BASE_DIRECTORY);
    std::string relativeFilePath = "/config/x2_params.yaml";

    YAML::Node params = YAML::LoadFile(baseDirectory + relativeFilePath);

    // if the robotName does not match with the name in x2_params.yaml
    if(!params[robotName]){
        spdlog::error("Parameters of {} couldn't be found in {} !", robotName, baseDirectory + relativeFilePath);
        spdlog::error("All parameters are zero !");

        return false;
    }

    // getting the parameters from the yaml file
    for(int i = 0; i<X2_NUM_JOINTS; i++){
        x2Parameters.m[i] = params[robotName]["m"][i].as<double>();
        x2Parameters.l[i] = params[robotName]["l"][i].as<double>();
        x2Parameters.s[i] = params[robotName]["s"][i].as<double>();
        x2Parameters.I[i] = params[robotName]["I"][i].as<double>();
        x2Parameters.c0[i] = params[robotName]["c0"][i].as<double>();
        x2Parameters.c1[i] = params[robotName]["c1"][i].as<double>();
        x2Parameters.c2[i] = params[robotName]["c2"][i].as<double>();
    }

    for(int i = 0; i<X2_NUM_FORCE_SENSORS; i++) {
        x2Parameters.cuffWeights[i] = params[robotName]["cuff_weights"][i].as<double>();
        x2Parameters.forceSensorScaleFactor[i] = params[robotName]["force_sensor_scale_factor"][i].as<double>();

    }

    if(!params[robotName]["technaid_imu"]){
        spdlog::warn("IMU parameters couldn't be found!");
        x2Parameters.imuParameters.useIMU = false;
    } else{
        x2Parameters.imuParameters.useIMU = true;
        x2Parameters.imuParameters.canChannel = params[robotName]["technaid_imu"]["can_channel"].as<std::string>();
        for(int i = 0; i<X2_NUM_IMUS; i++) {
            x2Parameters.imuParameters.serialNo.push_back(params[robotName]["technaid_imu"]["serial_no"][i].as<int>());
            x2Parameters.imuParameters.networkId.push_back(params[robotName]["technaid_imu"]["network_id"][i].as<int>());
            x2Parameters.imuParameters.location.push_back(params[robotName]["technaid_imu"]["location"][i].as<char>());
        }
    }

    int numberOfContactIMUs = 0;
    int numberOfBackpackIMUs = 0;
    for(int i = 0; i< X2_NUM_IMUS; i++){
        if(x2Parameters.imuParameters.location[i] == 'c') numberOfContactIMUs++;
        else if(x2Parameters.imuParameters.location[i] == 'b') numberOfBackpackIMUs++;
    }
    contactAccelerations_ = Eigen::MatrixXd::Zero(3, numberOfContactIMUs);
    contactQuaternions_ = Eigen::MatrixXd::Zero(4, numberOfContactIMUs);
    backpackAccelerations_ = Eigen::MatrixXd::Zero(3, numberOfBackpackIMUs);
    backpackQuaternions_ = Eigen::MatrixXd::Zero(4, numberOfBackpackIMUs);

    return true;
}

void X2Robot::freeMemory() {
    for (auto p : joints) {
        spdlog::debug("Delete Joint ID: {}", p->getId());
        delete p;
    }
    for (auto p : motorDrives) {
        spdlog::debug("Delete Drive Node: {}", p->getNodeID());
        delete p;
    }
    for (auto p : inputs) {
        spdlog::debug("Deleting Input");
        delete p;
    }
}
void X2Robot::updateRobot() {
    //TODO: generalise sensors update
    Robot::updateRobot();
}

Eigen::VectorXd X2Robot::getFeedForwardTorque(int motionIntend) {
    float coulombFriction;
    const float velTreshold = 1*M_PI/180.0; // [rad/s]

    // todo generalized 4 Dof Approach
    if(abs(jointVelocities_[1]) > velTreshold){ // if in motion
        coulombFriction = x2Parameters.c1[1]*jointVelocities_[1]/abs(jointVelocities_[1]) +
        + x2Parameters.c2[1]*sqrt(abs(jointVelocities_[1]))*jointVelocities_[1]/abs(jointVelocities_[1]);
    }else { // if static
        coulombFriction = x2Parameters.c1[1]*motionIntend/abs(motionIntend);
    }

    Eigen::VectorXd ffTorque = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
    ffTorque[1] = x2Parameters.m[1]*x2Parameters.s[1]*9.81*sin(jointPositions_[1] - jointPositions_[0]) + coulombFriction + x2Parameters.c0[1]*jointVelocities_[1];

    return ffTorque;

}

void X2Robot::setRobotName(std::string robotName) {
    robotName_ = robotName;
}

bool X2Robot::setContactIMUMode(IMUOutputMode imuOutputMode) {

    for(int i = 0; i<technaidIMUs->getNumberOfIMUs_(); i++){
        if(x2Parameters.imuParameters.location[i] == 'c'){
            if(!technaidIMUs->setOutputMode(i, imuOutputMode)){
                return false;
            }
        }
    }
}

bool X2Robot::setBackpackIMUMode(IMUOutputMode imuOutputMode) {

    for(int i = 0; i<technaidIMUs->getNumberOfIMUs_(); i++){
        if(x2Parameters.imuParameters.location[i] == 'b'){
            if(!technaidIMUs->setOutputMode(i, imuOutputMode)){
            return false;
            }
        }
    }
}

std::string & X2Robot::getRobotName() {
    return robotName_;
}

#ifdef SIM
void X2Robot::setNodeHandle(ros::NodeHandle &nodeHandle) {
    nodeHandle_ = &nodeHandle;
}

void X2Robot::jointStateCallback(const sensor_msgs::JointState &msg) {
    for (int i = 0; i < X2_NUM_JOINTS; i++) {
        simJointPositions_[i] = msg.position[i];
        simJointVelocities_[i] = msg.velocity[i];
        simJointTorques_[i] = msg.effort[i];
    }
}

#endif

#include "X2DemoMachine.h"

#define OWNER ((X2DemoMachine *)owner)

X2DemoMachine::X2DemoMachine() {
    robot_ = new X2Robot();
    // Create PRE-DESIGNED State Machine events and state objects.
    startExo = new StartExo(this);
    /**f
     * \brief add a tranisition object to the arch list of the first state in the NewTransition MACRO.
     * Effectively creating a statemachine transition from State A to B in the event of event c.
     * NewTranstion(State A,Event c, State B)
     *
     */

    // Create ros object
    x2DemoMachineRos_ = new X2DemoMachineROS(robot_);
}

/**
 * \brief start function for running any designed statemachine specific functions
 * for example initialising robot objects.
 *
 */
void X2DemoMachine::init(int argc, char *argv[]) {
    spdlog::debug("X2DemoMachine::init()");

    ros::init(argc, argv, "x2", ros::init_options::NoSigintHandler);
    ros::NodeHandle nodeHandle("~");

    // Pass nodeHandle to the classes that use ROS features
    x2DemoMachineRos_->setNodeHandle(nodeHandle);

    // Pass MachineRos to the State to use ROS features
    x2DemoState = new X2DemoState(this, robot_, x2DemoMachineRos_);

    // Create states with ROS features // This should be created after ros::init()
    StateMachine::initialize(x2DemoState);

    // Get robot name from the node name
    robotName_ = ros::this_node::getName();
    robotName_.erase(0,1); // erase the first character which is '/'

#ifdef SIM
    robot_->setNodeHandle(nodeHandle);
#endif

    robot_->setRobotName(robotName_);
    initialised = robot_->initialise();
    x2DemoMachineRos_->initialize();
    running = true;
    time0 = std::chrono::steady_clock::now();

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::stringstream logFileName;

    logFileName << "spdlogs/" << robotName_<< std::put_time(&tm, "/%d-%m-%Y_%H-%M-%S") << ".csv";

    logHelper.initLogger("test_logger", logFileName.str(), LogFormat::CSV, true);
    logHelper.add(time, "time");
    logHelper.add(x2DemoState->controller_mode_, "mode");
    logHelper.add(robot_->getPosition(), "JointPositions");
    logHelper.add(robot_->getVelocity(), "JointVelocities");
    logHelper.add(robot_->getTorque(), "JointTorques");
    logHelper.add(x2DemoState->getDesiredJointTorques(), "DesiredJointTorques");
    logHelper.add(x2DemoState->desiredJointVelocities_, "DesiredJointVelocities");
    logHelper.add(x2DemoState->desiredJointAcceleration_, "DesiredJointAcceleration");
    logHelper.add(robot_->correctedDesiredJointAcceleration_, "CorrectedDesiredJointAcceleration");
    logHelper.add(robot_->getInteractionForce(), "InteractionForces");
    logHelper.add(robot_->desiredInteractionForce, "DesiredInteractionForce");
    logHelper.add(robot_->targetJointPos, "targetJointPos");
    logHelper.add(robot_->getCorrectedContactAccelerationsZ_(), "ContactAcceleration");
    logHelper.add(robot_->mergedAcc_, "MergedContactAcceleration");
    logHelper.add(robot_->getFilteredCorrectedContactAccelerationsZ_(), "FilteredContactAcceleration");
    logHelper.add(robot_->getBackPackAngleOnMedianPlane(), "BackpackAngle");
    logHelper.add(x2DemoState->virtualMassRatio_, "virtualMassRatio");
    logHelper.add(x2DemoState->k_interaction_, "stiffness");
    logHelper.add(x2DemoState->f_pos_, "frequency");
    logHelper.add(x2DemoState->checkInput_, "input");
    logHelper.add(x2DemoState->checkOutput_, "output");

    logHelper.startLogger();
}

void X2DemoMachine::end() {
    if(initialised) {
        logHelper.endLog();
        currentState->exit();
        robot_->disable();
        delete x2DemoMachineRos_;
        delete robot_;
    }
}

////////////////////////////////////////////////////////////////
// Events ------------------------------------------------------
///////////////////////////////////////////////////////////////
bool X2DemoMachine::StartExo::check(void) {
    if (OWNER->robot_->keyboard->getS() == true || OWNER->x2DemoMachineRos_->startExoTriggered_) {
        spdlog::info("Exo started!");
        OWNER->x2DemoMachineRos_->startExoTriggered_ = false;
        return true;
    }
    return false;
}
/**
 * \brief Statemachine to hardware interface method. Run any hardware update methods
 * that need to run every program loop update cycle.
 *
 */
void X2DemoMachine::hwStateUpdate(void) {
    robot_->updateRobot();
}

void X2DemoMachine::update() {
    time = (std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - time0).count()) / 1e6;

    StateMachine::update();
    x2DemoMachineRos_->update();
    ros::spinOnce();
}

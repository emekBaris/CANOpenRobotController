#include "TechnaidIMU.h"

TechnaidIMU::TechnaidIMU(IMUParameters imuParameters)
    : canChannel_(imuParameters.canChannel),
      serialNo_(imuParameters.serialNo),
      networkId_(imuParameters.networkId),
      outputMode_(imuParameters.outputMode),
      dataSize_(imuParameters.dataSize)
{
    isInitialized_ = false;
}

bool TechnaidIMU::initialize() {
    // Read parameters
    if(!validateParameters()){
        spdlog::error("[TechnaidIMU::initialize]: Parameters can't be read successfully!");
        return false;
    }

    // CAN configuration
    if(!canConfiguration()){
        spdlog::error("[TechnaidIMU::initialize]: Error in CAN configuration");
        return false;
    }

    // Communication check
    if(!checkCommunication()){
        spdlog::error("[TechnaidIMU::initialize]: Error in IMU communication");
        return false;
    }

    // Set output mode of the IMUs
    for(int i = 0; i<numberOfIMUs_; i++){
        if(!setOutputMode(networkId_[i], outputMode_[i][0])){
            spdlog::error("[TechnaidIMU::initialize]: Error in setting the output mode of imu with net id {}. ", networkId_[i]);
            return false;
        }
    }

    sleep(5);

    //todo:: add start thread
    startUpdateThread();

    spdlog::info("[TechnaidIMU::initialize]: Successfully initialized");
    isInitialized_ = true;
    return true;
}

bool TechnaidIMU::setOutputMode(int networkId, unsigned char mode) {

    canFrame_.can_id = networkId;
    canFrame_.can_dlc = 1;
    canFrame_.data[0] = mode;
    int writeByte = write(canSocket_, &canFrame_, sizeof(struct can_frame));

    sleep(0.5);

    bool success = (writeByte == sizeof(struct can_frame));

    if(success) {
        spdlog::info("[TechnaidIMU::setOutputMode]: Mode is changed to {} for imu with net id: {}",mode, networkId);
        return true;
    } else {
        spdlog::error("[TechnaidIMU::setOutputMode]: Error while changing the mode to {} for imu with net id: {} !", mode, networkId);
        return false;
    }
}

void* TechnaidIMU::update(void) {

    std::chrono::steady_clock::time_point time0;
    while(!exitSignalReceived) {
        if (!isInitialized_) return NULL; // if not initialized, immediately return

        // Pooling
        canFrame_.can_id = BROADCAST_ID;
        canFrame_.can_dlc = 0;
        write(canSocket_, &canFrame_, sizeof(struct can_frame));

        int maxSize = 60;
        //specify the amount of data to read for each imu
        char data_bytes_multi[maxSize][numberOfIMUs_];
        int count[numberOfIMUs_]; // count of the number of bytes that have been read for each imu
        for (int i = 0; i < numberOfIMUs_; i++) count[i] = 0; // setting the elements to zero

        struct timeval timeout = {0, 10000}; // 10 ms timout
        fd_set readSet;
        FD_ZERO(&readSet);
        FD_SET(canSocket_, &readSet);

        bool exit = false;
        while (!exit && select((canSocket_ + 1), &readSet, NULL, NULL, &timeout) && !exitSignalReceived) {
            // stays in the loop until all data is received or timeout value is reached
            read(canSocket_, &canFrame_, sizeof(struct can_frame));

            exit = true;
            for (int i = 0; i < numberOfIMUs_; i++) {
                if (canFrame_.can_id == networkId_[i] + 16) {
                    for (int j = 0; j < canFrame_.can_dlc; j++) {
                        data_bytes_multi[count[i]][i] = canFrame_.data[j];
                        count[i]++;
                    }
                }
                exit = exit && (count[i] == dataSize_[i]); // if data from all sensors are received, this becomes true
            }
        }

        for (int i = 0; i < numberOfIMUs_; i++) {
            if (count[i] == dataSize_[i]) {

                // get the i th column
                char data_bytes[dataSize_[i]];
                for (int j = 0; j < dataSize_[i]; j++) {
                    data_bytes[j] = data_bytes_multi[j][i];
                }

                float *data_float;
                data_float = (float *) data_bytes;

                acceleration_(0, i) = data_float[0]; // acc x
                acceleration_(1, i) = data_float[1]; // acc y
                acceleration_(2, i) = data_float[2]; // acc z

            } else {
                spdlog::warn("[TechnaidIMU::update()]: Data was not successfully read for IMU no: {}!", serialNo_[i]);
            }
        }
    }
    double time_ms = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time0).count()/1000.0;
//    std::cout<<time_ms<<std::endl;
}

bool TechnaidIMU::validateParameters() {

    numberOfIMUs_ = serialNo_.size();
    if(networkId_.size()!=numberOfIMUs_ || outputMode_.size()!=numberOfIMUs_ || dataSize_.size()!=numberOfIMUs_){
        spdlog::error("[TechnaidIMU::readParameters()]: Size of the parameters are not same!");
        return false;
    }

    acceleration_ = Eigen::MatrixXd::Zero(3, numberOfIMUs_);

    return true;
}

bool TechnaidIMU::canConfiguration() {

    // CAN initialization
    canSocket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    struct sockaddr_can addr;
    struct ifreq ifr;
    strcpy(ifr.ifr_name, canChannel_.c_str());
    int ioControl = ioctl(canSocket_, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(canSocket_, (struct sockaddr *)&addr, sizeof(addr));

    return (ioControl >= 0);
}

bool TechnaidIMU::checkCommunication() {

    // send check communication command
    canFrame_.can_id = BROADCAST_ID;
    canFrame_.can_dlc = 1;
    canFrame_.data[0] = CHECK_COMMUNICATION;
    write(canSocket_, &canFrame_, sizeof(struct can_frame));

    sleep(0.1);

    struct timeval timeout = {1, 0};
    fd_set readSet;
    FD_ZERO(&readSet);
    FD_SET(canSocket_, &readSet);

    std::vector<int> receivedIds;

    // check the ids of the IMUs on the CAN network
    while(select((canSocket_ + 1), &readSet, NULL, NULL, &timeout) && !exitSignalReceived) {
        // stays in the loop as long as there is something to read
        std::cout<<" IN READ !!!"<<std::endl;
        read(canSocket_, &canFrame_, sizeof(struct can_frame));
        std::cout<<" ID: "<<canFrame_.can_id<<std::endl;
        receivedIds.push_back(canFrame_.can_id);
    }

    // compare the number of IMUs in the network with the size of the parameters
    if(receivedIds.size() != numberOfIMUs_){
        spdlog::error("[TechnaidIMU::checkCommunication()]: {} sets of parameters are given but there are {} IMUs on {}!",
                        numberOfIMUs_, receivedIds.size(), canChannel_);
        return false;
    }

    for(int id: receivedIds){
        if(std::count(networkId_.begin(), networkId_.end(), id-16) == 0){
            spdlog::error("[TechnaidIMU::checkCommunication()]: parameters of network id: {} could not be found!", (id-16));
            return false;
        } else if (std::count(networkId_.begin(), networkId_.end(), id-16) > 1){
            spdlog::error("[TechnaidIMU::checkCommunication()]: parameters of network id: {} is given more than once!", (id-16));
            return false;
        }
    }

    return true;
}

void * TechnaidIMU::updateHelper(void *This) {

    ((TechnaidIMU *)This)->update();
    return NULL;

}

bool TechnaidIMU::startUpdateThread() {

    return (pthread_create(&updateThread, NULL, &TechnaidIMU::updateHelper, this) == 0);
}

void TechnaidIMU::exit() {

    for(int i = 0; i<numberOfIMUs_; i++){
        canFrame_.can_id = networkId_[i];
        canFrame_.can_dlc = 1;
        canFrame_.data[0] = STOP_DATA_CAPTURE;
        write(canSocket_, &canFrame_, sizeof(struct can_frame));

        spdlog::info("Data capture ended on IMU with serial no {}.", serialNo_[i]);
    }

    close(canSocket_);
}

Eigen::MatrixXd& TechnaidIMU::getAcceleration() {

    return acceleration_;
}

void TechnaidIMU::signalHandler(int signum) {

    exitSignalReceived = 1;
    std::raise(SIGTERM); // clean exit

}

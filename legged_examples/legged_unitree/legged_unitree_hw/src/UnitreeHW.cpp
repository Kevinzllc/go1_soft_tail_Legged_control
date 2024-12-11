//
// Created by qiayuan on 1/24/22.
//

#include "legged_unitree_hw/UnitreeHW.h"
#include "free_dog_sdk/unitreeConnect.hpp"
#include "free_dog_sdk/lowState.hpp"
#include "free_dog_sdk/lowCmd.hpp"
#include "free_dog_sdk/highState.hpp"
#include "free_dog_sdk/highCmd.hpp"

#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16MultiArray.h>

namespace legged {

bool UnitreeHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
    if (!LeggedHW::init(root_nh, robot_hw_nh)) {
        return false;
    }

    robot_hw_nh.getParam("power_limit", powerLimit_);

    setupJoints();
    setupImu();
    setupContactSensor(robot_hw_nh);

    connection_ = std::make_shared<FDSC::UnitreeConnection>("LOW_WIRED_DEFAULTS");
    connection_->startRecv();

    std::string robot_type;
    root_nh.getParam("robot_type", robot_type);

    if (robot_type == "go1") {  
        safety_ = std::make_shared<FDSC::Safety>(FDSC::DogType::YourType);
    } else {
        ROS_FATAL("Unknown robot type: %s", robot_type.c_str());
        return false;
    }

    joyPublisher_ = root_nh.advertise<sensor_msgs::Joy>("/joy", 10);
    contactPublisher_ = root_nh.advertise<std_msgs::Int16MultiArray>("/contact", 10);
    return true;
}

void UnitreeHW::read(const ros::Time& time, const ros::Duration& /*period*/) {
    auto data = connection_->getData();

    for (int i = 0; i < 12; ++i) {
        jointData_[i].pos_ = data[i].position;
        jointData_[i].vel_ = data[i].velocity;
        jointData_[i].tau_ = data[i].torque;
    }

    imuData_.ori_ = { data.imu.orientation[0], data.imu.orientation[1], data.imu.orientation[2], data.imu.orientation[3] };
    imuData_.angularVel_ = { data.imu.angularVelocity[0], data.imu.angularVelocity[1], data.imu.angularVelocity[2] };
    imuData_.linearAcc_ = { data.imu.linearAcceleration[0], data.imu.linearAcceleration[1], data.imu.linearAcceleration[2] };

    for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
        contactState_[i] = data.contactStates[i] > contactThreshold_;
    }

    updateJoystick(time);
    updateContact(time);
}

void UnitreeHW::write(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
    std::vector<uint8_t> cmdData;

    for (int i = 0; i < 12; ++i) {
        lowCmd_.motorCmd[i].q = static_cast<float>(jointData_[i].posDes_);
        lowCmd_.motorCmd[i].dq = static_cast<float>(jointData_[i].velDes_);
        lowCmd_.motorCmd[i].Kp = static_cast<float>(jointData_[i].kp_);
        lowCmd_.motorCmd[i].Kd = static_cast<float>(jointData_[i].kd_);
        lowCmd_.motorCmd[i].tau = static_cast<float>(jointData_[i].ff_);
    }

    safety_->PositionLimit(lowCmd_);
    safety_->PowerProtect(lowCmd_, lowState_, powerLimit_);
    connection_->send(lowCmd_.buildCmd());
}

bool UnitreeHW::setupJoints() {
    for (const auto& joint : urdfModel_->joints_) {
        int leg_index = getLegIndex(joint.first);
        int joint_index = getJointIndex(joint.first);
        if (leg_index < 0 || joint_index < 0) continue;

        int index = leg_index * 3 + joint_index;
        hardware_interface::JointStateHandle state_handle(joint.first, &jointData_[index].pos_, &jointData_[index].vel_, &jointData_[index].tau_);
        jointStateInterface_.registerHandle(state_handle);
        hybridJointInterface_.registerHandle(HybridJointHandle(state_handle, &jointData_[index].posDes_, &jointData_[index].velDes_,
                                                               &jointData_[index].kp_, &jointData_[index].kd_, &jointData_[index].ff_));
    }
    return true;
}

bool UnitreeHW::setupImu() {
    imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle("base_imu", "base_imu", imuData_.ori_, imuData_.oriCov_,
                                                                           imuData_.angularVel_, imuData_.angularVelCov_, imuData_.linearAcc_,
                                                                           imuData_.linearAccCov_));
    imuData_.oriCov_ = {0.0012, 0, 0, 0, 0.0012, 0, 0, 0, 0.0012};
    imuData_.angularVelCov_ = {0.0004, 0, 0, 0, 0.0004, 0, 0, 0, 0.0004};
    imuData_.linearAccCov_ = {0.0004, 0, 0, 0, 0.0004, 0, 0, 0, 0.0004};
    return true;
}

bool UnitreeHW::setupContactSensor(ros::NodeHandle& nh) {
    nh.getParam("contact_threshold", contactThreshold_);
    for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
        contactSensorInterface_.registerHandle(ContactSensorHandle(CONTACT_SENSOR_NAMES[i], &contactState_[i]));
    }
    return true;
}

void UnitreeHW::updateJoystick(const ros::Time& time) {
    if ((time - lastJoyPub_).toSec() < 1 / 50.) {
        return;
    }
    lastJoyPub_ = time;
    xRockerBtnDataStruct keyData;
    memcpy(&keyData, &lowState_.wirelessRemote[0], 40);
    sensor_msgs::Joy joyMsg;  
    joyMsg.axes.push_back(-keyData.lx);
    joyMsg.axes.push_back(keyData.ly);
    joyMsg.axes.push_back(-keyData.rx);
    joyMsg.axes.push_back(keyData.ry);
    joyMsg.buttons.push_back(keyData.btn.components.X);
    joyMsg.buttons.push_back(keyData.btn.components.A);
    joyMsg.buttons.push_back (keyData.btn.components.B);
    joyMsg.buttons.push_back (keyData.btn.components.Y);
    joyMsg.buttons.push_back (keyData.btn.components.L1);
    joyMsg.buttons.push_back (keyData.btn.components.R1);
    joyMsg.buttons.push_back (keyData.btn.components.L2);
    joyMsg.buttons.push_back (keyData.btn.components.R2);
    joyMsg.buttons.push_back (keyData.btn.components.select);
    joyMsg.buttons.push_back (keyData.btn.components.start);
    joyPublisher_.publish(joyMsg);
}

void UnitreeHW::updateContact(const ros::Time& time) {
    if ((time - lastContactPub_).toSec() < 1 / 50.) {
        return;
    }
    lastContactPub_ = time;

    std_msgs::Int16MultiArray contactMsg;
    for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
        contactMsg.data.push_back(lowState_.footForce[i]);
    }
    contactPublisher_.publish(contactMsg);
}

}  // namespace legged



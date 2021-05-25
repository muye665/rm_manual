//
// Created by qiayuan on 5/22/21.
//

#ifndef RM_MANUAL_COMMON_SERVICE_CALLER_H_
#define RM_MANUAL_COMMON_SERVICE_CALLER_H_

#include <chrono>
#include <mutex>
#include <thread>
#include <ros/ros.h>
#include <ros/service.h>
#include <controller_manager_msgs/SwitchController.h>
#include <control_msgs/QueryCalibrationState.h>

namespace rm_manual {
template<class ServiceType>
class ServiceCallerBase {
 public:
  explicit ServiceCallerBase(ros::NodeHandle &nh) {
    if (!nh.getParam("service_name", service_name_))
      ROS_ERROR("Service name no defined (namespace: %s)", nh.getNamespace().c_str());
    client_ = nh.serviceClient<ServiceType>(service_name_);
  }
  ~ServiceCallerBase() { delete thread_; }
  virtual void callService() {
    std::unique_lock<std::mutex> guard(mutex_, std::try_to_lock);
    if (!guard.owns_lock())
      return;
    thread_ = new std::thread(&ServiceCallerBase::callingThread, this);
    thread_->detach();
  }

  ServiceType &getService() { return service_; }
  bool isCalling() {
    std::unique_lock<std::mutex> guard(mutex_, std::try_to_lock);
    return !guard.owns_lock();
  }
 protected:
  virtual void callingThread() {
    std::lock_guard<std::mutex> guard(mutex_);
    if (!client_.call(service_))
      ROS_ERROR("Failed to call service %s on %s", typeid(ServiceType).name(), service_name_.c_str());
  }

  std::string service_name_;
  ros::ServiceClient client_;
  ServiceType service_;
  std::thread *thread_{};
  std::mutex mutex_;
};

class SwitchControllerService : public ServiceCallerBase<controller_manager_msgs::SwitchController> {
 public:
  explicit SwitchControllerService(ros::NodeHandle &nh) :
      ServiceCallerBase<controller_manager_msgs::SwitchController>(nh) {
    XmlRpc::XmlRpcValue controllers;
    if (nh.getParam("start_controllers", controllers))
      for (int i = 0; i < controllers.size(); ++i)
        service_.request.start_controllers.push_back(controllers[i]);
    if (nh.getParam("stop_controllers", controllers))
      for (int i = 0; i < controllers.size(); ++i)
        service_.request.stop_controllers.push_back(controllers[i]);
    if (service_.request.start_controllers.empty() && service_.request.stop_controllers.empty())
      ROS_ERROR("No start/stop controllers specified (namespace: %s)", nh.getNamespace().c_str());
    service_.request.strictness = service_.request.STRICT;
  }
  bool getOk() { return service_.response.ok; }
};

class QueryCalibrationService : public ServiceCallerBase<control_msgs::QueryCalibrationState> {
 public:
  explicit QueryCalibrationService(ros::NodeHandle &nh) : ServiceCallerBase<control_msgs::QueryCalibrationState>(nh) {}
  bool getIsCalibrated() { return service_.response.is_calibrated; }
  void callService() override {
    std::unique_lock<std::mutex> guard(mutex_, std::try_to_lock);
    if (!guard.owns_lock())
      return;
    thread_ = new std::thread(&QueryCalibrationService::callingThread, this);
    thread_->detach();

  }
 protected:
  void callingThread() override {
    std::lock_guard<std::mutex> guard(mutex_);
    while (!service_.response.is_calibrated) {
      if (!client_.call(service_))
        ROS_ERROR("Failed to call service %s on %s",
                  typeid(control_msgs::QueryCalibrationState).name(),
                  service_name_.c_str());
    }
  }
};

}

#endif //RM_MANUAL_COMMON_SERVICE_CALLER_H_

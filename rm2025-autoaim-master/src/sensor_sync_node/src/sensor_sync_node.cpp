#include <chrono>
#include <exception>
#include <functional>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <interfaces/srv/detail/activatesensor__struct.hpp>
#include <interfaces/srv/detail/cameratimestamp__struct.hpp>
#include <memory>
#include <rclcpp/clock.hpp>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/detail/camera_info__struct.hpp>
#include <sensor_sync_node/sensor_sync_node.hpp>
#include <std_msgs/msg/detail/header__struct.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

namespace sensorsync {
SensorSyncNode::SensorSyncNode(const rclcpp::NodeOptions& options)
    : Node("Sensorsync", options), camera_awake_time_(rclcpp::Duration::from_seconds(0.114514)) {
    rclcpp::Clock sys_clock(RCL_SYSTEM_TIME);
    RCLCPP_INFO_STREAM(this->get_logger(), "Start initializing sensor_sync_node ...");
    activate_server_ = this->create_service<interfaces::srv::Activatesensor>(
        "/order2sync", [this](const std::shared_ptr<interfaces::srv::Activatesensor::Request> request,
                              std::shared_ptr<interfaces::srv::Activatesensor::Response> response) {
            ActivateSignalCallback(request, response);
        });

    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
                   .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                   .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST)
                   .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)
                   .keep_last(1);
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image", qos,
        [this](const std::shared_ptr<sensor_msgs::msg::Image> msg) { ImgSyncListenerCallback(msg); });

    camera_info_subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/camera_info", rclcpp::SensorDataQoS(),
        [this](const std::shared_ptr<sensor_msgs::msg::CameraInfo> msg) { CameraInfoListenerCallback(msg); });
    motor_angle_subscription_ = this->create_subscription<interfaces::msg::Angle>(
        "/mcu/angle", rclcpp::SensorDataQoS(),
        [this](const std::shared_ptr<interfaces::msg::Angle> msg) { MotorAngleListenerCallback(msg); });
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/mcu/imu_msg", rclcpp::SensorDataQoS(),
        [this](const std::shared_ptr<sensor_msgs::msg::Imu> msg) { ImuListenerCallback(msg); });
    camera_status_subscription_ = this->create_subscription<std_msgs::msg::Int64>(
        "/camera/waterfall", rclcpp::SensorDataQoS(),
        [this](const std::shared_ptr<std_msgs::msg::Int64> msg) { CameraAwakeCallback(msg); });

    activate_order_ = this->create_publisher<std_msgs::msg::String>("/sensor_manager/activate_order", rclcpp::SensorDataQoS());

    activate_client_ = this->create_client<interfaces::srv::Activatesensor>("/mcu/activate_camera");

    camera_pub_ = image_transport::create_publisher(this, "/sync/image", qos.get_rmw_qos_profile());

    sensor_sync_heart_beat_ = this->create_publisher<std_msgs::msg::String>("/sensor_sync/heartbeat", rclcpp::SensorDataQoS());

    timer_ = this->create_wall_timer(std::chrono::seconds(3), [this]() {
        std_msgs::msg::String s;
        s.data = "sensor_sync_node";
        this->sensor_sync_heart_beat_->publish(s);
    });

    while (!activate_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "waitting is interrupted ...");
            return;
        }
        RCLCPP_DEBUG_STREAM(this->get_logger(), "waitting ...");
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "initialized  sensor_sync_node");
}

void SensorSyncNode::ImgSyncListenerCallback(const std::shared_ptr<sensor_msgs::msg::Image> image_msg) {
    if (status == WORK_STATUS::START || status == WORK_STATUS::WAITING) {
        first_time_activate_camera_stamp_in_camera_ = image_msg->header.stamp;
        //CalculateAwakeTime();
        status = WORK_STATUS::WORKING;
        RCLCPP_INFO_STREAM(this->get_logger(), "Sync start working ");
        auto trigger_loop = [this](){
            while(this->stop_future.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout){
                this->activate_order_->publish(this->kOrder);
                std::this_thread::sleep_for(std::chrono::milliseconds(9));
            }
        };
        activate_thread = std::thread(trigger_loop);
    } else if (status == WORK_STATUS::WORKING) {
        //image_msg->header.stamp = SyncImageTimeStamp(image_msg->header.stamp);
        // RCLCPP_INFO_STREAM(this->get_logger(), "header time is : "<< image_msg->header.stamp.sec <<
        //           "." << image_msg->header.stamp.nanosec);
        camera_pub_.publish(image_msg);
        //RCLCPP_INFO_STREAM(this->get_logger(), "get img");

    } else {  // status == WORK_STATUS::SLEEP
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(),  *this->get_clock(), 1000, "Snyc_node not working");
    }
}

void SensorSyncNode::CameraAwakeCallback(const std::shared_ptr<std_msgs::msg::Int64> msg){
    if(status == SensorSyncNode::WORK_STATUS::SLEEP){
        auto activate_request = std::make_shared<interfaces::srv::Activatesensor::Request>();
        status = SensorSyncNode::WORK_STATUS::START;
        activate_request->activate_sensor = interfaces::srv::Activatesensor::Request::SYNCSENSOR;
        activate_client_->async_send_request(
            activate_request, [this](rclcpp::Client<interfaces::srv::Activatesensor>::SharedFuture future) {
                if (this->status == WORK_STATUS::START) {
                    auto res = future.get();
                    RCLCPP_INFO_STREAM(this->get_logger(), "Start Sync sensor Time is :"<< res->header.stamp.sec << "." <<
                            res->header.stamp.nanosec);
                    if (res->header.stamp.sec == 0)
                        rclcpp::shutdown();
                    this->status = WORK_STATUS::WAITING;

                    first_time_activate_camera_stamp_in_world_ =
                        res->header.stamp; 
                }
                //RCLCPP_DEBUG(this->get_logger(), "");
            });
    }
    if(msg->data < this->count){
        status = SensorSyncNode::WORK_STATUS::SLEEP;
    }
    this->count = msg->data;
}

void SensorSyncNode::ImuListenerCallback(const std::shared_ptr<sensor_msgs::msg::Imu> imu_msg) {
    if (tf_broadcaster_ == nullptr) {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this->shared_from_this());
    }
    geometry_msgs::msg::TransformStamped t;

    t.header.frame_id = "world";
    t.header.stamp = imu_msg->header.stamp;
    t.child_frame_id = "gimbal";

    t.transform.translation.x = 0.0;  // T.at<double>(0, 3);
    t.transform.translation.y = 0.0;  // T.at<double>(1, 3);
    t.transform.translation.z = 0.0;  // T.at<double>(2, 3);
    t.transform.rotation.w = imu_msg->orientation.w;
    t.transform.rotation.x = imu_msg->orientation.x;
    t.transform.rotation.y = imu_msg->orientation.y;
    t.transform.rotation.z = imu_msg->orientation.z;

    tf_broadcaster_->sendTransform(t);
}

void SensorSyncNode::CameraInfoListenerCallback(const std::shared_ptr<sensor_msgs::msg::CameraInfo> camera_info_msg) {}

void SensorSyncNode::MotorAngleListenerCallback(const std::shared_ptr<interfaces::msg::Angle> motor_angle_msg) {
    if (tf_broadcaster_ == nullptr) {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this->shared_from_this());
    }
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "world";
    t.header.stamp = motor_angle_msg->header.stamp;
    t.child_frame_id = "true_gimbal";
    // cv::Mat T = CoordChange();
    tf2::Quaternion q;
    q.setRPY(0, motor_angle_msg->pitch_motor, motor_angle_msg->yaw_motor);
    t.transform.rotation.w = q.w();
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    tf_broadcaster_->sendTransform(t);
}

void SensorSyncNode::ActivateSignalCallback(const std::shared_ptr<interfaces::srv::Activatesensor::Request> request,
                                            std::shared_ptr<interfaces::srv::Activatesensor::Response> response) {
    
    // auto activate_request = std::make_shared<interfaces::srv::Activatesensor::Request>(*request);
    // if (this->status == WORK_STATUS::SLEEP &&
    //     request->activate_sensor == interfaces::srv::Activatesensor::Request::SYNCSENSOR) {
    //     RCLCPP_INFO_STREAM(this->get_logger(), "START sync camera ");
    //     this->status = WORK_STATUS::START;
    // } else {
    //     request->activate_sensor = interfaces::srv::Activatesensor::Request::NONE;
    // }

    // if (this->status == WORK_STATUS::WORKING &&
    //     request->activate_sensor == interfaces::srv::Activatesensor::Request::CONTINUOUSTRIGGER) {
    //     RCLCPP_INFO_STREAM(this->get_logger(), "START trigger camera ");
    // } else {
    //     request->activate_sensor = interfaces::srv::Activatesensor::Request::NONE;
    // }

    // activate_client_->async_send_request(
    //     activate_request, [this](rclcpp::Client<interfaces::srv::Activatesensor>::SharedFuture future) {
    //         auto res = future.get();
    //         RCLCPP_DEBUG_STREAM(this->get_logger(), "Start Sync sensor Time is :"<< res->header.stamp.sec << "." <<
    //                     res->header.stamp.nanosec);
    //         if (this->status == WORK_STATUS::START) {
    //             if (res->header.stamp.sec == 0)
    //                 rclcpp::shutdown();
    //             this->status = WORK_STATUS::WAITING;

    //             first_time_activate_camera_stamp_in_world_ =
    //                 res->header.stamp; 
    //         }
    //         RCLCPP_DEBUG(this->get_logger(), "");
    //     });
}

void SensorSyncNode::CalculateAwakeTime() {
    rclcpp::Duration time =
        first_time_activate_camera_stamp_in_world_ -
        first_time_activate_camera_stamp_in_camera_;  

    camera_awake_time_ = time + rclcpp::Duration::from_seconds(0.002);  
    RCLCPP_INFO_STREAM(this->get_logger(), "camera_awake_time : " << camera_awake_time_.nanoseconds());
    RCLCPP_INFO_STREAM(this->get_logger(), "first_time_activate_camera_stamp_in_world_ : " <<
                first_time_activate_camera_stamp_in_world_.nanoseconds());
    RCLCPP_INFO_STREAM(this->get_logger(), "first_time_activate_camera_stamp_in_camera_ : " <<
                first_time_activate_camera_stamp_in_camera_.nanoseconds());

}


builtin_interfaces::msg::Time SensorSyncNode::SyncImageTimeStamp(const rclcpp::Time& time) {
    rclcpp::Time t = rclcpp::Time(time) + camera_awake_time_;
    return t; 
}
std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> SensorSyncNode::get_node_base_interface() {
    return rclcpp::Node::get_node_base_interface();
}

}  // namespace sensorsync

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sensorsync::SensorSyncNode)
#ifdef USE_ROS2
#include "IO/dataSource/dataSource_ROS2_bag.h"
#include "Algorithm/vision/camModel/camModel.h"
#include "rclcpp/rclcpp.hpp"

namespace DeltaVins {

DataSource_ROS2_bag::DataSource_ROS2_bag(const std::string& bag_file)
    : rclcpp::Node("DeltaVINSTest") {
    this->declare_parameter("DataSourcePath", "");
    auto bag_file_from_ros_param = this->get_parameter("DataSourcePath").as_string();
    RCLCPP_INFO(this->get_logger(), "bag_file_from_ros_param: %s", bag_file_from_ros_param.c_str());
    std::string bag_file_final = bag_file_from_ros_param.empty() ? bag_file : bag_file_from_ros_param;
    reader_ = std::make_shared<rosbag2_cpp::Reader>();
    reader_->open(bag_file_final);

    for(auto& sensor : Config::ROS2SensorTopics){
        if(sensor.type == ROS2SensorType::StereoCamera){
            topic_map_[sensor.topics[0]] = std::make_pair(sensor.type, sensor.sensor_id);
            topic_map_[sensor.topics[1]] = std::make_pair(sensor.type, sensor.sensor_id);
            stereo_map_[sensor.topics[0]] = StereoType::LEFT;
            stereo_map_[sensor.topics[1]] = StereoType::RIGHT;
        }else{
            topic_map_[sensor.topics[0]] = std::make_pair(sensor.type, sensor.sensor_id);
        }
    }
}

DataSource_ROS2_bag::~DataSource_ROS2_bag() { reader_->close(); }

bool DataSource_ROS2_bag::HaveThingsTodo() { 
    keep_running_ = rclcpp::ok() && reader_->has_next();
    return keep_running_;
}

void DataSource_ROS2_bag::DoWhatYouNeedToDo() {
    auto bag_message = reader_->read_next();
    auto topic_name = bag_message->topic_name;
    if(topic_map_.count(topic_name) == 0){
        return;
    }
    auto sensor_type = topic_map_[topic_name].first;
    auto sensor_id = topic_map_[topic_name].second;
    if (sensor_type == ROS2SensorType::MonoCamera) {
        auto image_data = std::make_shared<ImageData>();
        sensor_msgs::msg::Image::SharedPtr sensor_msg_img = std::make_shared<sensor_msgs::msg::Image>();
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        image_serializer_.deserialize_message(&serialized_msg, sensor_msg_img.get());
        auto cv_ptr = cv_bridge::toCvCopy(sensor_msg_img, sensor_msg_img->encoding);
        image_data->image = cv_ptr->image;
        image_data->timestamp = sensor_msg_img->header.stamp.sec * 1e9 + sensor_msg_img->header.stamp.nanosec;
        image_data->cam_id = sensor_id;
        if(image_data_cache_){
            std::lock_guard<std::mutex> lck(mtx_image_observer_);
            for(auto& callback : image_observers_) {
                callback->OnImageReceived(image_data_cache_);
            }
        }
        image_data_cache_ = image_data;
    }else if (sensor_type == ROS2SensorType::IMU) {
        Matrix3f Rci = CamModel::getCamModel()->getRci();
        ImuData imu_data;
        sensor_msgs::msg::Imu::SharedPtr sensor_msg_imu = std::make_shared<sensor_msgs::msg::Imu>();
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        imu_serializer_.deserialize_message(&serialized_msg, sensor_msg_imu.get());
        imu_data.timestamp = sensor_msg_imu->header.stamp.sec * 1e9 + sensor_msg_imu->header.stamp.nanosec;
        imu_data.gyro = Eigen::Vector3f(sensor_msg_imu->angular_velocity.x, sensor_msg_imu->angular_velocity.y, sensor_msg_imu->angular_velocity.z);
        imu_data.acc = Eigen::Vector3f(sensor_msg_imu->linear_acceleration.x, sensor_msg_imu->linear_acceleration.y, sensor_msg_imu->linear_acceleration.z);
        imu_data.gyro = Rci * imu_data.gyro;
        imu_data.acc = Rci * imu_data.acc;
        {
            std::lock_guard<std::mutex> lck(mtx_imu_observer_);
            for(auto& callback : imu_observers_) {
                callback->OnImuReceived(imu_data);
            }
        }
    }else if (sensor_type == ROS2SensorType::StereoCamera) {
        std::lock_guard<std::mutex> lck(mutex_stereo_);
        sensor_msgs::msg::Image::SharedPtr sensor_msg_img = std::make_shared<sensor_msgs::msg::Image>();    
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        image_serializer_.deserialize_message(&serialized_msg, sensor_msg_img.get());
        auto stereo_type = stereo_map_[topic_name];
        if (stereo_type == StereoType::LEFT) {
            cv_ptr_left_.push_back(sensor_msg_img);
        } else {
            cv_ptr_right_.push_back(sensor_msg_img);
        }
        if (cv_ptr_left_.empty() || cv_ptr_right_.empty()) return;
        auto left = cv_ptr_left_.front();
        do{
            auto right = cv_ptr_right_.front();
            if (left->header.stamp == right->header.stamp) {
                cv_ptr_left_.pop_front();
                cv_ptr_right_.pop_front();

                // Convert ROS2 image to cv::Mat
                cv_bridge::CvImageConstPtr cv_ptr_left;
                cv_bridge::CvImageConstPtr cv_ptr_right;
                try {
                    cv_ptr_left = cv_bridge::toCvShare(
                        left, sensor_msgs::image_encodings::BGR8);
                    cv_ptr_right = cv_bridge::toCvShare(
                        right, sensor_msgs::image_encodings::BGR8);
                } catch (cv_bridge::Exception& e) {
                    LOGI("cv_bridge exception: %s", e.what());
                    return;
                }
                // Publish image
                ImageData::Ptr image_data = std::make_shared<ImageData>();
                cv::cvtColor(cv_ptr_left->image, image_data->image, cv::COLOR_BGR2GRAY);
                image_data->timestamp = left->header.stamp.sec * 1e9 + left->header.stamp.nanosec;
                image_data->cam_id = sensor_id;
                cv::cvtColor(cv_ptr_right->image, image_data->right_image, cv::COLOR_BGR2GRAY);
                if(image_data_cache_){  
                    std::lock_guard<std::mutex> lck(mtx_image_observer_);
                    for(auto& callback : image_observers_) {
                        callback->OnImageReceived(image_data_cache_);
                    }
                }
                image_data_cache_ = image_data;

            }else{
                auto left_stamp = left->header.stamp.sec * 1000000000 + left->header.stamp.nanosec;
                auto right_stamp = right->header.stamp.sec * 1000000000 + right->header.stamp.nanosec;
                if (left_stamp > right_stamp) {
                    cv_ptr_right_.pop_front();
                } else {
                    cv_ptr_left_.pop_front();
                }
            }
        }while(!cv_ptr_left_.empty() && !cv_ptr_right_.empty());
    }
}

}  // namespace DeltaVins
#endif
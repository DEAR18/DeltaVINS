#if USE_ROS2
#include "IO/dataSource/dataSource_ROS2.h"

#include <cv_bridge/cv_bridge.h>

#include "Algorithm/vision/camModel/camModel.h"
#include "utils/Config.h"
#include "utils/SensorConfig.h"

namespace DeltaVins {

DataSource_ROS2::DataSource_ROS2(bool is_bag)
    : rclcpp::Node("data_source_ros2") {
    is_bag_ = is_bag;

    if (is_bag_) {
        this->declare_parameter("DataSourcePath", "");
        auto bag_file_from_ros_param =
            this->get_parameter("DataSourcePath").as_string();
        RCLCPP_INFO(this->get_logger(), "bag_file_from_ros_param: %s",
                    bag_file_from_ros_param.c_str());
        std::string bag_file_final = bag_file_from_ros_param.empty()
                                         ? Config::DataSourcePath
                                         : bag_file_from_ros_param;
        reader_ = std::make_shared<rosbag2_cpp::Reader>();
        reader_->open(bag_file_final);

        for (auto& sensor : Config::ROS2SensorTopics) {
            if (sensor.type == ROS2SensorType::StereoCamera) {
                topic_map_bag_[sensor.topics[0]] =
                    std::make_pair(sensor.type, sensor.sensor_id);
                topic_map_bag_[sensor.topics[1]] =
                    std::make_pair(sensor.type, sensor.sensor_id);
                stereo_map_bag_[sensor.topics[0]] = StereoType::LEFT;
                stereo_map_bag_[sensor.topics[1]] = StereoType::RIGHT;
            } else {
                topic_map_bag_[sensor.topics[0]] =
                    std::make_pair(sensor.type, sensor.sensor_id);
            }
        }
    } else {
        // Create subscribers
        for (auto& topic : Config::ROS2SensorTopics) {
            if (topic.type == ROS2SensorType::MonoCamera) {
                std::string topic_name = topic.topics[0];
                // Create camera subscriber
                image_sub_.push_back(
                    this->create_subscription<sensor_msgs::msg::Image>(
                        topic_name, 10,
                        [this,
                         topic](const sensor_msgs::msg::Image::SharedPtr msg) {
                            ImageCallback(msg, topic.sensor_id);
                        }));
                topic_map_[topic_name] = topic.sensor_id;
                LOGI("Create camera subscriber: %s", topic_name.c_str());
            } else if (topic.type == ROS2SensorType::IMU) {
                std::string topic_name = topic.topics[0];
                // Create imu subscriber
                imu_sub_.push_back(
                    this->create_subscription<sensor_msgs::msg::Imu>(
                        topic_name, rclcpp::SensorDataQoS(),
                        [this,
                         topic](const sensor_msgs::msg::Imu::SharedPtr msg) {
                            ImuCallback(msg, topic.sensor_id);
                        }));
                topic_map_[topic_name] = topic.sensor_id;
                LOGI("Create imu subscriber: %s", topic_name.c_str());
            } else if (topic.type == ROS2SensorType::ACC) {
                std::string topic_name = topic.topics[0];
                // Create acc subscriber
                imu_sub_.push_back(
                    this->create_subscription<sensor_msgs::msg::Imu>(
                        topic_name, rclcpp::SensorDataQoS(),
                        [this,
                         topic](const sensor_msgs::msg::Imu::SharedPtr msg) {
                            AccGyroCallback(msg, true, topic.sensor_id);
                        }));
                topic_map_[topic_name] = topic.sensor_id;
                IMUParams imu_params =
                    SensorConfig::Instance().GetIMUParams(topic.sensor_id);
                imu_gyro_interval_ =
                    std::min(imu_gyro_interval_,
                             1.2f * 1 / (imu_params.acc_fps + 1e-8f));
                LOGI("Create acc subscriber: %s", topic_name.c_str());
            } else if (topic.type == ROS2SensorType::GYRO) {
                std::string topic_name = topic.topics[0];
                // Create gyro subscriber
                imu_sub_.push_back(
                    this->create_subscription<sensor_msgs::msg::Imu>(
                        topic_name, rclcpp::SensorDataQoS(),
                        [this,
                         topic](const sensor_msgs::msg::Imu::SharedPtr msg) {
                            AccGyroCallback(msg, false, topic.sensor_id);
                        }));
                topic_map_[topic_name] = topic.sensor_id;
                IMUParams imu_params =
                    SensorConfig::Instance().GetIMUParams(topic.sensor_id);
                imu_gyro_interval_ =
                    std::min(imu_gyro_interval_,
                             1.2f * 1 / (imu_params.gyro_fps + 1e-8f));
                LOGI("Create gyro subscriber: %s", topic_name.c_str());
            } else if (topic.type == ROS2SensorType::StereoCamera) {
                std::string topic_name = topic.topics[0];

                // Create stereo subscriber
                stereo_sub_.first =
                    this->create_subscription<sensor_msgs::msg::Image>(
                        topic.topics[0], 10,
                        [this,
                         topic](const sensor_msgs::msg::Image::SharedPtr msg) {
                            StereoCallback(msg, StereoType::LEFT,
                                           topic.sensor_id);
                        });
                stereo_sub_.second =
                    this->create_subscription<sensor_msgs::msg::Image>(
                        topic.topics[1], 10,
                        [this,
                         topic](const sensor_msgs::msg::Image::SharedPtr msg) {
                            StereoCallback(msg, StereoType::RIGHT,
                                           topic.sensor_id);
                        });

                topic_map_[topic_name] = topic.sensor_id;
                topic_map_[topic.topics[1]] = topic.sensor_id;
                LOGI("Create stereo subscriber: %s, %s", topic_name.c_str(),
                     topic.topics[1].c_str());
            } else if (topic.type == ROS2SensorType::ODOMETER) {
                std::string topic_name = topic.topics[0];
                // Create odom subscriber
                odom_sub_.push_back(
                    this->create_subscription<nav_msgs::msg::Odometry>(
                        topic_name, 10,
                        [this,
                         topic](const nav_msgs::msg::Odometry::SharedPtr msg) {
                            OdometerCallback(msg, topic.sensor_id);
                        }));
                topic_map_[topic_name] = topic.sensor_id;
                LOGI("Create odometer subscriber: %s", topic_name.c_str());
            }
        }

        // std::string topic_name = "/cam0/image_raw";
        // // Create camera subscriber
        // image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        //     topic_name, 10,
        //     std::bind(&DataSource_ROS2::ImageCallback, this,
        //               std::placeholders::_1));
        // // Create camera info subscriber
        // // cameraInfoSubscriber =
        // //
        // //
        // node->create_subscription<sensor_msgs::msg::CameraInfo>(cameraInfoTopicName,
        // // 10, std::bind(&DataSource_ROS2::cameraInfoCallback, this,
        // // std::placeholders::_1));

        // std::string imu_topic_name = "/imu0";
        // // Create imu subscriber
        // imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        //     imu_topic_name, rclcpp::SensorDataQoS(),
        //     std::bind(&DataSource_ROS2::ImuCallback, this,
        //     std::placeholders::_1));

        // // Create odom subscriber
        // // odomSubscriber =
        // // node->create_subscription<nav_msgs::msg::Odometry>(odomTopicName,
        // 10,
        // // std::bind(&DataSource_ROS2::odomCallback, this,
        // std::placeholders::_1));
    }
}

DataSource_ROS2::~DataSource_ROS2() {
    if (is_bag_) {
        LOGI("Close bag file");
        reader_->close();
    }
}

void DataSource_ROS2::NavSatFixCallback(
    const sensor_msgs::msg::NavSatFix::SharedPtr msg, int sensor_id) {
    NavSatFixData navSatFixData;
    navSatFixData.timestamp =
        msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
    navSatFixData.latitude = msg->latitude;
    navSatFixData.longitude = msg->longitude;
    navSatFixData.altitude = msg->altitude;
    navSatFixData.status = static_cast<NavSatFixStatus>(msg->status.status);
    navSatFixData.sensor_id = sensor_id;
    memcpy(navSatFixData.covariance, msg->position_covariance.data(),
           sizeof(double) * 9);
    {
        std::lock_guard<std::mutex> lck(mtx_nav_sat_fix_observer_);
        for (auto& observer : nav_sat_fix_observers_) {
            observer->OnNavSatFixReceived(navSatFixData);
        }
    }
}

void DataSource_ROS2::ImageCallback(
    const sensor_msgs::msg::Image::SharedPtr msg, int sensor_id) {
    // Convert ROS2 image to cv::Mat
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Publish image
    ImageData::Ptr image_data = std::make_shared<ImageData>();
    // image_data->image = cv_ptr->image;
    image_data->timestamp =
        msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;

    image_data->sensor_id = sensor_id;

    cv::cvtColor(cv_ptr->image, image_data->image, cv::COLOR_BGR2GRAY);

    if (is_bag_) {
        if (image_data_cache_) {
            std::lock_guard<std::mutex> lck(mtx_image_observer_);
            for (auto& observer : image_observers_) {
                observer->OnImageReceived(image_data_cache_);
            }
        }
        image_data_cache_ = image_data;
    } else {
        std::lock_guard<std::mutex> lck(mtx_image_observer_);
        for (auto& observer : image_observers_) {
            observer->OnImageReceived(image_data);
        }
    }
}

void DataSource_ROS2::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg,
                                  int sensor_id) {
    (void)sensor_id;
    auto camModel = SensorConfig::Instance().GetCamModel(0);
    // Matrix3f Rci = camModel->getRci();
    // Publish IMU data
    ImuData imu_data;
    imu_data.timestamp =
        msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
    Eigen::Vector3f acc(msg->linear_acceleration.x, msg->linear_acceleration.y,
                        msg->linear_acceleration.z);
    Eigen::Vector3f gyro(msg->angular_velocity.x, msg->angular_velocity.y,
                         msg->angular_velocity.z);
    imu_data.sensor_id = sensor_id;

    // Rotate the acceleration and angular velocity to the camera frame
    // imu_data.acc = Rci * acc;
    // imu_data.gyro = Rci * gyro;
    imu_data.acc = acc;
    imu_data.gyro = gyro;

    {
        std::lock_guard<std::mutex> lck(mtx_imu_observer_);
        for (auto& observer : imu_observers_) {
            observer->OnImuReceived(imu_data);
        }
    }
}

void DataSource_ROS2::AccGyroCallback(
    const sensor_msgs::msg::Imu::SharedPtr msg, const bool is_acc,
    int sensor_id) {
    static const uint gyro_buff_size = 5;
    static const uint acc_buff_size = 5;
    static std::deque<std::pair<int64_t, Eigen::Vector3f>> acc_buff;
    static std::deque<std::pair<int64_t, Eigen::Vector3f>> gyro_buff;
    static std::mutex mtx_buff;

    std::unique_lock<std::mutex> buff_lck(mtx_buff);
    int64_t timestamp = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
    Eigen::Vector3f acc(msg->linear_acceleration.x, msg->linear_acceleration.y,
                        msg->linear_acceleration.z);
    Eigen::Vector3f gyro(msg->angular_velocity.x, msg->angular_velocity.y,
                         msg->angular_velocity.z);
    ImuData imu_data;
    if (is_acc) {
        if (gyro_buff.empty()) {
            acc_buff.emplace_back(std::make_pair(timestamp, acc));
        } else {
            const auto& last_gyro = gyro_buff.back();
            float delta_t = (timestamp - last_gyro.first) * 1e-9f;
            if (delta_t > imu_gyro_interval_) {
                acc_buff.emplace_back(std::make_pair(timestamp, acc));
                // gyro_buff.clear();
            } else if (std::fabs(delta_t) <= imu_gyro_interval_) {
                // construct an imu data
                imu_data.timestamp = timestamp;
                imu_data.sensor_id = sensor_id;
                imu_data.acc = acc;
                imu_data.gyro = last_gyro.second;
                // gyro_buff.clear();
            }
        }
    } else {
        if (acc_buff.empty()) {
            gyro_buff.emplace_back(std::make_pair(timestamp, gyro));
        } else {
            const auto& last_acc = acc_buff.back();
            float delta_t = (timestamp - last_acc.first) * 1e-9f;
            if (delta_t > imu_gyro_interval_) {
                gyro_buff.emplace_back(std::make_pair(timestamp, gyro));
                // acc_buff.clear();
            } else if (std::fabs(delta_t) <= imu_gyro_interval_) {
                // construct an imu data
                imu_data.timestamp = timestamp;
                imu_data.sensor_id = sensor_id;
                imu_data.acc = last_acc.second;
                imu_data.gyro = gyro;
                // acc_buff.clear();
            }
        }
    }
    if (acc_buff.size() > acc_buff_size) {
        acc_buff.pop_front();
    }
    if (gyro_buff.size() > gyro_buff_size) {
        gyro_buff.pop_front();
    }
    buff_lck.unlock();

    if (imu_data.timestamp > 0) {
        std::lock_guard<std::mutex> lck(mtx_imu_observer_);
        for (auto& observer : imu_observers_) {
            observer->OnImuReceived(imu_data);
        }
    }
}

void DataSource_ROS2::StereoCallback(
    const sensor_msgs::msg::Image::SharedPtr msg, StereoType stereo_type,
    int sensor_id) {
    std::lock_guard<std::mutex> lck(mutex_stereo_);
    if (stereo_type == StereoType::LEFT) {
        if (!cv_ptr_left_.empty()) {
            auto timestamp_last = cv_ptr_left_.back()->header.stamp.sec * 1e9 +
                                  cv_ptr_left_.back()->header.stamp.nanosec;
            if (msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec <
                timestamp_last) {
                return;
            }
        }
        cv_ptr_left_.push_back(msg);
    } else {
        if (!cv_ptr_right_.empty()) {
            auto timestamp_last = cv_ptr_right_.back()->header.stamp.sec * 1e9 +
                                  cv_ptr_right_.back()->header.stamp.nanosec;
            if (msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec <
                timestamp_last) {
                return;
            }
        }
        cv_ptr_right_.push_back(msg);
    }
    if (cv_ptr_left_.empty() || cv_ptr_right_.empty()) return;

    do {
        auto left = cv_ptr_left_.front();
        auto right = cv_ptr_right_.front();
        // Need to make sure the left and right image have the same timestamp
        if (left->header.stamp == right->header.stamp) {
            cv_ptr_left_.pop_front();
            cv_ptr_right_.pop_front();

            // Convert ROS2 image to cv::Mat
            cv_bridge::CvImageConstPtr cv_ptr_left;
            cv_bridge::CvImageConstPtr cv_ptr_right;
            try {
                cv_ptr_left = cv_bridge::toCvShare(left, left->encoding);
                cv_ptr_right = cv_bridge::toCvShare(right, right->encoding);
            } catch (cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s",
                             e.what());
                return;
            }
            // if(cv_ptr_left->image.channels() != 1){
            //     LOGE("Left image channel is not 1");
            //     return;
            // }
            // Publish image
            ImageData::Ptr image_data = std::make_shared<ImageData>();
            if (cv_ptr_left->encoding == sensor_msgs::image_encodings::BGR8) {
                cv::cvtColor(cv_ptr_left->image, image_data->image,
                             cv::COLOR_BGR2GRAY);
            } else {
                image_data->image = cv_ptr_left->image.clone();
            }
            image_data->timestamp =
                left->header.stamp.sec * 1e9 + left->header.stamp.nanosec;
            image_data->sensor_id = sensor_id;
            if (cv_ptr_right->encoding == sensor_msgs::image_encodings::BGR8) {
                cv::cvtColor(cv_ptr_right->image, image_data->right_image,
                             cv::COLOR_BGR2GRAY);
            } else {
                image_data->right_image = cv_ptr_right->image.clone();
            }
            if (is_bag_) {
                if (image_data_cache_) {
                    std::lock_guard<std::mutex> lck(mtx_image_observer_);
                    for (auto& observer : image_observers_) {
                        observer->OnImageReceived(image_data_cache_);
                    }
                }
                image_data_cache_ = image_data;
            } else {
                {
                    std::lock_guard<std::mutex> lck(mtx_image_observer_);
                    for (auto& observer : image_observers_) {
                        observer->OnImageReceived(image_data);
                    }
                }
            }
        } else {
            int64_t left_stamp = left->header.stamp.sec * 1000000000ll +
                                 left->header.stamp.nanosec;
            int64_t right_stamp = right->header.stamp.sec * 1000000000ll +
                                  right->header.stamp.nanosec;
            if (left_stamp > right_stamp) {
                cv_ptr_right_.pop_front();
            } else if (left_stamp < right_stamp) {
                cv_ptr_left_.pop_front();
            }
        }
    } while (!cv_ptr_left_.empty() && !cv_ptr_right_.empty());
}

void DataSource_ROS2::OdometerCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg, int sensor_id) {
    OdometerData odom_data;
    odom_data.timestamp =
        msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
    odom_data.velocity = msg->twist.twist.linear.x;
    odom_data.angularVelocity = msg->twist.twist.angular.z;
    odom_data.sensor_id = sensor_id;
    {
        std::lock_guard<std::mutex> lck(mtx_odometer_observer_);
        for (auto& observer : odometer_observers_) {
            observer->OnOdometerReceived(odom_data);
        }
    }
}

// // void DataSource_ROS2::Join() {
// //     if (modules_thread_->joinable()) modules_thread_->join();
// //     run_ = false;
// }

bool DataSource_ROS2::HaveThingsTodo() {
    if (!rclcpp::ok()) {
        keep_running_ = false;
    }
    if (is_bag_) {
        keep_running_ = rclcpp::ok() && reader_->has_next();
    }
    return keep_running_;
}

void DataSource_ROS2::DoWhatYouNeedToDo() {
    if (is_bag_) {
        auto bag_message = reader_->read_next();
        auto topic_name = bag_message->topic_name;
        if (topic_map_bag_.count(topic_name) == 0) {
            return;
        }
        auto sensor_type = topic_map_bag_[topic_name].first;
        auto sensor_id = topic_map_bag_[topic_name].second;
        if (sensor_type == ROS2SensorType::MonoCamera) {
            sensor_msgs::msg::Image::SharedPtr sensor_msg_img =
                std::make_shared<sensor_msgs::msg::Image>();
            rclcpp::SerializedMessage serialized_msg(
                *bag_message->serialized_data);
            image_serializer_.deserialize_message(&serialized_msg,
                                                  sensor_msg_img.get());
            ImageCallback(sensor_msg_img, sensor_id);
        } else if (sensor_type == ROS2SensorType::IMU) {
            sensor_msgs::msg::Imu::SharedPtr sensor_msg_imu =
                std::make_shared<sensor_msgs::msg::Imu>();
            rclcpp::SerializedMessage serialized_msg(
                *bag_message->serialized_data);
            imu_serializer_.deserialize_message(&serialized_msg,
                                                sensor_msg_imu.get());
            ImuCallback(sensor_msg_imu, sensor_id);
        } else if (sensor_type == ROS2SensorType::StereoCamera) {
            sensor_msgs::msg::Image::SharedPtr sensor_msg_img =
                std::make_shared<sensor_msgs::msg::Image>();
            rclcpp::SerializedMessage serialized_msg(
                *bag_message->serialized_data);
            image_serializer_.deserialize_message(&serialized_msg,
                                                  sensor_msg_img.get());
            StereoCallback(sensor_msg_img, stereo_map_bag_[topic_name],
                           sensor_id);
        } else if (sensor_type == ROS2SensorType::GNSS) {
            sensor_msgs::msg::NavSatFix::SharedPtr sensor_msg_nav_sat_fix =
                std::make_shared<sensor_msgs::msg::NavSatFix>();
            rclcpp::SerializedMessage serialized_msg(
                *bag_message->serialized_data);
            nav_sat_fix_serializer_.deserialize_message(
                &serialized_msg, sensor_msg_nav_sat_fix.get());
            NavSatFixCallback(sensor_msg_nav_sat_fix, sensor_id);
        } else if (sensor_type == ROS2SensorType::ACC) {
            sensor_msgs::msg::Imu::SharedPtr sensor_msg_imu =
                std::make_shared<sensor_msgs::msg::Imu>();
            rclcpp::SerializedMessage serialized_msg(
                *bag_message->serialized_data);
            imu_serializer_.deserialize_message(&serialized_msg,
                                                sensor_msg_imu.get());
            AccGyroCallback(sensor_msg_imu, true, sensor_id);
        } else if (sensor_type == ROS2SensorType::GYRO) {
            sensor_msgs::msg::Imu::SharedPtr sensor_msg_imu =
                std::make_shared<sensor_msgs::msg::Imu>();
            rclcpp::SerializedMessage serialized_msg(
                *bag_message->serialized_data);
            imu_serializer_.deserialize_message(&serialized_msg,
                                                sensor_msg_imu.get());
            AccGyroCallback(sensor_msg_imu, false, sensor_id);
        } else if (sensor_type == ROS2SensorType::ODOMETER) {
            nav_msgs::msg::Odometry::SharedPtr sensor_msg_odom =
                std::make_shared<nav_msgs::msg::Odometry>();
            rclcpp::SerializedMessage serialized_msg(
                *bag_message->serialized_data);
            odom_serializer_.deserialize_message(&serialized_msg,
                                                 sensor_msg_odom.get());
            OdometerCallback(sensor_msg_odom, sensor_id);
        }
    } else {
        // sleep 100ms
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
}  // namespace DeltaVins
#endif
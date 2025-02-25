#if USE_ROS2

#include "IO/dataOuput/DataOutputROS.h"

namespace DeltaVins {

DataOutputROS::DataOutputROS() : Node("data_output_ros") {
    path_publisher_ =
        this->create_publisher<nav_msgs::msg::Path>("camera_path", 10);
    point_cloud_publisher_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud",
                                                              10);
    current_point_cloud_publisher_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "current_point_cloud", 10);
}

DataOutputROS::~DataOutputROS() {}

void DataOutputROS::CreateImagePublisher(const std::string& name) {
    image_publishers_[name] =
        this->create_publisher<sensor_msgs::msg::Image>(name, 10);
}

void DataOutputROS::PushImageTexture(unsigned char* data, int width, int height,
                                     int channels, const std::string& name) {
    (void)channels;
    cv::Mat image(height, width, CV_8UC3, data);
    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

    if (image_publishers_.count(name) == 0) {
        CreateImagePublisher(name);
    }
    // use cv_bridge to convert the image to a ROS message
    sensor_msgs::msg::Image::SharedPtr msg =
        cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", image).toImageMsg();
    image_publishers_[name]->publish(*msg);
}

void DataOutputROS::PushViewMatrix(std::vector<FrameGL>& v_Rcw) {
    // create a path message to publish the camera path
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = this->now();
    path.poses.resize(v_Rcw.size());
    for (size_t i = 0; i < v_Rcw.size(); i++) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->now();
        pose.header.frame_id = "map";
        pose.pose.position.x = v_Rcw[i].Twc(0, 3);
        pose.pose.position.y = v_Rcw[i].Twc(1, 3);
        pose.pose.position.z = v_Rcw[i].Twc(2, 3);
        Eigen::Quaternionf q(v_Rcw[i].Twc.block<3, 3>(0, 0));
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        path.poses[i] = pose;
    }
    path_publisher_->publish(path);
}

void DataOutputROS::PushWorldPoint(
    const std::vector<WorldPointGL>& point_cloud) {
    for (size_t i = 0; i < point_cloud.size(); i++) {
        world_points_[point_cloud[i].m_id] = point_cloud[i];
    }

    // create a point cloud message to publish the point cloud
    sensor_msgs::msg::PointCloud2 point_cloud_msg;
    point_cloud_msg.header.frame_id = "map";
    point_cloud_msg.header.stamp = this->now();
    point_cloud_msg.height = 1;
    point_cloud_msg.width = world_points_.size();
    point_cloud_msg.fields.resize(3);
    point_cloud_msg.fields[0].name = "x";
    point_cloud_msg.fields[0].offset = 0;
    point_cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    point_cloud_msg.fields[0].count = 1;
    point_cloud_msg.fields[1].name = "y";
    point_cloud_msg.fields[1].offset = 4;
    point_cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    point_cloud_msg.fields[1].count = 1;
    point_cloud_msg.fields[2].name = "z";
    point_cloud_msg.fields[2].offset = 8;
    point_cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    point_cloud_msg.fields[2].count = 1;
    point_cloud_msg.point_step = 12;
    point_cloud_msg.row_step = 12 * world_points_.size();
    point_cloud_msg.is_dense = true;
    point_cloud_msg.is_bigendian = false;
    point_cloud_msg.data.resize(12 * world_points_.size());

    for (auto it = world_points_.begin(); it != world_points_.end(); it++) {
        memcpy(&point_cloud_msg.data[it->first * 12], &it->second.P.x(), 4);
        memcpy(&point_cloud_msg.data[it->first * 12 + 4], &it->second.P.y(), 4);
        memcpy(&point_cloud_msg.data[it->first * 12 + 8], &it->second.P.z(), 4);
    }

    point_cloud_publisher_->publish(point_cloud_msg);

    // create a point cloud message to publish the current point cloud
    sensor_msgs::msg::PointCloud2 current_point_cloud_msg;
    current_point_cloud_msg.header.frame_id = "map";
    current_point_cloud_msg.header.stamp = this->now();
    current_point_cloud_msg.height = 1;
    current_point_cloud_msg.width = point_cloud.size();
    current_point_cloud_msg.fields.resize(6);
    current_point_cloud_msg.fields[0].name = "x";
    current_point_cloud_msg.fields[0].offset = 0;
    current_point_cloud_msg.fields[0].datatype =
        sensor_msgs::msg::PointField::FLOAT32;
    current_point_cloud_msg.fields[0].count = 1;
    current_point_cloud_msg.fields[1].name = "y";
    current_point_cloud_msg.fields[1].offset = 4;
    current_point_cloud_msg.fields[1].datatype =
        sensor_msgs::msg::PointField::FLOAT32;
    current_point_cloud_msg.fields[1].count = 1;
    current_point_cloud_msg.fields[2].name = "z";
    current_point_cloud_msg.fields[2].offset = 8;
    current_point_cloud_msg.fields[2].datatype =
        sensor_msgs::msg::PointField::FLOAT32;
    current_point_cloud_msg.fields[2].count = 1;
    current_point_cloud_msg.fields[3].name = "r";
    current_point_cloud_msg.fields[3].offset = 12;
    current_point_cloud_msg.fields[3].datatype =
        sensor_msgs::msg::PointField::FLOAT32;
    current_point_cloud_msg.fields[3].count = 1;
    current_point_cloud_msg.fields[4].name = "g";
    current_point_cloud_msg.fields[4].offset = 16;
    current_point_cloud_msg.fields[4].datatype =
        sensor_msgs::msg::PointField::FLOAT32;
    current_point_cloud_msg.fields[4].count = 1;
    current_point_cloud_msg.fields[5].name = "b";
    current_point_cloud_msg.fields[5].offset = 20;
    current_point_cloud_msg.fields[5].datatype =
        sensor_msgs::msg::PointField::FLOAT32;
    current_point_cloud_msg.fields[5].count = 1;
    current_point_cloud_msg.point_step = 24;
    current_point_cloud_msg.row_step = 24 * point_cloud.size();
    current_point_cloud_msg.is_dense = true;
    current_point_cloud_msg.is_bigendian = false;
    current_point_cloud_msg.data.resize(24 * point_cloud.size());
    float color[3] = {1.0f, 0.0f, 0.0f};
    for (size_t i = 0; i < point_cloud.size(); i++) {
        memcpy(&current_point_cloud_msg.data[i * 24], &point_cloud[i].P.x(), 4);
        memcpy(&current_point_cloud_msg.data[i * 24 + 4], &point_cloud[i].P.y(),
               4);
        memcpy(&current_point_cloud_msg.data[i * 24 + 8], &point_cloud[i].P.z(),
               4);
        memcpy(&current_point_cloud_msg.data[i * 24 + 12], color, 12);
    }
    current_point_cloud_publisher_->publish(current_point_cloud_msg);
}

}  // namespace DeltaVins

#endif
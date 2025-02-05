#if USE_ROS2

#include "IO/dataOuput/DataOutputROS.h"


namespace DeltaVins{

    DataOutputROS::DataOutputROS():Node("data_output_ros"){

        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("debug_image", 10);
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("camera_path", 10);
        point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);

    }


    DataOutputROS::~DataOutputROS(){

    }


    void DataOutputROS::PushImageTexture(unsigned char* data, int width, int height, int channels){

        cv::Mat image(height, width, CV_8UC3, data);
        cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

        // use cv_bridge to convert the image to a ROS message
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", image).toImageMsg();
        image_publisher_->publish(*msg);


    }

    void DataOutputROS::PushViewMatrix(std::vector<FrameGL>& v_Rcw){

        // create a path message to publish the camera path
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        path.header.stamp = this->now();
        path.poses.resize(v_Rcw.size());
        for(int i = 0; i < v_Rcw.size(); i++){
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


    void DataOutputROS::PushWorldPoint(const std::vector<WorldPointGL>& point_cloud){

        for(int i = 0; i < point_cloud.size(); i++){
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

        for(auto it = world_points_.begin(); it != world_points_.end(); it++){
            memcpy(&point_cloud_msg.data[it->first * 12], &it->second.P.x(), 4);
            memcpy(&point_cloud_msg.data[it->first * 12 + 4], &it->second.P.y(), 4);
            memcpy(&point_cloud_msg.data[it->first * 12 + 8], &it->second.P.z(), 4);
        }

        point_cloud_publisher_->publish(point_cloud_msg);
        
    }

}




#endif
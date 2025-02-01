#include "Algorithm/Initializer/StaticInitializer.h"
namespace DeltaVins{


StaticInitializer::StaticInitializer(){

}

StaticInitializer::~StaticInitializer(){

}

bool StaticInitializer::Initialize(const ImageData::Ptr imageData){

   if(last_points_.empty()) {
        // detect feature using shi-tomasi
        cv::goodFeaturesToTrack(imageData->image,last_points_,100,0.01,10);
        last_image_ = imageData->image;
        return false;
   }

    std::vector<cv::Point2f> tracked_points;
    std::vector<uchar> status;
    std::vector<float> err;
    std::vector<cv::Point2f> tracked_points_valid;

   // track feature using optical flow
    cv::calcOpticalFlowPyrLK(last_image_, 
                            imageData->image,
                            last_points_,
                            tracked_points,
                            status,
                            err,
                            cv::Size(21, 21),
                            3);
    float mean_moved_pixels = 0.0f;
    int valid_points = 0;
    for(size_t i = 0; i < status.size(); i++){
        if(status[i]){
            mean_moved_pixels += cv::norm(last_points_[i] - tracked_points[i]);
            valid_points++;
            tracked_points_valid.push_back(tracked_points[i]);
        }
    }
    if(valid_points > 0){      
        mean_moved_pixels /= valid_points;
    }else{
        LOGI("No feature to track");
        return false;
    }
    float alpha = 0.1f;
    moved_pixels_last_few_frames_ =
    alpha * moved_pixels_last_few_frames_ + (1 - alpha) * mean_moved_pixels;
    if(tracked_points_valid.size() < 100){
        // detect feature using shi-tomasi
        cv::goodFeaturesToTrack(imageData->image,tracked_points_valid,100,0.01,10);
    }
    // 更新上一帧的特征点和图像
    last_points_ = tracked_points_valid;
    last_image_ = imageData->image;

    return moved_pixels_last_few_frames_ < 0.5f;
}





}
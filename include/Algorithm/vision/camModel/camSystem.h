#pragma once

#include "camModel.h"
#include <unordered_map>

namespace DeltaVins{


    /**
     * @brief Camera System class, manage all the cameras in the system
     * @note The system can have multiple monocular cameras and stereo cameras
     * @note For stereo camera, it is viewed as a pair of left and right camera, the left camera id is the stereo camera id
     */

    class CamSystem{

        public:

            CamSystem();
            ~CamSystem();

            /**
             * @brief Add a monocular camera to the system
             * @param cam_model 
             * @param cam_id 
             * @return if cam_id already exists, return false, else return true
             */
            bool AddMonocularCamera(CamModel* cam_model,int cam_id);
            /**
             * @brief Add a stereo camera to the system
             * @param cam_model_left
             * @param cam_model_right
             * @param cam_id
             * @return if left_cam_id or right_cam_id already exists, return false, else return true
             */
            bool AddStereoCamera(CamModel* cam_model_left,CamModel* cam_model_right,int cam_id);

            /**
             * @brief Get the Monocular Camera object
             * @return CamModel* if cam_id is monocular camera id, return the camera model, if cam_id is stereo camera id, return the left camera model, if cam_id is not in the system, return nullptr
             */
            CamModel* GetMonocularCamera(int cam_id);
            /**
             * @brief Get the Stereo Camera object
             * @return std::pair<CamModel*,CamModel*> first is left camera, second is right camera, if cam_id is left camera id, return the pair of left and right camera, if cam_id is not Stereo camera, return nullptr, it is suggested to use IsStereoCamera to check if cam_id is stereo camera.
             */
            std::pair<CamModel*,CamModel*> GetStereoCamera(int cam_id);
            /**
             * @brief Check if cam_id is stereo camera
             * @return true if cam_id is stereo camera, false if cam_id is monocular camera or not in the system
             */
            bool IsStereoCamera(int cam_id);

        private:
            
            std::unordered_map<int,CamModel*> cam_models; // cam_id -> cam_model
            std::unordered_map<int,std::pair<CamModel*,CamModel*>> stereo_cam_models; // left_cam_id or right_cam_id -> (left_cam_model,right_cam_model)
        

    };

}
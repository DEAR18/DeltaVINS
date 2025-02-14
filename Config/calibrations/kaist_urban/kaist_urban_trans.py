import numpy as np
import argparse
import os
import cv2







if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--calib_path", type=str, default="")
    parser.add_argument("--output_path", type=str, default="")
    args = parser.parse_args()

    calib_path = args.calib_path
    output_path = args.output_path

    if not os.path.exists(output_path):
        os.makedirs(output_path)

    # load a line "R: 9 float" and "T: 3 float"
    with open(os.path.join(calib_path, "Vehicle2Stereo.txt"), "r") as f:
        for line in f:
            if line.startswith("R:"):
                R = np.array(line.split(":")[1].split(), dtype=np.float32)
                R = R.reshape(3, 3)
                continue
            if line.startswith("T:"):
                t = np.array(line.split(":")[1].split(), dtype=np.float32)
                t = t.reshape(3, 1)
                break
    
    # using cv2 to load left.yaml
    left_cam = cv2.FileStorage(os.path.join(calib_path, "left.yaml"), cv2.FILE_STORAGE_READ)
    left_cam_matrix = left_cam.getNode("camera_matrix").mat()
    left_dist_coeffs = left_cam.getNode("distortion_coefficients").mat()
    left_cam_matrix = left_cam_matrix.reshape(3, 3)
    left_dist_coeffs = left_dist_coeffs.reshape(1, 5)
    width = left_cam.getNode("image_width").real()
    height = left_cam.getNode("image_height").real()
    # left instrinc cv2.mat with 6 float(width, height, fx, fy, cx, cy)
    left_intrinsic = np.array([width, height, left_cam_matrix[0, 0], left_cam_matrix[1, 1], left_cam_matrix[0, 2], left_cam_matrix[1, 2]], dtype=np.float64)
    left_intrinsic = left_intrinsic.reshape(1, 6)
    # left distortion cv2.mat with 5 float(k1, k2, p1, p2, k3)
    left_distortion = np.array([left_dist_coeffs[0, 0], left_dist_coeffs[0, 1], left_dist_coeffs[0, 2], left_dist_coeffs[0, 3], left_dist_coeffs[0, 4]], dtype=np.float64)
    left_distortion = left_distortion.reshape(1, 5)

    # using cv2 to load right.yaml
    right_cam = cv2.FileStorage(os.path.join(calib_path, "right.yaml"), cv2.FILE_STORAGE_READ)
    right_cam_matrix = right_cam.getNode("camera_matrix").mat()
    right_dist_coeffs = right_cam.getNode("distortion_coefficients").mat()
    right_cam_matrix = right_cam_matrix.reshape(3, 3)
    right_dist_coeffs = right_dist_coeffs.reshape(1, 5)
    # right instrinc cv2.mat with 6 float(width, height, fx, fy, cx, cy)
    width = right_cam.getNode("image_width").real()
    height = right_cam.getNode("image_height").real()
    right_intrinsic = np.array([width, height, right_cam_matrix[0, 0], right_cam_matrix[1, 1], right_cam_matrix[0, 2], right_cam_matrix[1, 2]], dtype=np.float64)
    right_intrinsic = right_intrinsic.reshape(1, 6)
    # right distortion cv2.mat with 5 float(k1, k2, p1, p2, k3)
    right_distortion = np.array([right_dist_coeffs[0, 0], right_dist_coeffs[0, 1], right_dist_coeffs[0, 2], right_dist_coeffs[0, 3], right_dist_coeffs[0, 4]], dtype=np.float64)
    right_distortion = right_distortion.reshape(1, 5)

    # compact Extrinsic to 4x4 matrix
    cam_extrinsic = np.concatenate((R, t), axis=1)
    cam_extrinsic = np.concatenate((cam_extrinsic, np.array([0, 0, 0, 1]).reshape(1, -1)), axis=0)


    f = cv2.FileStorage(os.path.join(output_path, "camera.yaml"), cv2.FILE_STORAGE_WRITE)
    f.write("SensorType", "StereoCamera")
    # output the result using cv2
    f.write("SensorId", 0)
    f.write("CamType", "RadTan")
    f.write("Intrinsic", left_intrinsic)
    f.write("Distortion", left_distortion)
    f.write("Intrinsic_right", right_intrinsic)
    f.write("Distortion_right", right_distortion)
    f.write("Tbs", cam_extrinsic)
    f.write("IsStereo",0 )
    f.write("PixelNoise", 0.7)
    f.write("ImageSampleFps", 10)

    IMU_path = os.path.join(output_path, "imu.yaml")

    # load Vehicle2IMU.txt
    with open(os.path.join(calib_path, "Vehicle2IMU.txt"), "r") as f:
        for line in f:
            if line.startswith("R:"):
                R = np.array(line.split(":")[1].split(), dtype=np.float32)
                R = R.reshape(3, 3)
                continue
            if line.startswith("T:"):
                t = np.array(line.split(":")[1].split(), dtype=np.float32)
                t = t.reshape(3, 1)
                break
    
    # compact Extrinsic to 4x4 matrix
    imu_extrinsic = np.concatenate((R, t), axis=1)
    imu_extrinsic = np.concatenate((imu_extrinsic, np.array([0, 0, 0, 1]).reshape(1, -1)), axis=0)

    # using cv2 to load imu.yaml
    imu = cv2.FileStorage(IMU_path, cv2.FILE_STORAGE_WRITE)
    imu.write("SensorType", "IMU")
    imu.write("SensorId", 0)
    imu.write("Tbs", imu_extrinsic)
    imu.write("ImuSampleFps", 100)
    imu.write("GyroNoise", 2e-4)
    imu.write("AccNoise", 2e-3)
    imu.write("GyroBiasNoise", 2e-5)
    imu.write("AccBiasNoise", 3e-3)


    # output the result using cv2
    
    

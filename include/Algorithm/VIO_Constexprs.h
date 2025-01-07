#pragma once

#if DISABLE_ACC_BIAS
constexpr int IMU_STATE_DIM  = 6;
#else
constexpr int IMU_STATE_DIM  = 9;
#endif

constexpr int MAX_WINDOW_SIZE = 10;
constexpr int CAM_DELETE_STEP = 3;


constexpr int PLANE_DIM = 3;
constexpr int CAM_STATE_DIM  = 6;
constexpr int NEW_STATE_DIM = CAM_STATE_DIM+IMU_STATE_DIM;

constexpr int MAX_POINT_SIZE = 20;
constexpr int MAX_ADDITIONAL_POINT = 4;
constexpr int MAX_ALL_POINT_SIZE = MAX_POINT_SIZE + MAX_ADDITIONAL_POINT;

constexpr int MAX_MATRIX_SIZE = CAM_STATE_DIM*(MAX_WINDOW_SIZE+1)+IMU_STATE_DIM*2 + MAX_ALL_POINT_SIZE *3 + PLANE_DIM +1;

constexpr int MAX_OBS_SIZE = MAX_ALL_POINT_SIZE * MAX_WINDOW_SIZE + 9+4;

constexpr int MAX_H_ROW = MAX_WINDOW_SIZE * 2;
constexpr int MAX_H_COL = MAX_WINDOW_SIZE*CAM_STATE_DIM +4;

static_assert(MAX_MATRIX_SIZE%4==0,"Matrix Size must be 128 bit alignment");
static_assert(MAX_H_COL%4==0,"Matrix Size must be 128 bit alignment");

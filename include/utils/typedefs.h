#pragma once
#include <Eigen/Dense>
#include "Algorithm/VIO_Constexprs.h"
#include <utils/basicTypes.h>

namespace DeltaVins {


	using MatrixXf = Eigen::Matrix<float, -1, -1, EigenMajorType>;
    using VectorXf = Eigen::Matrix<float, -1, 1, EigenMajorType>;

    using MatrixXfR = Eigen::Matrix<float, -1, -1, Eigen::RowMajor>;

	using MatrixMf = Eigen::Matrix<float, MAX_MATRIX_SIZE, MAX_MATRIX_SIZE,EigenMajorType>;
	using MatrixMfR = Eigen::Matrix<float, MAX_MATRIX_SIZE, MAX_MATRIX_SIZE, Eigen::RowMajor>;
	using VectorMf = Eigen::Matrix<float, MAX_MATRIX_SIZE, 1>;

	using MatrixHfR = Eigen::Matrix<float, MAX_H_ROW, MAX_H_COL, Eigen::RowMajor>;
	//Todo: take out residual from stacked matrix
	using MatrixOfR = Eigen::Matrix<float, MAX_OBS_SIZE, MAX_MATRIX_SIZE+1, Eigen::RowMajor>;
	using VectorOf = Eigen::Matrix<float, MAX_OBS_SIZE, 1>;



	
}
/**
* This file is part of Delta_VIO.
*
* Delta_VIO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Delta_VIO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with Delta_VIO. If not, see <http://www.gnu.org/licenses/>.
*/
#include "precompile.h"
#include <sophus/se3.hpp>
#include <IO/dataBuffer/imuBuffer.h>
#include <Algorithm/IMU/ImuPreintergration.h>
#include <utils/utils.h>

namespace DeltaVins
{
    ImuBuffer::ImuBuffer(): CircularBuffer<ImuData, 10>()
    {
        m_GyroBias.setZero();
        m_AccBias.setZero();

        m_NoiseCov.setIdentity(6,6);
        m_NoiseCov.topLeftCorner(3,3) *= Config::GyroNoise2;
        m_NoiseCov.bottomRightCorner(3,3) *= Config::AccNoise2;
    }

    void ImuBuffer::updateBias(const Vector3f& dBg, const Vector3f& dBa)
    {
        m_GyroBias += dBg;
        m_AccBias += dBa;
    }

    void ImuBuffer::setBias(const Vector3f& bg, const Vector3f& ba)
    {
        m_GyroBias = bg;
        m_AccBias = ba;
    }

    void ImuBuffer::setZeroBias()
    {
        m_GyroBias.setZero();
        m_AccBias.setZero();
    }

    void ImuBuffer::getBias(Vector3f& bg, Vector3f& ba) const
    {
        bg = m_GyroBias;
        ba = m_AccBias;
    }

    Vector3f ImuBuffer::getGravity()
    {
        std::lock_guard<std::mutex> lck(m_gMutex);
        return mGravity;
    }

    bool ImuBuffer::getDataByBinarySearch(ImuData& imuData) const
    {
        int index = binarySearch<long long>(imuData.timestamp,Left);
        if (index < 0)
        {
            LOGE("t:%lld,imu0:%lld,imu1:%lld\n",imuData.timestamp,_buf[getDeltaIndex(m_tail,3)].timestamp,_buf[getDeltaIndex(m_head,-1)].timestamp);

            throw std::runtime_error("No Imu data found,Please check timestamp1");
        }

        auto& left = _buf[index];
        auto& right = _buf[getDeltaIndex(index, 1)];


        //linear interpolation
        float k = float(imuData.timestamp - left.timestamp) / float(right.timestamp - left.timestamp);

        imuData.gyro = linearInterpolate(left.gyro,right.gyro,k);
        imuData.acc = linearInterpolate(left.acc,right.acc,k);

        return index>=0;
    }

    inline Matrix3f vector2Jac(const Vector3f& x)
    {
        return Matrix3f::Identity() - 0.5f * crossMat(x);
    }

    /*----------------------------------------------------------------------------
    * IMU Preintegration on Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation"
    * http://www.roboticsproceedings.org/rss11/p06.pdf
    */
    bool ImuBuffer::imuPreIntegration(ImuPreintergration& ImuTerm) const
    {
        if (ImuTerm.t0 >= ImuTerm.t1)
        {
            LOGW("t0:%lld t1:%lld",ImuTerm.t0,ImuTerm.t1);
            throw std::runtime_error("t0>t1");
        }
        int Index0 = binarySearch<long long>(ImuTerm.t0,Left);
        int Index1 = binarySearch<long long>(ImuTerm.t1,Left);

        while(Index1<0){
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            Index1 = binarySearch<long long>(ImuTerm.t1,Left);
        }

        if (Index0 < 0 || Index1 < 0)
        {
            LOGW("dt0:%lld dt1:%lld,dT:%lld",ImuTerm.t0-_buf[Index0].timestamp,ImuTerm.t1-_buf[Index1].timestamp,ImuTerm.t1-ImuTerm.t0);
            if(Index0<0){
                LOGW("Error Code:%d",Index0);
                LOGE("t0:%lld,imu0:%lld\n",ImuTerm.t0,_buf[getDeltaIndex(m_tail,3)].timestamp);
            }
            if(Index1<0){
                LOGE("t1:%lld,imu1:%lld\n",ImuTerm.t1,_buf[getDeltaIndex(m_head,-1)].timestamp);
            }
            throw std::runtime_error("No Imu data found.Please check timestamp2");
        }
        ImuTerm.reset();

        Matrix3f& dRdg = ImuTerm.dRdg;
        Matrix3f& dPda = ImuTerm.dPda;
        Matrix3f& dPdg = ImuTerm.dPdg;
        Matrix3f& dVda = ImuTerm.dVda;
        Matrix3f& dVdg = ImuTerm.dVdg;

        Matrix3f& dR = ImuTerm.dR;
        Vector3f& dV = ImuTerm.dV;
        Vector3f& dP = ImuTerm.dP;
        Matrix3f dR0 = dR;
        Vector3f dV0 = dV;

        Matrix9f& cov = ImuTerm.Cov;

        MatrixXf A = MatrixXf::Identity(9, 9);
        MatrixXf B = MatrixXf::Zero(9, 6);


        int indexEnd = getDeltaIndex(Index1, 1);

        int index = Index0;

        float dt;

        Vector3f gyro, acc;

        while (index != indexEnd)
        {
            auto& imuData = _buf[index];
            int nextIndex = getDeltaIndex(index, 1);
            auto& nextImuData = _buf[nextIndex];

            if (index == Index0)
            {
                dt = nextImuData.timestamp - ImuTerm.t0;
                float a = nextImuData.timestamp -ImuTerm.t0 ;
                float b = nextImuData.timestamp - imuData.timestamp;
                float k = a / b;
                gyro = linearInterpolate(imuData.gyro, nextImuData.gyro, 1 - k);
                acc = linearInterpolate(imuData.acc, nextImuData.acc, 1 - k);
            }
            else if (index == Index1)
            {
                dt = ImuTerm.t1 - imuData.timestamp;
                float a = nextImuData.timestamp - ImuTerm.t1 ;
                float b = nextImuData.timestamp - imuData.timestamp;
                float k = a / b;
            	gyro = linearInterpolate(imuData.gyro, nextImuData.gyro, k);
                acc = linearInterpolate(imuData.acc, nextImuData.acc, k);
            }
            else
            {
                dt = nextImuData.timestamp - imuData.timestamp;
                gyro = (imuData.gyro + nextImuData.gyro) * 0.5f;
                acc = (imuData.acc + nextImuData.acc) * 0.5f;
            }
            dt *= 1e-9;


            gyro -= m_GyroBias;
            acc -= m_AccBias;

            Vector3f ddV0 = acc*dt;
            Vector3f ddR0 = gyro*dt;

            Matrix3f ddR = Sophus::SO3Group<float>::exp(ddR0).matrix();

            //update covariance iteratively
            A.topLeftCorner(3,3) = ddR.transpose();
            A.block<3, 3>(3, 0) = -dR0 * crossMat(ddV0);
            A.bottomLeftCorner(3,3) = 0.5*dt *A.block<3, 3>(3, 0);
            A.block<3, 3>(6, 3) = Matrix3f::Identity() * dt;

            B.topLeftCorner(3,3) = vector2Jac(ddR0) * dt;
            B.block<3, 3>(3, 3) = dR0 * dt;
            B.block<3, 3>(6, 3) = dR0 * (0.5 * dt * dt);

            cov = A * cov * A.transpose() + B * m_NoiseCov * B.transpose();

            //update Jacobian Matrix iteratively

            dPdg += dVdg * dt + A.bottomLeftCorner(3,3) * dRdg;
            dPda += dVda * dt - dR0 * (0.5 * dt * dt);
            dVdg -= dR0 * crossMat(ddV0) * dRdg;
            dVda -= dR0 * dt;
            dRdg = ddR.transpose() * dRdg - vector2Jac(ddR0) * dt;

            //update delta states iteratively
            Vector3f ddV = dR0 * acc * dt;
            dP += dV0 * dt + 0.5* ddV * dt;
            dR0 = dR = dR0 * ddR;
            dV0 = dV = dV0 + ddV;


            index = nextIndex;
        }

        // add time
        ImuTerm.dT += ImuTerm.t1 - ImuTerm.t0;

    	
        if(ImuTerm.dT>70000000)
            LOGW("Detected a Frame Drop, dT:%lld " ,ImuTerm.dT);

        return true;
    }


    void ImuBuffer::onImuReceived(const ImuData& imuData)
    {
        _buf[m_head] = imuData;

        //do low pass filter to get gravity
        static Eigen::Vector3f gravity = imuData.acc;
        gravity = 0.95f * gravity + 0.05f * imuData.acc;
        {
            std::lock_guard<std::mutex> lck(m_gMutex);
            mGravity = gravity;
        }

        pushIndex();
    }

    Vector3f ImuBuffer::getGravity(long long timestamp)
    {
        BufferIndex index0 = binarySearch<long long>(timestamp,Left);
        int nSize = index0 > m_tail ? index0 - m_tail : index0+_END-m_tail;
    	BufferIndex _index = getDeltaIndex(index0, nSize>=21?-20:-nSize+1);
        BufferIndex index_ = index0;
        index0 = _index;
        Vector3f gravity = _buf[index0].acc;
        while (index0 != index_) {
            gravity = 0.95f * gravity + 0.05f * _buf[index0].acc;
            index0 = getDeltaIndex(index0, 1);
        }
        return gravity;
    }

    long long ImuBuffer::getNextSyncTimestamp(int &imuIdx, long long lastTimeStamp) {
        if(imuIdx == -1) {
            do {
                BufferIndex _index = m_tail;
                while (_index != m_head) {
                    if (_buf[_index].syncFlag) {
                        imuIdx = _buf[_index].idx;
                        return _buf[_index].timestamp;
                    }
                    _index = getDeltaIndex(_index, 1);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            } while (imuIdx < 0);
        }
        else{
            int ret = -1;
            while (ret == -1) {
                int nextIndex = imuIdx + Config::nImuPerImage;
                int ret = binarySearch<int>(nextIndex, Exact);
                if (ret > 0) {
                    imuIdx = _buf[ret].idx;
                    return _buf[ret].timestamp;
                } else if (ret == -2) {
                    imuIdx = nextIndex;
                    return lastTimeStamp + 1000000000 / Config::nImageSample;
                }else if(ret == -3){
                    throw std::runtime_error("Image Slow");
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));

            }
        }
        return 0;
    }

    bool ImuBuffer::detectStatic(long long timestamp)const {

        BufferIndex index1 = binarySearch(timestamp, Left);
        int nSize = index1 > m_tail ? index1 - m_tail : index1 + _END - m_tail;
        if (nSize < 100) return false;

        nSize = nSize < 200 ? nSize:200;
    	
        BufferIndex index0 = getDeltaIndex(index1, -nSize);
        Vector3f sum_acc(0,0,0);
        Vector3f sum_gyro(0,0,0);
    	for (int i=0;i< nSize;++i)
    	{
            auto& imu_data = _buf[getDeltaIndex(index0, i)];
            sum_acc += imu_data.acc;
            sum_gyro += imu_data.gyro;
    	}
        Vector3f mean_acc = sum_acc / nSize;
        Vector3f mean_gyro = sum_gyro / nSize;

        float g_div = 0;
        float a_div = 0;

    	for (int i=0;i< nSize;++i)
    	{
            auto& imu_data = _buf[getDeltaIndex(index0, i)];
            a_div += (imu_data.acc - mean_acc).norm();
            g_div += (imu_data.gyro - mean_gyro).norm();
    	}
        a_div /= nSize;
        g_div /= nSize;

        float g_div_thresh = 0.04;
        float a_div_thresh = 0.5;

        if (g_div < g_div_thresh && a_div < a_div_thresh)
            return true;
        return false;
    	
    }


}

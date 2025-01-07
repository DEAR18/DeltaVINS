#pragma once

#include "dataStructure/ringBuffer.h"
#include "IO/dataSource/dataSource.h"

namespace DeltaVins {
	struct ImuPreintergration;


	class ImuBuffer: public CircularBuffer<ImuData,10>,public DataSource::ImuObserver
	{
	public:
	    friend class DataRecorder;
		static ImuBuffer& getInstance() {
			static ImuBuffer imuBuffer;
			return imuBuffer;
		}
		void onImuReceived(const ImuData& imuData) override;
		Vector3f getGravity(long long timestamp);
        Vector3f getGravity();

        long long getNextSyncTimestamp(int &imuIdx, long long lastTimeStamp);
		bool getDataByBinarySearch(ImuData& imuData) const;

		bool imuPreIntegration(ImuPreintergration& ImuTerm) const;

		void updateBias(const Vector3f& dBg, const Vector3f& dBa);

		void setBias(const Vector3f& bg, const Vector3f& ba);

		void setZeroBias();

		void getBias(Vector3f& bg, Vector3f& ba) const;

		bool detectStatic(long long timestamp) const;
		
	private:
		ImuBuffer();

		MatrixXf m_NoiseCov;

		Vector3f m_GyroBias;
		Vector3f m_AccBias;

		Vector3f mGravity;

		std::mutex m_gMutex;


    };

	


}
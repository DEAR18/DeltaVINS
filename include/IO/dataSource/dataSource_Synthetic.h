#pragma once
#include "dataSource.h"

namespace DeltaVins
{
	class DataSource_Synthetic:public DataSource
	{ 
	public:
		DataSource_Synthetic();
		~DataSource_Synthetic();

	private:
		void _loadIMU();
		void _loadImage();
		bool haveThingsTodo() override;
		void doWhatYouNeedToDo() override;
		std::string m_dataSetDir;
		std::string m_camDir;
		std::string m_imuDir;
		int m_iImageIdx;
		int m_iImuIndex;
		std::vector<_ImageData> m_vImages;
		std::vector<ImuData> m_vIMU;
	};




	
}

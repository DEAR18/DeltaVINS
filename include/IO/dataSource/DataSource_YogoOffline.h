#pragma once

#include <IO/dataSource/dataSource.h>
#include <dataStructure/IO_Structures.h>

namespace DeltaVins
{
	class DataSource_YogoOffline:public DataSource
	{
	public:

	public:
		DataSource_YogoOffline();
		~DataSource_YogoOffline();

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

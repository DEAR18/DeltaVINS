#pragma once
#include "dataSource.h"
#if defined(PLATFORM_ARM)
#include <serial/serial.h>
namespace DeltaVins
{



	class DataSource_Yogo :public DataSource
	{
	public:
		DataSource_Yogo();
		~DataSource_Yogo();

	private:

		bool haveThingsTodo() override;
		void doWhatYouNeedToDo() override;

		void _initIMU();
		void _initCamera();
		void _closeCamera();
		void _readIMU();
		void _readImage();

        serial::Serial * m_serial = nullptr;
	};

}
#endif
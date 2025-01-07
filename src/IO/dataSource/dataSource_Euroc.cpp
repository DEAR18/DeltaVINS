#include "precompile.h"
#include "IO/dataSource/dataSource_Euroc.h"

#include "Algorithm/vision/camModel/camModel.h"
#include "utils/utils.h"

using namespace std;

namespace DeltaVins
{
	DataSource_Euroc::DataSource_Euroc() : DataSource()
	{
		m_dataSetDir = Config::DataSourcePath;
		m_camDir = m_dataSetDir + "/cam0";
		m_imuDir = m_dataSetDir + "/imu0";

		m_iImageIdx = Config::ImageStartIdx;
		m_iImuIndex = 0;
		_loadImage();
		_loadIMU();
	}

	DataSource_Euroc::~DataSource_Euroc()
	{
	}

	void DataSource_Euroc::_loadIMU()
	{
		ifstream imuCsv;
		imuCsv.open(m_imuDir + "/data.csv");
		static Matrix3f Rci = CamModel::getCamModel()->getRci();
		ImuData imuData;
		std::string s;
		getline(imuCsv, s);//read first comment line
		while (!imuCsv.eof())
		{
			getline(imuCsv, s);
			if (s.empty()) continue;

			auto nums = split(s, ',');
			imuData.timestamp = strtoll(nums[0].c_str(), nullptr, 10) ;

			for (int i = 0; i < 3; i++)
			{
				imuData.gyro(i) = strtof(nums[i + 1].c_str(), nullptr);
				imuData.acc(i) = strtof(nums[i + 4].c_str(), nullptr);
			}
#if 0
			imuData.gyro[0] *= -1;
			std::swap(imuData.gyro[0], imuData.gyro[1]);

			imuData.acc[0] *= -1;
			std::swap(imuData.acc[0], imuData.acc[1]);
#endif
			imuData.acc = Rci * imuData.acc;
			imuData.gyro = Rci * imuData.gyro;

			m_vIMU.push_back(imuData);
		}
		imuCsv.close();
	}

	void DataSource_Euroc::_loadImage()
	{
		const string path = m_camDir + "/data.csv";
		ifstream camCsv;
		camCsv.open(path);
		if (!camCsv.is_open())
		{
			throw std::runtime_error("Failed to open file:" + path);
		}

		_ImageData image_data;
		string s;
		getline(camCsv, s);
		while (!camCsv.eof())
		{
			getline(camCsv, s);
			if (s.empty()) break;

			std::vector<std::string> nums = split(s, ',');
			image_data.timestamp = strtoll(nums[0].c_str(), nullptr, 10);
			image_data.imagePath = m_camDir + "/data/" + nums[0] + ".png";

			m_vImages.push_back(image_data);
		}

		camCsv.close();
	}

	bool DataSource_Euroc::haveThingsTodo()
	{
		if (m_iImageIdx < m_vImages.size())
		{
			return true;
		}else
		{
			keepRunning.store(false);
			return false;
		}
	}

	void DataSource_Euroc::doWhatYouNeedToDo()
	{
		auto& imageInput = m_vImages[m_iImageIdx];
		ImageData::Ptr imageData = std::make_shared<ImageData>();
		long long ddt = 5e7;
		imageData->timestamp = imageInput.timestamp;

		while (m_iImuIndex < m_vIMU.size() && m_vIMU[m_iImuIndex].timestamp < imageData->timestamp + ddt)
		{
			auto& imuInput = m_vIMU[m_iImuIndex++];
			std::lock_guard<std::mutex> lck(m_mtx_ImuObserver);

			for (auto& imu_listener : m_v_imuObservers)
			{
				imu_listener->onImuReceived(imuInput);
			}
		}


		auto img = cv::imread(imageInput.imagePath, CV_LOAD_IMAGE_GRAYSCALE);
		if (img.empty())
		{
			throw std::runtime_error("Failed to load images");
		}
		int width_crop = 0;
		if (img.cols > 640)
			width_crop = (img.cols - 640) / 2;
		imageData->image = img.colRange(width_crop, width_crop + 640).clone();
		{
			std::lock_guard<std::mutex> lck(m_mtx_imageObserver);
			for (auto& image_listener : m_v_imageObservers)
			{
				image_listener->onImageReceived(imageData);
			}
		}
		m_iImageIdx++;

		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
}
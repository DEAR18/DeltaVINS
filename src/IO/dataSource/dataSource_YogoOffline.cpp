#include "precompile.h"
#include <IO/dataSource/DataSource_YogoOffline.h>

#include "Algorithm/vision/camModel/camModel.h"
#include "utils/utils.h"

using namespace std;

namespace DeltaVins
{
	DataSource_YogoOffline::DataSource_YogoOffline() : DataSource()
	{
		m_dataSetDir = Config::DataSourcePath;
		m_camDir = m_dataSetDir + "/cam0";
		//m_camDir = m_dataSetDir + "/biliteral";
		m_imuDir = m_dataSetDir + "/imu0";

		m_iImageIdx = 0;
		m_iImuIndex = 0;
		_loadImage();
		_loadIMU();
	}

	DataSource_YogoOffline::~DataSource_YogoOffline()
	{
	}

	void DataSource_YogoOffline::_loadIMU()
	{
		ifstream imuCsv;
		imuCsv.open(m_imuDir + "/data.csv");

		ImuData imuData;
		std::string s;

		while (!imuCsv.eof())
		{
			getline(imuCsv, s);//read first comment line

			auto nums = split(s, ',');
			if(nums.size()<9)
				continue;
			imuData.timestamp = strtoll(nums[0].c_str(), nullptr, 10);

			imuData.idx = strtol(nums[1].c_str(), nullptr, 10);
			imuData.syncFlag = strtol(nums[2].c_str(), nullptr, 10);
			for (int i = 0; i < 3; i++)
			{
				imuData.gyro(i) = strtof(nums[i + 3].c_str(), nullptr);
				imuData.acc(i) = strtof(nums[i + 6].c_str(), nullptr);
			}
			m_vIMU.push_back(imuData);
		}
		imuCsv.close();
	}

	void DataSource_YogoOffline::_loadImage()
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
		bool first = true;
		long long timestamp1;
		while (!camCsv.eof())
		{
			getline(camCsv, s);
			auto splits = split(s, ',');
			if(splits.size()<2)
				continue;;
			image_data.timestamp = strtoll(splits[0].c_str(), nullptr, 10);

			image_data.imagePath = m_dataSetDir+"/" + splits[1];

			m_vImages.push_back(image_data);
		}

		camCsv.close();
	}

	bool DataSource_YogoOffline::haveThingsTodo()
	{
		if (m_iImageIdx < m_vImages.size())
		{
			return true;
		}
		else
		{
			keepRunning.store(false);
			return false;
		}
	}

	void DataSource_YogoOffline::doWhatYouNeedToDo()
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
		imageData->image = img.clone();
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
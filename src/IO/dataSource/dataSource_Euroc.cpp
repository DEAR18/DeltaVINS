#include "IO/dataSource/dataSource_Euroc.h"

#include "Algorithm/vision/camModel/camModel.h"
#include "precompile.h"
#include "utils/utils.h"
#include "utils/SensorConfig.h"
using namespace std;

namespace DeltaVins {
DataSource_Euroc::DataSource_Euroc() : DataSource() {
    dataset_dir_ = Config::DataSourcePath;
    cam_dir_ = dataset_dir_ + "/cam0";
    imu_dir_ = dataset_dir_ + "/imu0";

    image_idx_ = Config::ImageStartIdx;
    imu_index_ = 0;
    _LoadImage();
    _LoadIMU();
}

DataSource_Euroc::~DataSource_Euroc() {}

void DataSource_Euroc::_LoadIMU() {
    ifstream imuCsv;
    imuCsv.open(imu_dir_ + "/data.csv");
    CamModel::Ptr camModel = SensorConfig::Instance().GetCamModel(0);
    // static Matrix3f Rci = camModel->getRci();
    ImuData imuData;
    std::string s;
    getline(imuCsv, s);  // read first comment line
    while (!imuCsv.eof()) {
        getline(imuCsv, s);
        if (s.empty()) continue;

        auto nums = split(s, ',');
        imuData.timestamp = strtoll(nums[0].c_str(), nullptr, 10);

        for (int i = 0; i < 3; i++) {
            imuData.gyro(i) = strtof(nums[i + 1].c_str(), nullptr);
            imuData.acc(i) = strtof(nums[i + 4].c_str(), nullptr);
        }
#if 0
			imuData.gyro[0] *= -1;
			std::swap(imuData.gyro[0], imuData.gyro[1]);

			imuData.acc[0] *= -1;
			std::swap(imuData.acc[0], imuData.acc[1]);
#endif
        // imuData.acc = Rci * imuData.acc;
        // imuData.gyro = Rci * imuData.gyro;

        imus_.push_back(imuData);
    }
    imuCsv.close();
}

void DataSource_Euroc::_LoadImage() {
    const string path = cam_dir_ + "/data.csv";
    ifstream camCsv;
    camCsv.open(path);
    if (!camCsv.is_open()) {
        throw std::runtime_error("Failed to open file:" + path);
    }

    _ImageData image_data;
    string s;
    getline(camCsv, s);
    while (!camCsv.eof()) {
        getline(camCsv, s);
        if (s.empty()) break;

        std::vector<std::string> nums = split(s, ',');
        image_data.timestamp = strtoll(nums[0].c_str(), nullptr, 10);
        image_data.imagePath = cam_dir_ + "/data/" + nums[0] + ".png";

        images_.push_back(image_data);
    }

    camCsv.close();
}

bool DataSource_Euroc::HaveThingsTodo() {
    if (image_idx_ < images_.size()) {
        return true;
    } else {
        keep_running_.store(false);
        return false;
    }
}

void DataSource_Euroc::DoWhatYouNeedToDo() {
    auto& imageInput = images_[image_idx_];
    ImageData::Ptr imageData = std::make_shared<ImageData>();
    long long ddt = 5e7;
    imageData->timestamp = imageInput.timestamp;

    while (imu_index_ < imus_.size() &&
           imus_[imu_index_].timestamp < imageData->timestamp + ddt) {
        auto& imuInput = imus_[imu_index_++];
        std::lock_guard<std::mutex> lck(mtx_imu_observer_);

        for (auto& imu_listener : imu_observers_) {
            imu_listener->OnImuReceived(imuInput);
        }
    }

    auto img = cv::imread(imageInput.imagePath, cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
        throw std::runtime_error("Failed to load images");
    }
    int width_crop = 0;
    if (img.cols > 640) width_crop = (img.cols - 640) / 2;
    imageData->image = img.colRange(width_crop, width_crop + 640).clone();
    {
        std::lock_guard<std::mutex> lck(mtx_image_observer_);
        for (auto& image_listener : image_observers_) {
            image_listener->OnImageReceived(imageData);
        }
    }
    image_idx_++;

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}
}  // namespace DeltaVins
#include "IO/dataSource/dataSource_Synthetic.h"

#include "Algorithm/vision/camModel/camModel.h"
#include "precompile.h"
#include "utils/utils.h"
#include "utils/SensorConfig.h"
using namespace std;

namespace DeltaVins {
DataSource_Synthetic::DataSource_Synthetic() : DataSource() {
    dataset_dir_ = Config::DataSourcePath;
    cam_dir_ = dataset_dir_ + "/cam0";
    imu_dir_ = dataset_dir_ + "/imu0";

    image_idx_ = Config::ImageStartIdx;
    imu_index_ = 0;
    _LoadImage();
    _LoadIMU();
}

DataSource_Synthetic::~DataSource_Synthetic() {}

void DataSource_Synthetic::_LoadIMU() {
    ifstream imuCsv;
    imuCsv.open(imu_dir_ + "/data.csv");
    CamModel::Ptr camModel = SensorConfig::Instance().GetCamModel(0);
    static Matrix3f Rci = camModel->getRci();
    ImuData imuData;
    std::string s;
    while (!imuCsv.eof()) {
        getline(imuCsv, s);
        if (s.empty()) continue;
        auto nums = split(s, ',');
        imuData.timestamp = strtoll(nums[0].c_str(), nullptr, 10);

        for (int i = 0; i < 3; i++) {
            imuData.gyro(i) = strtof(nums[i + 1].c_str(), nullptr);
            imuData.acc(i) = strtof(nums[i + 4].c_str(), nullptr);
        }

        imuData.acc = Rci * imuData.acc;
        imuData.gyro = Rci * imuData.gyro;

        imus_.push_back(imuData);
    }
    imuCsv.close();
}

void DataSource_Synthetic::_LoadImage() {
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

bool DataSource_Synthetic::HaveThingsTodo() {
    if (imu_index_ < imus_.size()) {
        return true;
    } else {
        keep_running_.store(false);
        return false;
    }
}

void DataSource_Synthetic::DoWhatYouNeedToDo() {
    ImageData::Ptr imageData;

    if (images_.empty()) {
        static long long lastTimestamp = imus_[0].timestamp + 150000000;

        imageData = std::make_shared<ImageData>();
        imageData->timestamp = lastTimestamp + 50000000;
        imageData->image = cv::Mat(480, 640, CV_8UC1);
        lastTimestamp = imageData->timestamp;
    } else {
        auto& imageInput = images_[image_idx_];
        imageData = std::make_shared<ImageData>();
        // long long ddt = 5e7;
        imageData->timestamp = imageInput.timestamp;

        auto img = cv::imread(imageInput.imagePath, cv::IMREAD_GRAYSCALE);
        if (img.empty()) {
            throw std::runtime_error("Failed to load images");
        }
        image_idx_++;
    }
    long long ddt = 50000000;
    while (imu_index_ < imus_.size() &&
           imus_[imu_index_].timestamp < imageData->timestamp + ddt) {
        auto& imuInput = imus_[imu_index_++];
        std::lock_guard<std::mutex> lck(mtx_imu_observer_);

        for (auto& imu_listener : imu_observers_) {
            imu_listener->OnImuReceived(imuInput);
        }
    }

    {
        std::lock_guard<std::mutex> lck(mtx_image_observer_);
        for (auto& image_listener : image_observers_) {
            image_listener->OnImageReceived(imageData);
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}
}  // namespace DeltaVins
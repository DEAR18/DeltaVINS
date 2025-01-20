#pragma once
#include "dataSource.h"

namespace DeltaVins {
class DataSource_Synthetic : public DataSource {
   public:
    DataSource_Synthetic();
    ~DataSource_Synthetic();

   private:
    void _LoadIMU();
    void _LoadImage();
    bool HaveThingsTodo() override;
    void DoWhatYouNeedToDo() override;
    std::string dataset_dir_;
    std::string cam_dir_;
    std::string imu_dir_;
    int image_idx_;
    int imu_index_;
    std::vector<_ImageData> images_;
    std::vector<ImuData> imus_;
};

}  // namespace DeltaVins

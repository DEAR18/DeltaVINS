#include "precompile.h"
#include "IO/dataSource/dataSource_Yogo.h"

#include <IO/dataBuffer/imageBuffer.h>
#include <IO/dataBuffer/imuBuffer.h>
#include <Algorithm/vision/camModel/camModel.h>
#if defined(PLATFORM_ARM)
#include <serial/serial.h>
#include <CameraLib.h>

#define MANUAL_EXPOSURE
#define OV7750_GREY
int ov7750_width = 640;
int ov7750_height = 480;


#define USE_ONE_THREAD 0




namespace DeltaVins
{
	DataSource_Yogo::DataSource_Yogo()
	{
	    _initCamera();
	    _initIMU();
	}

	DataSource_Yogo::~DataSource_Yogo()
	{
		
	}

	bool DataSource_Yogo::haveThingsTodo()
	{
		return true;
	}


    void convert(uint8_t *data)
    {

        //static int n;
        float s = 0.000001;
        size_t N = 67;
        const size_t A = 5;
        const size_t W = 19;
        const size_t T = 47;
        static long long sum_t = -1;
        static ImuBuffer&buffer = ImuBuffer::getInstance();
        ImuData&headData = buffer.getHeadNode();
        static Matrix3f Rci = CamModel::getCamModel()->getRci();
        //int last_imu_nums;
        static int lastIdx = -1;
//        if(data.size() != N) {
//            LOGW("IMU Data Length Error: length:%d", data.size());
//            return;
//        }
//        else
        {

            if (data[T] == 0x41)
            {
                int dt = (data[T + 2]) | (data[T + 3] << 8) | (data[T + 4] << 16) | (data[T + 5] << 24);
                int imu_nums = (data[T + 6]) | (data[T + 7] << 8) | (data[T + 8] << 16) | (data[T + 9] << 24);
                int camera_plus_flag = (data[T + 10]) | (data[T + 11] << 8) | (data[T + 12] << 16) | (data[T + 13] << 24);
                if(dt<0||(camera_plus_flag!=0&&camera_plus_flag!=1)) {
                    LOGW("IMU data Error:dt :%d flag:%d",dt,camera_plus_flag);
                    return;
                }
                if(sum_t<0) {
                    timespec t;
                    timespec_get(&t,TIME_UTC);
                    sum_t = t.tv_nsec+t.tv_sec*1000000000;
                    lastIdx = imu_nums;
                }
                else {
                    int deltaIdx = (imu_nums - lastIdx);

                    if(deltaIdx>10||deltaIdx<0){
                        LOGW("Imu deta Idx error:%d",deltaIdx);
                        return;
                    }
                    sum_t += dt* deltaIdx* 1000;
                    lastIdx = imu_nums;
                }
//                LOGW("timestamp : %llu dt:%d ,idx:%d cam_flag:%d",sum_t,dt,imu_nums,camera_plus_flag);

                headData.timestamp = sum_t;
                headData.idx = imu_nums;
                headData.syncFlag = camera_plus_flag;

            }
            else{
                LOGW("IMU Data Error1:");
                return;
            }



            // long long t = getTimestamp();
            if (data[W] == 0x20)
            {
                int wx = (data[W + 2]) | (data[W + 3] << 8) | (data[W + 4] << 16) | (data[W + 5] << 24);
                int wy = (data[W + 6]) | (data[W + 7] << 8) | (data[W + 8] << 16) | (data[W + 9] << 24);
                int wz = (data[W + 10]) | (data[W + 11] << 8) | (data[W + 12] << 16) | (data[W + 13] << 24);
                float wxf = wx * s * M_PI / 180.0;
                float wyf = wy * s * M_PI / 180.0;
                float wzf = wz * s * M_PI / 180.0;
                headData.gyro <<wxf,wyf,wzf;
//                LOGI("gyro : %f , %f , %f ",wxf,wyf,wzf);
            }
            else {
                LOGW("IMU Data Error2:");
                return;
            }

            if (data[A] == 0x10)
            {
                int ax = (data[A + 2]) | (data[A + 3] << 8) | (data[A + 4] << 16) | (data[A + 5] << 24);
                int ay = (data[A + 6]) | (data[A + 7] << 8) | (data[A + 8] << 16) | (data[A + 9] << 24);
                int az = (data[A + 10]) | (data[A + 11] << 8) | (data[A + 12] << 16) | (data[A + 13] << 24);
                float axf = ax * s ;
                float ayf = ay * s;
                float azf = az * s;
                headData.acc<<axf,ayf,azf;
//                LOGI("acc : %f , %f , %f \n",axf,ayf,azf);
            }
            else {
                LOGW("IMU Data Error3:");
                return;
            }

            headData.acc = Rci*headData.acc;
            headData.gyro = Rci*headData.gyro;

            buffer.pushIndex();
        }
    }


    void DataSource_Yogo::doWhatYouNeedToDo()
	{
#if USE_ONE_THREAD


        bool first = true;
        const int bufferSize = 1000;
        std::vector<uint8_t> data;
        data.reserve(bufferSize);
        std::vector<uint8_t> temp_data;
        std::vector<uint8_t> p_data;
        long long t = 0;
        long long start_t = 0;
        bool change_flag = false;
        m_serial->flushInput();//Clear buffer to get real timestamp

        std::vector<uint8_t> start_command{0x59, 0xe1, 0x22, 0x01, 0x00, 0x00, 0x11, 0x12, 0x47};
        //send start command
        m_serial->write(start_command);

        char s[3] = {0x59,0x53,0};
        std::string end(s);

        std::string ss;
        m_serial->readline(ss,200,end);

        data.push_back(0x59);
        data.push_back(0x53);
        m_serial->read(data, 65);

        int lastHeader = 0;
        while (true){

            if(data.size()>=67){

                while(data.size() - lastHeader >=67) {
                    p_data.insert(p_data.begin(), data.begin(), data.begin() + 67);
                    lastHeader = 67;
                    convert(&data[lastHeader]);
                    p_data.clear();
                    lastHeader+=67;
                }




            }

            int available = m_serial->available();
            size_t buffer_nums = m_serial->read(data, available);



        }



#else
	    std::thread *imuThread = new std::thread([&](){
	        this->_readIMU();
	    })  ;
	    std::thread *imageThread = new std::thread([&](){
	        this->_readImage();
	    });
	    imuThread->join();
	    imageThread->join();
#endif
	}



    void DataSource_Yogo::_readIMU() {
        bool first = true;
        const int bufferSize = 1000;
        std::vector<uint8_t> data;
        data.reserve(bufferSize);
        std::vector<uint8_t> temp_data;
        std::vector<uint8_t> p_data;
        long long t = 0;
        long long start_t = 0;
        bool change_flag = false;
        m_serial->flushInput();//Clear buffer to get real timestamp

        std::vector<uint8_t> start_command{0x59, 0xe1, 0x22, 0x01, 0x00, 0x00, 0x11, 0x12, 0x47};
        //send start command
        m_serial->write(start_command);

        char s[3] = {0x59,0x53,0};
        std::string end(s);

        std::string ss;
        m_serial->readline(ss,200,end);

        data.push_back(0x59);
        data.push_back(0x53);
        m_serial->read(data,65);
        convert(&data[0]);
        data.clear();

        int count = 0;
        //Todo: To make it more faster, erase and insert function of vector is too slow.
        while (true) {

            size_t available = m_serial->available();
            size_t buffer_nums = m_serial->read(data, available);
            LOGI("Serial Data Avaliable: %d Bytes",available);
            assert(data[0] == 0x59 && "Data From Serial Error");
            if(data.size()<69){
                std::this_thread::sleep_for(std::chrono::milliseconds(40));
                continue;
            };
            int lastHeader = 0;
            while (lastHeader+68<data.size()) {
                if (data[67 + lastHeader] == 0x59 && data[68 + lastHeader] == 0x53) {
                    convert(&data[lastHeader]);
                    lastHeader += 67;
                }

            }
            data.erase(data.begin(),data.begin()+lastHeader);


            if (8 < (getTimestamp() - start_t) / 1e9) {
                start_t = getTimestamp();
                m_serial->write(start_command);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(40));
        }
	}

	void DataSource_Yogo::_readImage()
	{

#ifdef OV7750_GREY
        cv::Mat ov7750(ov7750_height,ov7750_width,CV_8UC1);
#else
        cv::Mat ov7750(ov7750_height, ov7750_width, CV_8UC4);
#endif
        static ImageBuffer& buffer = ImageBuffer::getInstance();
        static ImuBuffer& imuBuffer = ImuBuffer::getInstance();
        ::ImageData imagedata;
        int tail = -1;
        long long lastImageTimestamp = -1;


        while (true) {
            captureTopImage(&imagedata);
            if (!imagedata.length) continue;
            ImageData::Ptr imageData = std::make_shared<ImageData>();
#if 0
#ifdef OV7750_GREY
            imageData->image = cv::Mat(ov7750_height, ov7750_width, CV_8UC1);
#else
            imageData->image = cv::Mat(ov7750_height, ov7750_width, CV_8UC4);
#endif
#ifdef OV7750_GREY
            memcpy(imageData->image.data, imagedata.data, ov7750_width * ov7750_height * sizeof(uint8_t));
#else
            memcpy(imageData->image.data,imagedata.data,ov7750_width*ov7750_height*4*sizeof(uint8_t));
#endif
#endif
            imageData->image = cv::Mat(ov7750_height,ov7750_width,CV_8UC1,imagedata.data);

            imageData->timestamp = imuBuffer.getNextSyncTimestamp(tail, lastImageTimestamp);

            {
#if !STATIC_OBSERVER
                std::lock_guard<std::mutex> lck(m_mtx_imageObserver);
#endif

                for (auto &image_listener : m_v_imageObservers) {
                    image_listener->onImageReceived(imageData);
                }
            }

            lastImageTimestamp = imageData->timestamp;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

        }

	}

    void DataSource_Yogo::_initIMU() {
        std::string port = "/dev/ttysWK2";
        unsigned long baud = 1000000;
        m_serial = new serial::Serial(port, baud, serial::Timeout::simpleTimeout(10));
        assert(m_serial->isOpen());
        m_serial->flush();
    }


    void DataSource_Yogo::_initCamera() {

	    system("v4l2-ctl -d /dev/video5 --set-ctrl 'vertical_blanking=2080'");
	    CameraParams param;
	    param.bAutoExposure = true;
	    param.exposureTime = Config::ExposureTime;
	    param.gain = Config::Gain;
	    param.imageFormat = V4L2_PIX_FMT_NV12;
	    param.width = ov7750_width;
	    param.height = ov7750_height;
	    param.bufferSize = 20;

        openAndInitTopCamera(&param,0);
    }

    void DataSource_Yogo::_closeCamera() {
        closeTopCamera();
    }
}
#endif

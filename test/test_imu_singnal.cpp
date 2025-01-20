//
// Created by chenguojun on 2020/5/21.
//

#include <serial/serial.h>
#include <cassert>
#include <thread>
#include <math.h>

inline long long getTimestamp()
{
    static timespec t;
    timespec_get(&t,TIME_UTC);
    return t.tv_nsec+t.tv_sec*1000000000;

}
std::vector<uint8_t> start_command{0x59, 0xe1, 0x22, 0x01, 0x00, 0x00, 0x11, 0x12, 0x47};


void convert(uint8_t *data)
{

    //static int n;
    float s = 0.000001;
    size_t N = 67;
    const size_t A = 5;
    const size_t W = 19;
    const size_t T = 47;
    static long long sum_t = -1;
    //int last_imu_nums;
    static int lastIdx = -1;
    static int camFlagCnt = 0;
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
                printf("\nIMU data Error:dt :%d flag:%d\n",dt,camera_plus_flag);
                return;
            }
            camFlagCnt+=camera_plus_flag;
            if(sum_t<0) {
                timespec t;
                timespec_get(&t,TIME_UTC);
                sum_t = t.tv_nsec+t.tv_sec*1000000000;
                lastIdx = imu_nums;
            }
            else {
                int deltaIdx = (imu_nums - lastIdx);

                if(deltaIdx>10||deltaIdx<0){
                    printf("\nImu deta Idx error:%d\n",deltaIdx);
                    return;
                }
                sum_t += dt* deltaIdx* 1000;
                lastIdx = imu_nums;
            }
            printf("timestamp : %llu dt:%d ,idx:%d cam_flag:%d,camFlagCnt%d ",sum_t,dt,imu_nums,camera_plus_flag,camFlagCnt);


        }
        else{
            printf("\nIMU Data Error1:\n");
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
            printf(" gyro : %f , %f , %f ",wxf,wyf,wzf);
        }
        else {
            printf("\nIMU Data Error2:\n");
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
            printf("\nacc : %f , %f , %f \n",axf,ayf,azf);
        }
        else {
            printf("\nIMU Data Error3:\n");
            return;
        }

    }
}




int main(int argc,char**argv){

    long long timeAll = 10000000000;
    if(argc>1)
        timeAll = strtoll(argv[1], nullptr,10);

    int64_t start_t = 0;

    std::string port = "/dev/ttysWK2";
    unsigned long baud = 1000000;
    auto m_serial = new serial::Serial(port, baud, serial::Timeout::simpleTimeout(10));
    assert(m_serial->isOpen());
    m_serial->flush();

    bool first = true;
    const int bufferSize = 1000;
    std::vector<uint8_t> data;
    data.reserve(bufferSize);
    std::vector<uint8_t> temp_data;
    std::vector<uint8_t> p_data;
    long long t = 0;
    bool change_flag = false;
    m_serial->flushInput();//Clear buffer to get real timestamp

    //send Start command
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


        if (timeAll< (getTimestamp() - start_t) / 1e9) {
            start_t = getTimestamp();
            m_serial->write(start_command);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
    }





}
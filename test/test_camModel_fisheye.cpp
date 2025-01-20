//
// Created by Administrator on 2020/3/25.
//

#include <precompile.h>
#include <utils/Config.h>
#include <Algorithm/vision/camModel/camModel.h>
#include <utils/log.h>
using namespace DeltaVins;

DataSource::Ptr dataSourcePtr = nullptr;

class TestFisheyeCamModel:public DataSource::ImageObserver{


public:
    TestFisheyeCamModel(){
        std::string datasetPath = Config::DataSourcePath;
        existOrMkdir(datasetPath+"/rectify");
        rectifyPath = datasetPath+"/rectify/";
        testJacobian();
        testReprojectErr();
    }
    void OnImageReceived(const ImageData::Ptr imageData) override{

        static CamModel* camModel = CamModel::getCamModel();
        cv::Mat rectifyImage;
        camModel->rectifyImage(imageData->image,rectifyImage,640,480,240,319.5,239.5);
        cv::imwrite(rectifyPath+std::to_string(imageData->timestamp)+".png",rectifyImage);

    }

    double testReprojectErr(){
        static CamModel* camModel = CamModel::getCamModel();
        float sum_err=0;
        int n = 0;
        int nErr = 0;
        for (float i = 0;  i<480 ;i+=0.1 ) {
            for (float j = 0;j  <640 ; j+=0.1) {
                Vector2f px(j,i);
                Vector3f ray = camModel->imageToCam(px);
                Vector2f px_reporj = camModel->camToImage(ray);
                float err = (px-px_reporj).norm();
                if(err>1.f){
                    printf("%f %f ->%f %f\n",j,i,px_reporj.x(),px_reporj.y());
                    nErr++;
                }else if(isnanf(err)){
                    printf("Nan %f %f ->%f %f\n",j,i,px_reporj.x(),px_reporj.y());
                    nErr++;
                }
                sum_err+=err;
                n++;

            }
        }

        printf("Num:%d ErrNum:%d Sum_err%f,Mean Err:%f\n",n,nErr,sum_err,sum_err/n);
    }

    void testJacobian(){
        static CamModel* camModel = CamModel::getCamModel();

        float d = 0.001f;
        Vector2f px,px2;
        px.x() = 200;
        px.y() = 200;

        Vector3f ray = camModel->imageToCam(px);
        px = camModel->camToImage(ray);

        Vector3f ray1 = ray;
        ray1.x() += d;

        Vector2f px1 = camModel->camToImage(ray1);
        float dudx = (px1.x() - px.x())/d;
        float dvdx = (px1.y() - px.y())/d;

        ray1 = ray;
        ray1.y() +=d;

        px1 = camModel->camToImage(ray1);
        float dudy = (px1.x() - px.x())/d;
        float dvdy = (px1.y() - px.y())/d;



        ray1 = ray;
        ray1.z() +=d;

        px1 = camModel->camToImage(ray1);
        float dudz = (px1.x() - px.x())/d;
        float dvdz = (px1.y() - px.y())/d;

        Matrix23f J23;
        camModel->camToImage(ray,J23);

        std::cout<<"Analytic Jacobian\n"<<J23<<std::endl;

        printf("Numerical Jacobian:\n%f %f %f\n%f %f %f\n",dudx,dudy,dudz,dvdx,dvdy,dvdz);



    }
    std::string rectifyPath;
};


int main(){

    logInit();
    Config::loadConfigFile("","");
    CamModel::loadCalibrations();
    TestFisheyeCamModel test;
    dataSourcePtr->AddImageObserver(&test);
    dataSourcePtr->Start();
    dataSourcePtr->Join();
    dataSourcePtr->Stop();


}
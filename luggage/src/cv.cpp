#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudabgsegm.hpp>
#include <opencv2/cudacodec.hpp>
#include <opencv2/cudaobjdetect.hpp>
#include <opencv2/cudaoptflow.hpp>
#include <opencv2/cudastereo.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudafeatures2d.hpp>


using namespace std;
using namespace cv;

VideoCapturecap(0);

int main(int argc, char ** argv)
{
    cuda::printCudaDeviceInfo(8);

    Mat img;
    cuda::GpuMat imgGpu;

    while(cap.isOpened()){
        auto start = getTickCount();

        cap.read(img);
        imgGpu.upload(img);

        //Filtering Image
        auto gaussianFilter = cuda::createGaussianFilter(CV_8UC1,CV_8UC1, {3,3},1);


        imgGpu.download(img);
        auto end = getTickCount();
        auto totalTime = (end - start) / getTickFrequency();
        auto fps = 1/totalTime;

        putText(img, "FPS: " + to_string(int(fps)), Point(50, 50), FONT_HERSHEY_DUPLEX, 1, Scalar(255, 255, 255), 2, false);
        imshow("Image", img);


        if(waitKey(1) == 'q'){
            break;
        }


    }
}
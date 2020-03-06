#include<iostream>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "ORB.hpp"
#include "C:\Users\zhaozhiyi\Desktop\orb\ORB.hpp"
#include "C:\Users\zhaozhiyi\Desktop\orb\ICAngles.cpp"
#include "C:\Users\zhaozhiyi\Desktop\orb\HarrisResponses.cpp"
#include "C:\Users\zhaozhiyi\Desktop\orb\fastAtan2.cpp"
int main(int args,char** argv){
    MyKeyPoint allkeypoints[240]={};
    int nfeatures=100,edgeThreshold=31,patchSize=31,scoreType=0,fastThreshold=20;
    int allkeypoints_num=0;
    int keypoints_num =0;
    double scaleFactor=1.2;
    xf::Rect layerInfo[8];
    float layerScale[8]={1,1.2,1.44,1.728,2.0736,2.48832,2.98598,3.58318};
    int layerInfo_x[8]={32,32,32,32,32,1543,32,1101};
    int layerInfo_y[8]={32,3096,5660,7807,9607,9607,11118,11118};
    int layerInfo_height[8]={3000,2500,2083,1736,1447,1206,1005,837};
    int layerInfo_width[8] ={3000,2500,2083,1736,1447,1206,1005,837};

    for(int i=0;i<8;i++){
        layerInfo[i].x =layerInfo_x[i];
        layerInfo[i].y =layerInfo_y[i];
        layerInfo[i].height =layerInfo_height[i];
        layerInfo[i].width =layerInfo_width[i];
    }


    cv::Mat img1 =cv::imread(argv[1],0);

    xf::Mat<TYPE,IMAGEPYRAMID_HEIGHT,IMAGEPYRAMID_WIDTH,XF_NPPC1> imp;
    imp.copyTo(img1.data);
    computeKeyPoints(imp,layerInfo,layerScale,allkeypoints, nfeatures, scaleFactor, edgeThreshold,
         patchSize, scoreType,fastThreshold, allkeypoints_num, keypoints_num);
    for(int i=0;i<100;i++){
        std::cout<<"keypoints_"<<i<<":\n"
        <<"x:"<<allkeypoints[i].x<<"  "
        <<"y:"<<allkeypoints[i].y<<"  "
        <<"octave:"<<allkeypoints[i].octave<<"  "
        <<"angle:"<<allkeypoints[i].angle<<"  "
        <<"response"<<allkeypoints[i].response<<"  "<<std::endl;
    }
	return 0;
}

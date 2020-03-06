#include "ORB.hpp"
#ifndef __COMPUTERORBKEYPOINT_H_
#define __COMPUTERORBKEYPOINT_H_
#include "features/xf_fast.hpp"
#define NMS 1
#define MAXCORNERS  1024
#define ALL_KEYPOINTS_NUMBER  240 
#define KEYPOINTS_NUMBER 100
enum { kBytes = 32, HARRIS_SCORE=0, FAST_SCORE=1 };
#endif
//#include "imgproc/xf_resize.hpp"
//#include "imgproc/xf_resize_headers.h"
const float HARRIS_K = 0.04f;
void computeKeyPoints(xf::Mat<TYPE,IMAGEPYRAMID_HEIGHT, IMAGEPYRAMID_WIDTH, NPC1>& imagePyramid,
        const xf::Rect layerInfo[8],
        const float layerScale[8],
        MyKeyPoint allKeypoints[ALL_KEYPOINTS_NUMBER],//实验中共�?�?216个，但是此处多给，最后一个作为哨�?
        int nfeatures, double scaleFactor,
        int edgeThreshold, int patchSize, int scoreType,
        int fastThreshold,int &allkeypoints_num,int &keypoints_num)
    {
        
        int i, nkeypoints, level;
        //int nlevels = (int)sizeof(layerInfo)/sizeof(xf::Rect);
        int nlevels =8; //openc默认金字塔层数是8
        int nfeaturesPerLevel[NLEVELS];//长度为nlevels

        // fill the extractors and descriptors for the corresponding scales
        float factor = (float)(1.0 / scaleFactor);
        float ndesiredFeaturesPerScale = nfeatures * (1 - factor) / (1 - (float)hls::pow((double)factor, (double)nlevels));

        int sumFeatures = 0;
        for (level = 0; level < nlevels - 1; level++)
        {
            nfeaturesPerLevel[level] = hls::round(ndesiredFeaturesPerScale);
            sumFeatures += nfeaturesPerLevel[level];
            ndesiredFeaturesPerScale *= factor;
        }
        nfeaturesPerLevel[nlevels - 1] = hls::max(nfeatures - sumFeatures, 0);

        // Make sure we forget about what is too close to the boundary
        //edge_threshold_ = std::max(edge_threshold_, patch_size_/2 + kKernelWidth / 2 + 2);

        // pre-compute the end of a row in a circular patch
        int  halfPatchSize = 31 / 2; //PatchSize = 31;
        int umax[(int)(31/2) + 2];

        int v, v0, vmax = hls::floor(halfPatchSize * hls::sqrt(2.f) / 2 + 1);
        int vmin = hls::ceil(halfPatchSize * hls::sqrt(2.f) / 2);
        for (v = 0; v <= vmax; ++v)
            umax[v] = hls::round(hls::sqrt((double)halfPatchSize * halfPatchSize - v * v));

        // Make sure we are symmetric
        for (v = halfPatchSize, v0 = 0; v >= vmin; --v)
        {
            while (umax[v0] == umax[v0 + 1])
                ++v0;
            umax[v] = v0;
            ++v0;
        }

    
        MyKeyPoint keypoints[100];//暂定100个点
        int counters[NLEVELS];  //长度为nlevels
        //keypoints.reserve(nfeaturesPerLevel[0] * 2);

        for (level = 0; level < nlevels; level++)
        {
            int featuresNum = nfeaturesPerLevel[level];
           // xf::Mat<TYPE,HEIGHT,WIDTH,NPC1> img = imagePyramid(layerInfo[level]);//选定区域
            xf::Mat<TYPE,HEIGHT,WIDTH,NPC1> img;
             // imagePyramid(layerInfo[level]);
        
             img.cols =layerInfo[level].width;
             img.rows =layerInfo[level].height;

                for(int j=0;j<layerInfo[level].height;j++){
                    for(int i=0;i<layerInfo[level].width;i++){
                img.data[j*img.cols+i] = imagePyramid.data[(layerInfo[level].y-1)*imagePyramid.cols+layerInfo[level].x+i];
                }
            }
        
            img.cols =layerInfo[level].width;
            img.rows =layerInfo[level].height;
            //xf::Mat<TYPE,HEIGHT,WIDTH,NPC1> mask(0,0);//mask()仿佛用不�?

            // Detect FAST features, 20 is a good threshold
            {   
                xf::Mat<TYPE,HEIGHT,WIDTH,NPC1> _dst(img.rows,img.cols);
                xf::fast<NMS,XF_8UC1,HEIGHT,WIDTH,NPC1>(img,_dst,fastThreshold);
                int number=0;
                for(int col=0;col<_dst.cols;col++){
                    for(int row=0;row<_dst.rows;row++){
                        if(_dst.data[row*_dst.cols+col]>250){
                            keypoints[number].x=col;
                            keypoints[number].y =row;
                            number++;
                        }
                    } 
                }
                keypoints_num=number; //把数组最后一个作为哨兵计数；
            }

            // Remove keypoints very close to the border

           // KeyPointsFilter::runByImageBorder(keypoints, img.size(), edgeThreshold);
     int borderSize =edgeThreshold;
     xf::Size imageSize(img.cols,img.rows);       
if( borderSize > 0)//31
    {
        if (imageSize.height <= borderSize * 2 || imageSize.width <= borderSize * 2){        
            keypoints_num=0;
        }
        else{
            for(int i=0;i<keypoints_num;i++){
                if(xf::Rect(xf::Point(borderSize, borderSize),imageSize).bContains(xf::Point(imageSize.width - borderSize, imageSize.height - borderSize))){
                    for(int j=i;j<keypoints_num;j++){
                        keypoints[i]=keypoints[i+1];
                    }
                }
            }
            
    }
    }
            // Keep more points than necessary as FAST does not give amazing corners
            //KeyPointsFilter::retainBest(keypoints, scoreType == ORB_Impl::HARRIS_SCORE ? 2 * featuresNum : featuresNum);
            int n_point;
            if( scoreType==HARRIS_SCORE ){
                     n_point =2*featuresNum;
            }else{
                n_point=featuresNum;
            }
           
               MyKeyPoint tmp;
               for(int i=0;i<keypoints_num;i++){
                   if(keypoints[i].response>keypoints[i+1].response){
                  for(int j=0;j<keypoints_num;j++){
                      
                      tmp = keypoints[j];
                      keypoints[j] =keypoints[j+1];
                      keypoints[j+1]=tmp;
                      }
                }
            }
             keypoints_num =n_point;

            nkeypoints = keypoints_num;
            counters[level] = nkeypoints;

            float sf = layerScale[level];
            for(i = 0; i < nkeypoints; i++)
            {
                keypoints[i].octave = level;
                keypoints[i].size = patchSize * sf;
            }
            for(int i=0;i<keypoints_num;i++){
                allKeypoints[allkeypoints_num+i]=keypoints[i];
            }
            allkeypoints_num +=keypoints_num;
            //std::copy(keypoints.begin(), keypoints.end(), std::back_inserter(allKeypoints));
        }


        nkeypoints = allkeypoints_num;
        if(nkeypoints == 0)
        {
            return;
        }
        xf::Mat<TYPE,HEIGHT,WIDTH,NPC1> responses;
        xf::Mat<TYPE,HEIGHT,WIDTH,NPC1> ukeypoints;
        //xf::Mat<TYPE,HEIGHT,WIDTH,NPC1> uresponses(1, nkeypoints, CV_32F);

        // Select best features using the Harris cornerness (better scoring than FAST)
        if(scoreType == HARRIS_SCORE)
        {
                HarrisResponses(imagePyramid, layerInfo, allKeypoints, 7, HARRIS_K,keypoints_num);

            MyKeyPoint newAllKeypoints[ALL_KEYPOINTS_NUMBER];  //图片中获得的值为216作为参�??,不再动�?�申请内存扩�?
            //newAllKeypoints.reserve(nfeaturesPerLevel[0] * nlevels);


            int offset = 0;
        //     for (level = 0; level < nlevels; level++)
        //     {
        //         int featuresNum = nfeaturesPerLevel[level];
        //         nkeypoints = counters[level];
        //         //keypoints.resize(nkeypoints);申请过内存了固定数组结构，不再重新调�?
        //         /*
        //         std::copy(allKeypoints.begin() + offset,
        //             allKeypoints.begin() + offset + nkeypoints,
        //             keypoints.begin());
        //             */
        //         for(int i=0;i<(offset+nkeypoints-1);i++){
        //             keypoints[i] =allKeypoints[i];
        //         }
        //         keypoints_num =nkeypoints;
        //         offset += nkeypoints;

        //         //cull to the final desired level, using the new Harris scores.
        //         //KeyPointsFilter::retainBest(keypoints, featuresNum);
        //         /*******/
        //      int n_point1;
        //     if( scoreType==HARRIS_SCORE ){
        //              n_point1 =2*featuresNum;
        //     }else{
        //         n_point1=featuresNum;
        //     }
        //     {
        //        MyKeyPoint tmp;
               
        //           for(int j=0;j<keypoints_num;j++){
        //               if(keypoints[j].response>keypoints[j+1].response)
        //               tmp = keypoints[j];
        //               keypoints[j] =keypoints[j+1];
        //               keypoints[j+1]=tmp;
        //               }
                
        //     }
        //    keypoints_num =n_point1;


        //         /******/
        //         //std::copy(keypoints.begin(), keypoints.end(), std::back_inserter(newAllKeypoints));
        //         for(int i=0;i<keypoints_num;i++){
        //             newAllKeypoints[allkeypoints_num+i]=keypoints[i];
        //             allkeypoints_num++;
        //         }

        //         /******/
        //     }
            //std::swap(allKeypoints, newAllKeypoints);
            MyKeyPoint tmp1;
            for(int i=0;i<allkeypoints_num;i++){
                tmp1 =allKeypoints[i];
                allKeypoints[i]=newAllKeypoints[i];
                newAllKeypoints[i] = tmp1;
            }
        }

        nkeypoints = allkeypoints_num;


        {
            ICAngles(imagePyramid, layerInfo, allKeypoints, umax, halfPatchSize, nkeypoints);//TODO:改写ICAngles函数
        }

        for (i = 0; i < nkeypoints; i++)
        {
            float scale = layerScale[allKeypoints[i].octave];
            allKeypoints[i].x *= scale; 
            allKeypoints[i].y *=scale;   //关键点的坐标
        }
    }

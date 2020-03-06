#include "ORB.hpp"
void computeOrbDescriptors(xf::Mat<TYPE,IMAGEPYRAMID_HEIGHT,IMAGEPYRAMID_WIDTH,NPC1>& imagePyramid,xf::Rect layerInfo,
            float layerScale, MyKeyPoint keypoints[100],
            xf::Mat<XF_8UC1,100,32,XF_NPPC1>& descriptors,xf::Point _pattern[512], int dsize, int wta_k,int octave,int *valout)
{
        int step = (int)imagePyramid.rows;
        //int j, i, nkeypoints = keypoints[59].class_id;
        int j,i,nkeypoints=100;  //鐗瑰緛鐐规暟閲忎负100锟�??
#pragma HLS dataflow
        for (j = 0; j < nkeypoints; j++)
        {
            
            const MyKeyPoint kpt = keypoints[j];
            if(kpt.octave==octave){
            const xf::Rect layer = layerInfo;
            float scale = 1.f / layerScale;
            float angle = kpt.angle;

            angle *= (float)(XF_PI / 180.f);
            float a = (float)(hls::cos(angle)), b = (float)(hls::sin(angle));
            int tmp =imagePyramid.cols*(hls::round(kpt.y * scale) + layer.y)+(hls::round(kpt.x * scale) + layer.x);
            *valout = kpt.y * scale;
            //imagePyramid.data[tmp];
            float x, y;
            int ix, iy;
            xf::Point *pattern =_pattern;
            //uchar* desc = descriptors.ptr<uchar>(j); 绗琷锟�?????

#if 1
#define GET_VALUE(idx) \
               (x = pattern[idx].x*a - pattern[idx].y*b, \
                y = pattern[idx].x*b + pattern[idx].y*a, \
                ix = hls::round(x), \
                iy = hls::round(y), \
                (imagePyramid.data[tmp] + iy*step + ix) )
#endif

            if (wta_k == 2)
            {

                for (i = 0; i < dsize; ++i, pattern += 16)
                {
                    ap_uint<8> t0, t1, val;
                    t0 = GET_VALUE(0); t1 = GET_VALUE(1);
                    val = t0 < t1;
                    t0 = GET_VALUE(2); t1 = GET_VALUE(3);
                    val |= (t0 < t1) << 1;
                    t0 = GET_VALUE(4); t1 = GET_VALUE(5);
                    val |= (t0 < t1) << 2;
                    t0 = GET_VALUE(6); t1 = GET_VALUE(7);
                    val |= (t0 < t1) << 3;
                    t0 = GET_VALUE(8); t1 = GET_VALUE(9);
                    val |= (t0 < t1) << 4;
                    t0 = GET_VALUE(10); t1 = GET_VALUE(11);
                    val |= (t0 < t1) << 5;
                    t0 = GET_VALUE(12); t1 = GET_VALUE(13);
                    val |= (t0 < t1) << 6;
                    t0 = GET_VALUE(14); t1 = GET_VALUE(15);
                    val |= (t0 < t1) << 7;

                    descriptors.data[j*descriptors.cols+i]=(ap_uint<8>)val;
                   // desc[i] = (uchar)val;
                }
            }

#undef GET_VALUE
        }
        }
    }

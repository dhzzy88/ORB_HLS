#include "ORB.hpp"
void computeOrbDescriptors1(xf::Mat<TYPE,IMAGEPYRAMID_HEIGHT,IMAGEPYRAMID_WIDTH,NPC1>& imagePyramid,xf::Rect layerInfo[8],
            float layerScale[8], MyKeyPoint keypoints[60],
            xf::Mat<TYPE,HEIGHT,WIDTH,NPC1>& descriptors,xf::Point _pattern[50], int dsize, int wta_k)
    {
        int step = (int)imagePyramid.rows;
        int j, i, nkeypoints = keypoints[59].class_id;


        for (j = 0; j < nkeypoints; j++)
        {
            const MyKeyPoint kpt = keypoints[j];
            const xf::Rect layer = layerInfo[kpt.octave];
            float scale = 1.f / layerScale[kpt.octave];
            float angle = kpt.angle;

            angle *= (float)(XF_PI / 180.f);
            float a = (float)(hls::cos(angle)), b = (float)(hls::sin(angle));
            int tmp =(hls::round(kpt.y * scale) + layer.y)*(hls::round(kpt.x * scale) + layer.x);
             int center = imagePyramid.data[tmp];
            float x, y;
            int ix, iy;
            xf::Point *pattern =_pattern;
            //uchar* desc = descriptors.ptr<uchar>(j); 第j�???

#if 1
#define GET_VALUE(idx) \
               (x = pattern[idx].x*a - pattern[idx].y*b, \
                y = pattern[idx].x*b + pattern[idx].y*a, \
                ix = hls::round(x), \
                iy = hls::round(y), \
                (center + iy*step + ix) )
#endif

            if (wta_k == 2)
            {
                for (i = 0; i < dsize; ++i, pattern += 16)
                {
                    int t0, t1, val;
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
                    descriptors.data[j][i]=val;
                   // desc[i] = (uchar)val;
                }
            }
            else if (wta_k == 3)
            {
                for (i = 0; i < dsize; ++i, pattern += 12)
                {
                    int t0, t1, t2, val;
                    t0 = GET_VALUE(0); t1 = GET_VALUE(1); t2 = GET_VALUE(2);
                    val = t2 > t1 ? (t2 > t0 ? 2 : 0) : (t1 > t0);

                    t0 = GET_VALUE(3); t1 = GET_VALUE(4); t2 = GET_VALUE(5);
                    val |= (t2 > t1 ? (t2 > t0 ? 2 : 0) : (t1 > t0)) << 2;

                    t0 = GET_VALUE(6); t1 = GET_VALUE(7); t2 = GET_VALUE(8);
                    val |= (t2 > t1 ? (t2 > t0 ? 2 : 0) : (t1 > t0)) << 4;

                    t0 = GET_VALUE(9); t1 = GET_VALUE(10); t2 = GET_VALUE(11);
                    val |= (t2 > t1 ? (t2 > t0 ? 2 : 0) : (t1 > t0)) << 6;

                   // desc[i] = (uchar)val;
                   descriptors.data[j][i]=val;
                }
            }
            else if (wta_k == 4)
            {
                for (i = 0; i < dsize; ++i, pattern += 16)
                {
                    int t0,t1,t2,t3,u,v,k,val;
                    t0 = GET_VALUE(0); t1 = GET_VALUE(1);
                    t2 = GET_VALUE(2); t3 = GET_VALUE(3);
                    u = 0, v = 2;
                    if (t1 > t0) t0 = t1, u = 1;
                    if (t3 > t2) t2 = t3, v = 3;
                    k = t0 > t2 ? u : v;
                    val = k;

                    t0 = GET_VALUE(4); t1 = GET_VALUE(5);
                    t2 = GET_VALUE(6); t3 = GET_VALUE(7);
                    u = 0, v = 2;
                    if (t1 > t0) t0 = t1, u = 1;
                    if (t3 > t2) t2 = t3, v = 3;
                    k = t0 > t2 ? u : v;
                    val |= k << 2;

                    t0 = GET_VALUE(8); t1 = GET_VALUE(9);
                    t2 = GET_VALUE(10); t3 = GET_VALUE(11);
                    u = 0,v = 2;
                    if (t1 > t0) t0 = t1, u = 1;
                    if (t3 > t2) t2 = t3, v = 3;
                    k = t0 > t2 ? u : v;
                    val |= k << 4;

                    t0 = GET_VALUE(12); t1 = GET_VALUE(13);
                    t2 = GET_VALUE(14); t3 = GET_VALUE(15);
                    u = 0, v = 2;
                    if (t1 > t0) t0 = t1, u = 1;
                    if (t3 > t2) t2 = t3, v = 3;
                    k = t0 > t2 ? u : v;
                    val |= k << 6;
                    descriptors.data[j][i]=val;
                    //desc[i] = (uchar)val;
                }
            }
#undef GET_VALUE
        }
    }

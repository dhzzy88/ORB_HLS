#include "ORB.hpp"
void ICAngles(xf::Mat<TYPE,IMAGEPYRAMID_HEIGHT, IMAGEPYRAMID_WIDTH, NPC1>& img, const xf::Rect layerinfo[8],
        MyKeyPoint pts[240], int u_max[17], int half_k,int keypoints_num)
    {
        //int step = (int)img.step1();
        int step =1;  //通道数为1�?
        size_t ptidx, ptsize = keypoints_num;

        for (ptidx = 0; ptidx < ptsize; ptidx++)
        {
            const xf::Rect& layer = layerinfo[pts[ptidx].octave];
            const ap_uint<8> center = img.data[(pts[ptidx].y + layer.y)*img.cols+(pts[ptidx].x + layer.x)];

            int m_01 = 0, m_10 = 0;

            // Treat the center line differently, v=0
            for (int u = -half_k; u <= half_k; ++u)
                m_10 += u * center[u];

            // Go line by line in the circular patch
            for (int v = 1; v <= half_k; ++v)
            {
                // Proceed over the two lines
                int v_sum = 0;
                int d = u_max[v];
                for (int u = -d; u <= d; ++u)
                {
                    int val_plus = center[u + v * step], val_minus = center[u - v * step];
                    v_sum += (val_plus - val_minus);
                    m_10 += u * (val_plus + val_minus);
                }
                m_01 += v * v_sum;
            }

            pts[ptidx].angle = fastAtan2((float)m_01, (float)m_10);
        }
    }
#include "ORB.hpp"

void  HarrisResponses(const xf::Mat<TYPE,IMAGEPYRAMID_HEIGHT, IMAGEPYRAMID_WIDTH, NPC1>& img, const xf::Rect layerinfo[8],MyKeyPoint pts[240], int blockSize, float harris_k,int keypoints_num)
    {
        size_t ptidx, ptsize = keypoints_num;
    
        const ap_uint<8>* ptr00 = img.data;
        int step = (int)(img.cols /1); //用cols代替step  因为是gray图片所以使用1代替img.elemSize1()
        int r = blockSize / 2;

        float scale = 1.f / ((1 << 2) * blockSize * 255.f);
        float scale_sq_sq = scale * scale * scale * scale;

        //AutoBuffer<int> ofsbuf(blockSize * blockSize);
        int ofsbuf_num=0;
        int ofsbuf[100]; //需要49，末尾作为哨兵
        int* ofs = ofsbuf;
        for (int i = 0; i < blockSize; i++)
            for (int j = 0; j < blockSize; j++)
                ofs[i * blockSize + j] = (int)(i * step + j);
                ofsbuf_num++;

        for (ptidx = 0; ptidx < ptsize; ptidx++)
        {
            int x0 = (pts[ptidx].x);
            int y0 = (pts[ptidx].y);
            int z = pts[ptidx].octave;

            const ap_uint<8>* ptr0 = ptr00 + (y0 - r + layerinfo[z].y) * step + x0 - r + layerinfo[z].x;
            int a = 0, b = 0, c = 0;

            for (int k = 0; k < blockSize * blockSize; k++)
            {
                const ap_uint<8>* ptr = ptr0 + ofs[k];
                int Ix = (ptr[1] - ptr[-1]) * 2 + (ptr[-step + 1] - ptr[-step - 1]) + (ptr[step + 1] - ptr[step - 1]);
                int Iy = (ptr[step] - ptr[-step]) * 2 + (ptr[step - 1] - ptr[-step - 1]) + (ptr[step + 1] - ptr[-step + 1]);
                a += Ix * Ix;
                b += Iy * Iy;
                c += Ix * Iy;
            }
            pts[ptidx].response = ((float)a * b - (float)c * c -
                harris_k * ((float)a + b) * ((float)a + b)) * scale_sq_sq;
        }
    }

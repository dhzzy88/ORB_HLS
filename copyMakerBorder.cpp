//#include "ORB.hpp"
#include "common/xf_structs.h"
#include "common/xf_types.h"
#define STEP  3072  //分辨率为3000 
#define CV_CN_SHIFT   3
#define CV_DEPTH_MAX  (1 << CV_CN_SHIFT)
#define CV_MAT_DEPTH_MASK       (CV_DEPTH_MAX - 1)
#define CV_MAT_DEPTH(flags)     ((flags) & CV_MAT_DEPTH_MASK)
#define CV_MAKETYPE(depth,cn) (CV_MAT_DEPTH(depth) + (((cn)-1) << CV_CN_SHIFT))
//void copyMakeBorder(xf::Mat<TYPE,HEIGHT,WIDTH,NPC1> image,xf::Mat<TYPE,HEIGHT,WIDTH,NPC1> extImg,int border,int border1,int border2,int border3,int L=4){



}
template <typename T> static inline
void scalarToRawData_(const xf::Scalar& s, T * const buf, const int cn, const int unroll_to)
{
    int i = 0;
    for(; i < cn; i++)
        buf[i] = saturate_cast<T>(s.val[i]);
    for(; i < unroll_to; i++)
        buf[i] = buf[i-cn];
}

void scalarToRawData(const xf::Scalar& s, void* _buf, int type, int unroll_to)
{
    

    const int depth = type, cn = type;
    
    switch(depth)
    {
    case XF_8UC1:
        scalarToRawData_<ap_uint<8>>(s, (ap_uint<8>*)_buf, cn, unroll_to);
        break;
    }
}
int borderInterpolate( int p, int len, int borderType )
{
    if( (unsigned)p < (unsigned)len )
        ;
    else if( borderType == BORDER_REPLICATE )
        p = p < 0 ? 0 : len - 1;
    else if( borderType == BORDER_REFLECT || borderType == BORDER_REFLECT_101 )
    {
        int delta = borderType == BORDER_REFLECT_101;
        if( len == 1 )
            return 0;
        do
        {
            if( p < 0 )
                p = -p - 1 + delta;
            else
                p = len - 1 - (p - len) - delta;
        }
        while( (unsigned)p >= (unsigned)len );
    }
    else if( borderType == BORDER_WRAP )
    {
        if( p < 0 )
            p -= ((p-len+1)/len)*len;
        if( p >= len )
            p %= len;
    }
    else if( borderType == BORDER_CONSTANT )
        p = -1;
    else
        return;
    return p;
}

void copyMakeBorder_8u( const ap_uint<8>* src, size_t srcstep, xf::Size srcroi,
                        ap_uint<8>* dst, size_t dststep, xf::Size dstroi,
                        int top, int left, int cn, int borderType )
{
    const int isz = (int)sizeof(int);
    int i, j, k, elemSize = 1;
    bool intMode = false;

    if( (cn | srcstep | dststep | (size_t)src | (size_t)dst) % isz == 0 )
    {
        cn /= isz;
        elemSize = isz;
        intMode = true;
    }

    int _tab[(dstroi.width - srcroi.width)*cn];
    int* tab = _tab;
    int right = dstroi.width - srcroi.width - left;
    int bottom = dstroi.height - srcroi.height - top;

    for( i = 0; i < left; i++ )
    {
        j = borderInterpolate(i - left, srcroi.width, borderType)*cn;
        for( k = 0; k < cn; k++ )
            tab[i*cn + k] = j + k;
    }

    for( i = 0; i < right; i++ )
    {
        j = borderInterpolate(srcroi.width + i, srcroi.width, borderType)*cn;
        for( k = 0; k < cn; k++ )
            tab[(i+left)*cn + k] = j + k;
    }

    srcroi.width *= cn;
    dstroi.width *= cn;
    left *= cn;
    right *= cn;

    ap_uint<8>* dstInner = dst + dststep*top + left*elemSize;

    for( i = 0; i < srcroi.height; i++, dstInner += dststep, src += srcstep )
    {
        if( dstInner != src )
            memcpy(dstInner, src, srcroi.width*elemSize);

        if( intMode )
        {
            const int* isrc = (int*)src;
            int* idstInner = (int*)dstInner;
            for( j = 0; j < left; j++ )
                idstInner[j - left] = isrc[tab[j]];
            for( j = 0; j < right; j++ )
                idstInner[j + srcroi.width] = isrc[tab[j + left]];
        }
        else
        {
            for( j = 0; j < left; j++ )
                dstInner[j - left] = src[tab[j]];
            for( j = 0; j < right; j++ )
                dstInner[j + srcroi.width] = src[tab[j + left]];
        }
    }

    dstroi.width *= elemSize;
    dst += dststep*top;

    for( i = 0; i < top; i++ )
    {
        j = borderInterpolate(i - top, srcroi.height, borderType);
        memcpy(dst + (i - top)*dststep, dst + j*dststep, dstroi.width);
    }

    for( i = 0; i < bottom; i++ )
    {
        j = borderInterpolate(i + srcroi.height, srcroi.height, borderType);
        memcpy(dst + (i + srcroi.height)*dststep, dst + j*dststep, dstroi.width);
    }
}
void copyMakeConstBorder_8u( const ap_uint<8>* src, size_t srcstep, xf::Size srcroi,
                             ap_uint<8>* dst, size_t dststep, xf::Size dstroi,
                             int top, int left, int cn, const ap_uint<8>* value )
{
    int i, j;
    
    ap_uint<8> _constBuf[8];
    ap_uint<8>* constBuf = _constBuf;
    int right = dstroi.width - srcroi.width - left;
    int bottom = dstroi.height - srcroi.height - top;

    for( i = 0; i < dstroi.width; i++ )
    {
        for( j = 0; j < cn; j++ )
            constBuf[i*cn + j] = value[j];
    }

    srcroi.width *= cn;
    dstroi.width *= cn;
    left *= cn;
    right *= cn;

    ap_uint<8>* dstInner = dst + dststep*top + left;

    for( i = 0; i < srcroi.height; i++, dstInner += dststep, src += srcstep )
    {
        if( dstInner != src )
            memcpy( dstInner, src, srcroi.width );
        memcpy( dstInner - left, constBuf, left );
        memcpy( dstInner + srcroi.width, constBuf, right );
    }

    dst += dststep*top;

    for( i = 0; i < top; i++ )
        memcpy(dst + (i - top)*dststep, constBuf, dstroi.width);

    for( i = 0; i < bottom; i++ )
        memcpy(dst + (i + srcroi.height)*dststep, constBuf, dstroi.width);
}
void Mat::locateROI( Size& wholeSize, Point& ofs ) const
{
    CV_Assert( dims <= 2 && step[0] > 0 );
    size_t esz = elemSize(), minstep;
    ptrdiff_t delta1 = data - datastart, delta2 = dataend - datastart;

    if( delta1 == 0 )
        ofs.x = ofs.y = 0;
    else
    {
        ofs.y = (int)(delta1/step[0]);
        ofs.x = (int)((delta1 - step[0]*ofs.y)/esz);
        CV_DbgAssert( data == datastart + ofs.y*step[0] + ofs.x*esz );
    }
    minstep = (ofs.x + cols)*esz;
    wholeSize.height = (int)((delta2 - minstep)/step[0] + 1);
    wholeSize.height = std::max(wholeSize.height, ofs.y + rows);
    wholeSize.width = (int)((delta2 - step*(wholeSize.height-1))/esz);
    wholeSize.width = std::max(wholeSize.width, ofs.x + cols);
}

adjustROI(xf::Mat<TYPE,HEIGHT,WIDTH,NPC1>& _SRC,int dtop, int dbottom, int dleft, int dright )
{
    Size wholeSize; Point ofs;
    size_t esz = elemSize();
    locateROI( wholeSize, ofs );
    int row1 = std::min(std::max(ofs.y - dtop, 0), wholeSize.height), row2 = std::max(0, std::min(ofs.y + rows + dbottom, wholeSize.height));
    int col1 = std::min(std::max(ofs.x - dleft, 0), wholeSize.width), col2 = std::max(0, std::min(ofs.x + cols + dright, wholeSize.width));
    if(row1 > row2)
        std::swap(row1, row2);
    if(col1 > col2)
        std::swap(col1, col2);

    data += (row1 - ofs.y)*step + (col1 - ofs.x)*esz;
    rows = row2 - row1; cols = col2 - col1;
    size.p[0] = rows; size.p[1] = cols;
    if( esz*cols == step[0] || rows == 1 )
        flags |= CONTINUOUS_FLAG;
    else
        flags &= ~CONTINUOUS_FLAG;
    return *this;
}


void copyMakeBorder( xf::Mat<TYPE,HEIGHT,WIDTH,NPC1> _src, xf::Mat<TYPE,HEIGHT,WIDTH,NPC1> &_dst, int top, int bottom,
                         int left, int right, int borderType, xf::Scalar &value )
{
    xf::Mat src = _src;
    int type = TYPE;

    if((borderType & BORDER_ISOLATED) == 0 )
    {
        xf::Size wholeSize;
        xf::Point ofs;
        src.locateROI(wholeSize, ofs);
        int dtop = std::min(ofs.y, top);
        int dbottom = std::min(wholeSize.height - src.rows - ofs.y, bottom);
        int dleft = std::min(ofs.x, left);
        int dright = std::min(wholeSize.width - src.cols - ofs.x, right);
        src.adjustROI(dtop, dbottom, dleft, dright);
        top -= dtop;
        left -= dleft;
        bottom -= dbottom;
        right -= dright;
    }
    

    _dst.rows =src.rows + top + bottom;
    _dst.cols =src.cols + left + right, type;
    xf::Mat<TYPE,HEIGHT,WIDTH,NPC1>& dst = _dst;

    if(top == 0 && left == 0 && bottom == 0 && right == 0)
    {
        if(src.data != dst.data)// || src.step != dst.step) 
            src.copyTo(_dst);
        return;
    }

    borderType &= ~BORDER_ISOLATED;
    xf::Size tmp(src.cols,src.rows);
    if( borderType != BORDER_CONSTANT ){
        copyMakeBorder_8u( src.data, STEP, tmp,
                           dst.data, STEP, xf::Size(dst.rows,dst.cols),
                           top, left, 1, borderType );
    }else
    {
        int cn = 1, cn1 = cn;
        double buf[8];//最后一个哨兵
        if( cn > 4 )
        {
            cn1 = 1;
        }
        scalarToRawData(value, buf, CV_MAKETYPE(XF_8UC1, cn1), cn);
        copyMakeConstBorder_8u( src.data, STEP, xf::Size(src.cols,src.rows),
                                dst.data, STEP, xf::Size(dst.cols,dst.rows),
                                top, left, (int)sizeof(ap_uint<8>), (ap_uint<8>*)(double*)buf );
    }
}
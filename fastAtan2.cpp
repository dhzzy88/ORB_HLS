#include "ORB.hpp";

float fastAtan2( float y, float x )
{    
    static const float atan2_p1 = 0.9997878412794807f*(float)(180/XF_PI);
    static const float atan2_p3 = -0.3258083974640975f*(float)(180/XF_PI);
    static const float atan2_p5 = 0.1555786518463281f*(float)(180/XF_PI);
    static const float atan2_p7 = -0.04432655554792128f*(float)(180/XF_PI);
    float ax = hls::abs(x), ay = hls::abs(y);//首先不分象限，求得一个锐角角度
    float a, c, c2;
    if( ax >= ay )    
    {       
         c = ay/(ax + (float)DBL_EPSILON);
         c2 = c*c; 
         a = (((atan2_p7*c2 + atan2_p5)*c2 + atan2_p3)*c2 + atan2_p1)*c;
     }   
      else  
    {        c = ax/(ay + (float)DBL_EPSILON);
             c2 = c*c;
             a = 90.f - (((atan2_p7*c2 + atan2_p5)*c2 + atan2_p3)*c2 + atan2_p1)*c; 
        }    
    if( x < 0 )//锐角求出后，根据x和y的正负性确定向量的方向，即角度。  
         a = 180.f - a;    
    if( y < 0 )    
         a = 360.f - a;   
    return a;
}
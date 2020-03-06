## ORB_HLS

opencv_ORB2xilinx_hls


1、使用xilinx vivado_hls高级综合工具实现orb算法，目前c_testbench仿真通过，可综合的功能函数computeOrbDescriptors2.cpp,对应的test bench文件是computeOrbDescriptors_tb.cpp，在opencv中直接是对图像金字塔构成的图像矩阵进行描述子的计算编码，在computeOrbDescriptors2.cpp将图像金字塔的传入，仅计算了第一层的描述子，可以通过多次调用实现金字塔所有层数的描述子编码，但是由于hls和c++基本数据类型在高级综合中的过程中会有精度损失，因此，不能完全获得和opencv中完全相同的描述子。</br>
![avatar](https://github.com/dhzzy88/ORB_HLS/blob/master/out.bmp)仿真得到图像金字塔第一层的描述子矩阵（opencv中使用32*100的Mat矩阵来存储）</br>

![avatar](https://github.com/dhzzy88/ORB_HLS/blob/master/first.bmp)从opencv中获得的描述子矩阵</br>

![avatar](https://github.com/dhzzy88/ORB_HLS/blob/master/descriptors2%E7%BB%BC%E5%90%88.png)</br>高级综合资源，没有对接口interface的编译定义，后续根据ORB.cpp的完成情况，来确定接口定义。

2、在opencv中所有的特征点的存储，图像金字塔的管理全部通过动态内存管理的方式完成，而在hls高级综合中完全不能使用动态内存的方式来管理内存的，必须在高级综合前把所有的动态内存管理的代码转换成固定长度的数组来保证在综合阶段成功，opencv中使用的部分代码使用了opencv的数学函数，使用hls::math来完成相应的操作。</br>

3、把整个orb代码的使用hls实现的过程中有部分函数功能难以实现，比如在金字塔构建过程中的copyMakeBorder，在该函数内部使用了多个opencv内部定义的函数。</br>

# note:
1、在寻找特征点的函数中使用ICAngles.cpp，HarrisResponses.cpp两个模块不准备单独编写test bench文件，在computerOrbKeyPoint.cpp中会使用到两个小模块，直接test_bench computerOrbKeyPoint.cpp(test_bench文件是computerOrbKeyPoint_tb.cpp)目前没有完全通过仿真。正在查找和修改问题。

2、在之前的工作中只替换了fast等一些少数函数，在硬件上实现的功能有限，暂且搁置了，但是有一些需要note：使用VDMA传送图像，在linux下驱动程序的关键点是对于图像在内存中的物理地址需要传送给vdma，而linux中使用虚拟地址，读取图像内存地址的得到的是虚拟地址，dma_alloc_coherent函数应该可以解决内存转换问题。

3、对于相机等其他外来图像源使用V4L2框架解决zynq的图像传送。（但是以上部分应当在hls模块完成后将ip加入原理图后进行，hls会生成部分驱动函数。）
# TODO:
1、接下来需要在ORB.cpp中具体实现各个部分的耦合，由于部分函数功能难以实现，正在寻找解决办法。（copyMakeBorder难以解决）
2、继续完成computerOrbKeyPoint.cpp,将功能逻辑重新对照opencv算法来实现。

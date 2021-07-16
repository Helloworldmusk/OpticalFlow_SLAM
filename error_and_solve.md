## 编译错误

![image-20210713165832604](typora_image/image-20210713165832604.png)

原因分析： 文件命名中间多了一个空格；



**找不到头文件**

![image-20210715172339236](typora_image/image-20210715172339236.png)

问题描述：

​	由于主程序在 app文件夹下，而库文件在algorithm 文件夹下，编译库文件的时候，找不到algorithm 文件夹下面相应的头文件。

原因分析：找不到头文件，说明头文件路径不对

解决办法：include_directories(${PROJECT_SOURCE_DIR})



**error: ‘Optimizer’ was not declared in this scope** 

![image-20210715194904325](typora_image/image-20210715194904325.png)



问题描述：  两个头文件中的类互相有指针引用，编译时出错，这时需要前置声明两个类的定义；

![image-20210715195210606](typora_image/image-20210715195210606.png)

![image-20210715195233059](typora_image/image-20210715195233059.png)





编译错误：

××× 未定义的引用 

![image-20210716122844869](typora_image/image-20210716122844869.png)

原因分析： 可能是库没链接上；
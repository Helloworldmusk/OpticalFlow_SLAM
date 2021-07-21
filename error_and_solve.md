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



编译错误：

报错信息：

/usr/local/lib/libgflags.a(gflags.cc.o): relocation R_X86_64_PC32 against symbol `stderr@@GLIBC_2.2.5' can not be used when making a shared object; recompile with -fPIC

最后的链结失败: 错误的值

![image-20210719154247375](typora_image/image-20210719154247375.png)

原因分析：之前编译gflags的时候，没有给定编译说明编译成静态库还是动态库，所以在工程中按照动态库进行链接的时候，出现了这个错误（第一次安装完成后，并没有出现这个错误，关键后再开机，就出现了这个问题）

解决办法：应该在编译的时候给定编译结果为动态库还是静态库，cmake .. -DBUILD_SHARED_LIBS=ON

参考链接： https://stackoverflow.com/questions/45691778/cdt-using-lib-a-relocation-r-x86-64-32s-against-symbol-can-not-be-used

​					https://github.com/gflags/gflags/blob/master/INSTALL.md





多线程运行错误：

Thread 1 "run_vo" received signal SIGABRT, Aborted.
__GI_raise (sig=sig@entry=6) at ../sysdeps/unix/sysv/linux/raise.c:51
51	../sysdeps/unix/sysv/linux/raise.c: 没有那个文件或目录.

![image-20210719165105978](typora_image/image-20210719165105978.png)

​				

原因： 对于创建的两个线程，在主线程中没有使用join() 进行阻塞，导致主线程提前结束了，



编译错误：

error: extra qualification ‘OpticalFlow_SLAM_algorithm_opticalflow_slam::Tracker::’ on member ‘set_front_end_status’ [-fpermissive]

bool Tracker::set_front_end_status(const FrontEndStatus new_status);

![image-20210721094026402](typora_image/image-20210721094026402.png)



原因：        bool Tracker::set_front_end_status(const FrontEndStatus new_status);在类内声明的时候，不应该加入该类的作用域限定符；

解决办法： 删除Tracker::即可；





运行错误：terminate called without an active exception

![image-20210721173733000](typora_image/image-20210721173733000.png)

子线程还没有结束的时候，主线程却提前结束了；对任务线程在主线程中加join即可；

https://blog.csdn.net/github_20066005/article/details/79999530



运行异常：

通过主线程调用tarcker的 notify 去通知 viewer 线程 和 optimizer 线程进行终止， 然后去调用 viewer 和 optimizer  的.join阻塞等待，发现系统卡在了这里，不动了；

![image-20210721192447351](typora_image/image-20210721192447351.png)

原因分析：还是两个线程没有彻底关闭；

解决方案：调用detach()而不是join()；

```cpp
void Optimizer::stop()
{
        //TODO(snowden) : directly detach thread , if this thread will be terminated ? 
        back_end_thread_.detach();
}
```


### 工厂模式

通俗的说，工厂模式就是设计一个同意的订货接口，然后让一个工厂类去实现这个接口，用户只需要告诉工厂类需要什么样的产品，工厂类根据用于需求，产生不同的商品；

![image-20210716160717690](typora_image/image-20210716160717690.png)

参考：https://www.runoob.com/design-pattern/factory-pattern.html

### 单例模式

```cpp
/*饿汉模式
* 缺点： 只能使用默认的构造函数，无法使用外部参数对其进行初始化；
* 优点： 由于在创建对象的时候就进行了对象的定义，所以不存在多线程竞争的影响；
*/
public class Singleton {  
    private static Singleton instance = new Singleton();  
    private Singleton (){}  
    public static Singleton getInstance() {  
    return instance;  
    }  
}

/** 懒汉模式（饱汉模式) 之 双重检测锁模式；
* 缺点： 加锁会在一定程度上影响性能；
* 优点：线程安全，且相对于直接加锁的方式，双重检测模式效率更高。
*/
public class Singleton {  
    private volatile static Singleton singleton;  
    private Singleton (){}  
    public static Singleton getSingleton() {  
    if (singleton == null) {  
        synchronized (Singleton.class) {  
        if (singleton == null) {  
            singleton = new Singleton();  
        }  
        }  
    }  
    return singleton;  
    }  
}
```

参考链接：https://www.runoob.com/design-pattern/singleton-pattern.html

### 静态成员变量需要在类外进行初始化

初始化的时候需要使用作用域限定符和 具体类型；

```cpp
class sample{
   int var;
   static int count;
  
   public:
};
int sample::count = 0;                 //static variable initialisation
int main()
{

}
```



### 使用=delete 去 disable 一些函数；

> 目标： disable 掉一些运算操作 或者 函数；

> 使用时机： 在首次申明这个函数的时候，就要使用= delete进行设置

该特性是在C++11才引入的，所以之前的版本是无法使用的；

```cpp
	class B{
public:
  B(int){}                
  B(double) = delete;   // Declare the conversioin constructor as a deleted function
};

int main(){
  B b1(1);        
  B b2(100.1);          // Error, conversion from double to class B is disabled.
}
```

```cpp
class C {
public:  
  C();
};

C::C() = delete;    // Error, the deleted definition of function C must be
                    // the first declaration of the function.
```

参考链接：https://www.ibm.com/docs/en/i/7.3?topic=definitions-deleted-functions-c11

### CmakeLists.txt 中将某一文件夹下的所有文件当做源文件的方法；

- 下面语句将会把./src目录下所有符合*.cpp结尾的文件存入USER_LIBS_PATH变量中，

```cmake
file(GLOB USER_LIBS_PATH ./src/*.cpp)
1
```

*如果我们不但在当前目录需要引入，还需要在当前目录子目录引入了，这里就直接使用GLOB_RECURSE*

```cmake
file(GLOB_RECURSE USER_LIBS_PATH ./src/*.cpp)
```

```cmake
 file( GLOB ALGORITHM_SRC */src/*.cc)
message(" ALGORITHM_SRC : " ${ALGORITHM_SRC})
add_library(op_slam SHARED 
        ${ALGORITHM_SRC}
        )
```

参考链接：https://blog.csdn.net/qq_31261509/article/details/88692736

​				   http://www.voidcn.com/article/p-urttzyjn-buo.html



### GLOG 

等级：

INFO, WARNING,ERROR,FATAL

可配置参数 Setting Flags：

`logtostderr` *(*`bool`*, default=*`false`*)*

`stderrthreshold` *(*`int`*, default=2, which is* `ERROR`*)*

`minloglevel` *(*`int`*, default=0, which is* `INFO`*)*

`v` *(*`int`*, default=0)*

**note**: 这些参数直接配合 gflags 库使用会更好，可以运行命令中加入上述参数，进行设置；具体参考

[链接](https://github.com/google/glog#severity-levels)

使用案例：

```cpp
#include <glog/logging.h>

int main(int argc, char* argv[]) {
    // Initialize Google’s logging library.
    google::InitGoogleLogging(argv[0]);

    // ...
    LOG(INFO) << "Found " << num_cookies << " cookies";
    DLOG(INFO) << "Found cookies";
	DLOG_IF(INFO, num_cookies > 10) << "Got lots of cookies";
    CHECK(fp->Write(x) == 4) << "Write failed!";
    CHECK_NE(1, 2) << ": The world must be ending!";
    CHECK_EQ(string("abc")[1], ’b’);
    CHECK_EQ(some_ptr, static_cast<SomeType*>(NULL));
    CHECK_NOTNULL(some_ptr);
}
```

参考链接： https://github.com/google/glog#severity-levels

​					https://www.jianshu.com/p/2179938a818d

​					http://www.yeolar.com/note/2014/12/14/gflags/



### std::thread



参考链接： https://www.cnblogs.com/adorkable/p/12722209.html






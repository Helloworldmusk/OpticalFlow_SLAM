### 工厂模式

通俗的说，工厂模式就是设计一个统一的订货接口，然后让一个工厂类去实现这个接口，用户只需要告诉工厂类需要什么样的产品，工厂类根据用于需求，产生不同的商品；

![image-20210716160717690](typora_image/image-20210716160717690.png)

参考：https://www.runoob.com/design-pattern/factory-pattern.html

### 单例模式

```cpp
/*饿汉模式  初始化的时候，直接就创建了对象；
* 缺点： 只能使用默认的构造函数，无法使用外部参数对其进行初始化；
* 优点： 由于在创建类的时候就进行了对象的定义，所以不存在多线程竞争的影响；
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
    LOG_INFO << "Found " << num_cookies << " cookies";
    DLOG_INFO << "Found cookies";
	DLOG_if (INFO, num_cookies > 10) << "Got lots of cookies";
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

**头文件** 

#include <thread>

**线程的创建**

```cpp
int main()
{
    int n = 0;
    std::thread t1; // t1 is not a thread
    std::thread t2(f1, n + 1); // pass by value
    std::thread t3(f2, std::ref(n)); // pass by reference
    std::thread t4(std::move(t3)); // t4 is now running f2(). t3 is no longer a thread
    t2.join();
    t4.join();
    std::cout << "Final value of n is " << n << '\n';
}
```

**线程相关其他函数**

```cpp
//get_id
  std::thread t2(foo);
  std::thread::id t2_id = t2.get_id();
std::this_thread::get_id();

//joinable
//检查线程是否可被 join。检查当前的线程对象是否表示了一个活动的执行线程，由默认构造函数创建的线程是不能被 join 的。
void foo()
{
  std::this_thread::sleep_for(std::chrono::seconds(1));
}
int main()
{
  std::thread t;
  std::cout << "before starting, joinable: " << t.joinable() << '\n';

  t = std::thread(foo);
  std::cout << "after starting, joinable: " << t.joinable() << '\n';

  t.join();
}

//detach
/*
Detach 线程。 将当前线程对象所代表的执行实例与该线程对象分离，使得线程的执行可以单独进行。一旦线程执行完毕，它所分配的资源将会被释放。
调用 detach 函数之后：
1. *this 不再代表任何的线程执行实例。
2. joinable() == false
3. get_id() == std::thread::id()
*/
std::thread t(independentThread);
t.detach();

//swap: 
//Swap 线程，交换两个线程对象所代表的底层句柄(underlying handles)。
  std::thread t1(foo);
  std::thread t2(bar);
  std::swap(t1, t2);
  t1.swap(t2);


//pthread_setschedparam 
//设置线程的优先级；

  std::thread t1(f, 1), t2(f, 2);
  sched_param sch;
  int policy; 
  pthread_getschedparam(t1.native_handle(), &policy, &sch);
  sch.sched_priority = 20;
  if (pthread_setschedparam(t1.native_handle(), SCHED_FIFO, &sch)) {
      std::cout << "Failed to setschedparam: " << std::strerror(errno) << '\n';
  }

// hardware_concurrency 
// 检测硬件并发特性，返回当前平台的线程实现所支持的线程并发数目，但返回值仅仅只作为系统提示(hint)。
#include <iostream>
#include <thread>
 
int main() {
    unsigned int n = std::thread::hardware_concurrency();
    std::cout << n << " concurrent threads are supported.\n";
}

//多线程加锁
std::mutex g_display_mutex;
void foo()
{
  std::thread::id this_id = std::this_thread::get_id();
  g_display_mutex.lock();
  std::cout << "thread " << this_id << " sleeping...\n";
  g_display_mutex.unlock();
  std::this_thread::sleep_for(std::chrono::seconds(1));
}
int main()
{
  std::thread t1(foo);
  std::thread t2(foo);
  t1.join();
  t2.join();
}

//yield 
//当前线程放弃执行，通知操作系统调度另一个线程继续执行；
void little_sleep(std::chrono::microseconds us)
{
  auto start = std::chrono::high_resolution_clock::now();
  auto end = start + us;
  do {
      std::this_thread::yield();
  } while (std::chrono::high_resolution_clock::now() < end);
}

//sleep_until
// 线程休眠至某个指定的时刻(time point)，该线程才被重新唤醒。
template< class Clock, class Duration >
void sleep_until( const std::chrono::time_point<Clock,Duration>& sleep_time );

//sleep_for
//线程休眠某个指定的时间片(time span)，该线程才被重新唤醒，不过由于线程调度等原因，实际休眠时间可能比 sleep_duration 所表示的时间片更长。
int main()
{
  std::cout << "Hello waiter" << std::endl;
  std::chrono::milliseconds dura( 2000 );
  std::this_thread::sleep_for( dura );
  std::cout << "Waited 2000 ms\n";
}

```

**note**

使用线程库之前，在CMakelists.txt中，首先要进行链接： target_link_libraries(project_name  pthread)

参考链接： https://www.cnblogs.com/adorkable/p/12722209.html

​					https://www.runoob.com/w3cnote/cpp-std-thread.html



### std::mutex

常用函数：

| [lock](https://en.cppreference.com/w/cpp/thread/mutex/lock)  | locks the mutex, blocks if the mutex is not available (public member function) |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| [try_lock](https://en.cppreference.com/w/cpp/thread/mutex/try_lock) | tries to lock the mutex, returns if the mutex is not available (public member function) |
| [unlock](https://en.cppreference.com/w/cpp/thread/mutex/unlock) | unlocks the mutex (public member function)                   |

 但是不经常使用std::mutex

>`std::mutex` is usually not accessed directly: [std::unique_lock](https://en.cppreference.com/w/cpp/thread/unique_lock), [std::lock_guard](https://en.cppreference.com/w/cpp/thread/lock_guard), or [std::scoped_lock](https://en.cppreference.com/w/cpp/thread/scoped_lock) (since C++17) manage locking in a more exception-safe manner.

```示例代码
#include <iostream>
#include <map>
#include <string>
#include <chrono>
#include <thread>
#include <mutex>
 
std::map<std::string, std::string> g_pages;
std::mutex g_pages_mutex;
 
void save_page(const std::string &url)
{
    // simulate a long page fetch
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::string result = "fake content";
 
    std::lock_guard<std::mutex> guard(g_pages_mutex);
    g_pages[url] = result;
}
 
int main() 
{
    std::thread t1(save_page, "http://foo");
    std::thread t2(save_page, "http://bar");
    t1.join();
    t2.join();
 
    // safe to access g_pages without lock now, as the threads are joined
    for (const auto &pair : g_pages) {
        std::cout << pair.first << " => " << pair.second << '\n';
    }
}
```



### std::unique_lock

**为什么不用std::mutex , 而要用 std::unique_lock?**

> The class unique_lock is a general-purpose mutex ownership wrapper allowing deferred locking, time-constrained attempts at locking, recursive locking, transfer of lock ownership, and use with condition variables.



**常用函数**

| [lock](https://en.cppreference.com/w/cpp/thread/unique_lock/lock) | locks (i.e., takes ownership of) the associated mutex (public member function) |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| [try_lock](https://en.cppreference.com/w/cpp/thread/unique_lock/try_lock) | tries to lock (i.e., takes ownership of) the associated mutex without blocking (public member function) |
| [try_lock_for](https://en.cppreference.com/w/cpp/thread/unique_lock/try_lock_for) | attempts to lock (i.e., takes ownership of) the associated [*TimedLockable*](https://en.cppreference.com/w/cpp/named_req/TimedLockable) mutex, returns if the mutex has been unavailable for the specified time duration (public member function) |
| [try_lock_until](https://en.cppreference.com/w/cpp/thread/unique_lock/try_lock_until) | tries to lock (i.e., takes ownership of) the associated [*TimedLockable*](https://en.cppreference.com/w/cpp/named_req/TimedLockable) mutex, returns if the mutex has been unavailable until specified time point has been reached (public member function) |
| [unlock](https://en.cppreference.com/w/cpp/thread/unique_lock/unlock) | unlocks (i.e., releases ownership of) the associated mutex (public member function) |
| [swap](https://en.cppreference.com/w/cpp/thread/unique_lock/swap) | swaps state with another **std::unique_lock** (public member function) |

| [mutex](https://en.cppreference.com/w/cpp/thread/unique_lock/mutex) | returns a pointer to the associated mutex (public member function) |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| [owns_lock](https://en.cppreference.com/w/cpp/thread/unique_lock/owns_lock) | tests whether the lock owns (i.e., has locked) its associated mutex (public member function)   for example : lck.owns_lock() |

```cpp
#include <mutex>
#include <thread>
#include <chrono>
 
struct Box {
    explicit Box(int num) : num_things{num} {}
 
    int num_things;
    std::mutex m;
};
 
void transfer(Box &from, Box &to, int num)
{
    // don't actually take the locks yet
    std::unique_lock<std::mutex> lock1(from.m, std::defer_lock);
    std::unique_lock<std::mutex> lock2(to.m, std::defer_lock);
 
    // lock both unique_locks without deadlock
    std::lock(lock1, lock2);
 
    from.num_things -= num;
    to.num_things += num;
 
    // 'from.m' and 'to.m' mutexes unlocked in 'unique_lock' dtors
}
 
int main()
{
    Box acc1(100);
    Box acc2(50);
 
    std::thread t1(transfer, std::ref(acc1), std::ref(acc2), 10);
    std::thread t2(transfer, std::ref(acc2), std::ref(acc1), 5);
 
    t1.join();
    t2.join();
}
```



### std::atomic

**常用函数**

| [operator=](https://en.cppreference.com/w/cpp/atomic/atomic/operator%3D) | stores a value into an atomic object (public member function) |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| [is_lock_free](https://en.cppreference.com/w/cpp/atomic/atomic/is_lock_free) | checks if the atomic object is lock-free (public member function) |
| [store](https://en.cppreference.com/w/cpp/atomic/atomic/store) | atomically replaces the value of the atomic object with a non-atomic argument (public member function) |
| [load](https://en.cppreference.com/w/cpp/atomic/atomic/load) | atomically obtains the value of the atomic object (public member function) |

| [wait](https://en.cppreference.com/w/cpp/atomic/atomic/wait)(C++20) | blocks the thread until notified and the atomic value changes (public member function) |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| [notify_one](https://en.cppreference.com/w/cpp/atomic/atomic/notify_one)(C++20) | notifies at least one thread waiting on the atomic object (public member function) |
| [notify_all](https://en.cppreference.com/w/cpp/atomic/atomic/notify_all)(C++20) | notifies all threads blocked waiting on the atomic object (public member function) |

| [fetch_add](https://en.cppreference.com/w/cpp/atomic/atomic/fetch_add) | atomically adds the argument to the value stored in the atomic object and obtains the value held previously (public member function) |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| [fetch_sub](https://en.cppreference.com/w/cpp/atomic/atomic/fetch_sub) | atomically subtracts the argument from the value stored in the atomic object and obtains the value held previously (public member function) |
| [fetch_and](https://en.cppreference.com/w/cpp/atomic/atomic/fetch_and) | atomically performs bitwise AND between the argument and the value of the atomic object and obtains the value held previously (public member function) |
| [fetch_or](https://en.cppreference.com/w/cpp/atomic/atomic/fetch_or) | atomically performs bitwise OR between the argument and the value of the atomic object and obtains the value held previously (public member function) |
| [fetch_xor](https://en.cppreference.com/w/cpp/atomic/atomic/fetch_xor) | atomically performs bitwise XOR between the argument and the value of the atomic object and obtains the value held previously (public member function) |

```cpp
// constructing atomics
#include <iostream>       // std::cout
#include <atomic>         // std::atomic, std::atomic_flag, ATOMIC_FLAG_INIT
#include <thread>         // std::thread, std::this_thread::yield
#include <vector>         // std::vector

std::atomic<bool> ready (false);
std::atomic_flag winner = ATOMIC_FLAG_INIT;

void count1m (int id) {
  while (!ready) { std::this_thread::yield(); }      // wait for the ready signal
  for (volatile int i=0; i<1000000; ++i) {}          // go!, count to 1 million
  if (!winner.test_and_set()) { std::cout << "thread #" << id << " won!\n"; }
};

int main ()
{
  std::vector<std::thread> threads;
  std::cout << "spawning 10 threads that count to 1 million...\n";
  for (int i=1; i<=10; ++i) threads.push_back(std::thread(count1m,i));
  ready = true;
  for (auto& th : threads) th.join();

  return 0;
}
```



### std::condition_variable

作用：线程间通信的机制，通过一个线程去唤醒另一个线程或者另外其他所有线程；

主要功能函数

- [**wait**](https://www.cplusplus.com/reference/condition_variable/condition_variable/wait/)

  Wait until notified (public member function )

- [**wait_for**](https://www.cplusplus.com/reference/condition_variable/condition_variable/wait_for/)

  Wait for timeout or until notified (public member function )

- [**wait_until**](https://www.cplusplus.com/reference/condition_variable/condition_variable/wait_until/)

  Wait until notified or time point (public member function )

- [**notify_one**](https://www.cplusplus.com/reference/condition_variable/condition_variable/notify_one/)

  Notify one (public member function )

- [**notify_all**](https://www.cplusplus.com/reference/condition_variable/condition_variable/notify_all/)

  Notify all (public member function )

运行示例

```cpp
// condition_variable example
#include <iostream>           // std::cout
#include <thread>             // std::thread
#include <mutex>              // std::mutex, std::unique_lock
#include <condition_variable> // std::condition_variable

std::mutex mtx;
std::condition_variable cv;
bool ready = false;

void print_id (int id) {
  std::unique_lock<std::mutex> lck(mtx);
  while (!ready) cv.wait(lck);
  // ...
  std::cout << "thread " << id << '\n';
}

void go() {
  std::unique_lock<std::mutex> lck(mtx);
  ready = true;
  cv.notify_all();
}

int main ()
{
  std::thread threads[10];
  // spawn 10 threads:
  for (int i=0; i<10; ++i)
    threads[i] = std::thread(print_id,i);

  std::cout << "10 threads ready to race...\n";
  go();                       // go!

  for (auto& th : threads) th.join();

  return 0;
}
```

参考链接：https://www.cplusplus.com/reference/condition_variable/condition_variable/



**有关atomic 、 condition_variable 、unique_lock 和 mutex 的通俗解释**

个人理解： mutex 提供了底层的锁机制，而unique_lock 是对mutex的一种包装，在mutex的基础上提供了其他一些机制，比如延时锁，atomic是为了让变量在操作过程中不被其他线程干扰，从而可以执行原子操作，condition_variable 执行的是多个线程之间的通信；

总的来说： atomic 、 mutex 和 unique_lock 主要实现的是线程之间的互斥机制；

​					atomic 主要是针对一个变量上的操作，避免频繁使用加锁解锁；

​					unique_lock 和 mutex 主要用于 多语句之间的加锁解锁，多使用unique_lock，其接口更加丰富，而且可以直接作为条件变量的参数；

​                    condition_variable 主要为了实现线程之间的同步机制；

其他博主理解：https://zhuanlan.zhihu.com/p/136861784



### std::bind

目的： 给原有函数进行一次包装，对相应的参数进行指定或者改变，只是改变外部接口，并不改变内部逻辑，可以设置一些默认参数，或者通过多个参数来计算出一个新的值，交给底层去处理；主要是为了应对接口的变化；

> The function template `bind` generates a forwarding call wrapper for `f`. Calling this wrapper is equivalent to invoking `f` with some of its arguments bound to `args`.

参考链接：https://en.cppreference.com/w/cpp/utility/functional/bind

​				   https://blog.csdn.net/qq_37653144/article/details/79285221

```cpp

#include <iostream>
#include <functional>
 
void fn(int n1, int n2, int n3) {
	std::cout << n1 << " " << n2 << " " << n3 << std::endl;
}
 
int fn2() {
	std::cout << "fn2 has called.\n";
	return -1;
}
 
int main()
{
	using namespace std::placeholders;
	auto bind_test1 = std::bind(fn, 1, 2, 3);
	auto bind_test2 = std::bind(fn, _1, _2, _3);
	auto bind_test3 = std::bind(fn, 0, _1, _2);
	auto bind_test4 = std::bind(fn, _2, 0, _1);
 
	bind_test1();//输出1 2 3
	bind_test2(3, 8, 24);//输出3 8 24
	bind_test2(1, 2, 3, 4, 5);//输出1 2 3，4和5会被丢弃
	bind_test3(10, 24);//输出0 10 24
	bind_test3(10, fn2());//输出0 10 -1
	bind_test3(10, 24, fn2());//输出0 10 24，fn2会被调用，但其返回值会被丢弃
	bind_test4(10, 24);//输出24 0 10
	return 0;

```

​					





### 智能指针VS 常规指针

weak_ptr 的使用：

​	weak_ptr 不能直接访问所指对象的内容，而是要先获得临时所有权才可以，这个可以通过 .lock()函数来实现；

> `std::weak_ptr` is a smart pointer that holds a non-owning ("weak") reference to an object that is managed by [std::shared_ptr](https://en.cppreference.com/w/cpp/memory/shared_ptr). It must be converted to [std::shared_ptr](https://en.cppreference.com/w/cpp/memory/shared_ptr) in order to access the referenced object.
>
> If the original [std::shared_ptr](https://en.cppreference.com/w/cpp/memory/shared_ptr) is destroyed at this time, the object's lifetime is extended until the temporary [std::shared_ptr](https://en.cppreference.com/w/cpp/memory/shared_ptr) is destroyed as well.

常用函数：

| [reset](https://en.cppreference.com/w/cpp/memory/weak_ptr/reset) | releases the ownership of the managed object             |
| ------------------------------------------------------------ | -------------------------------------------------------- |
| [expired](https://en.cppreference.com/w/cpp/memory/weak_ptr/expired) | checks whether the referenced object was already deleted |

| [lock](https://en.cppreference.com/w/cpp/memory/weak_ptr/lock) | creates a `shared_ptr` that manages the referenced object |
| ------------------------------------------------------------ | --------------------------------------------------------- |
|                                                              |                                                           |

使用案例

```cpp
wp_map_.lock()->condition_var_is_map_updated_.wait(wp_map_.lock()->data_lock_);
```



参考链接：

https://en.cppreference.com/w/cpp/memory/weak_ptr

### 类型转换对比



### 关键词explict



### C++的异常机制, C++为什么不推荐使用异常，而是使用错误码和断言？



### typedefs



### detach 的作用： 具体的，数据结构是否发生了变化，主线程和子线程之间的数据结构是怎样处理的？

解决三线程终结的问题；



### linux 相关 优质网址：

https://linuxtools-rst.readthedocs.io/zh_CN/latest/tool/gdb.html



Cmake 构建工程优质链接：

https://segmentfault.com/a/1190000022075547





### KITTI calib.txt 中各个参数的说明；

```
S_xx: 1x2 size of image xx before rectification
K_xx: 3x3 calibration matrix of camera xx before rectification
D_xx: 1x5 distortion vector of camera xx before rectification
R_xx: 3x3 rotation matrix of camera xx (extrinsic)
T_xx: 3x1 translation vector of camera xx (extrinsic)
S_rect_xx: 1x2 size of image xx after rectification
R_rect_xx: 3x3 rectifying rotation to make image planes co-planar
P_rect_xx: 3x4 projection matrix after rectification
```



### 解读KITTI数据集calib.txt参数：

```shell
//原参数格式：
P0: 7.070912000000e+02 0.000000000000e+00 6.018873000000e+02 0.000000000000e+00 0.000000000000e+00 7.070912000000e+02 1.831104000000e+02 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 1.000000000000e+00 0.000000000000e+00
P1: 7.070912000000e+02 0.000000000000e+00 6.018873000000e+02 -3.798145000000e+02 0.000000000000e+00 7.070912000000e+02 1.831104000000e+02 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 1.000000000000e+00 0.000000000000e+00
//->重新排版后；
//相机1的内参；为主参考相机
# 7.070912000000e+02 0.000000000000e+00 6.018873000000e+02 0.000000000000e+00
# 0.000000000000e+00 7.070912000000e+02 1.831104000000e+02 0.000000000000e+00
# 0.000000000000e+00 0.000000000000e+00 1.000000000000e+00 0.000000000000e+00
//相机2的内参
# 7.070912000000e+02 0.000000000000e+00 6.018873000000e+02 -3.798145000000e+02
# 0.000000000000e+00 7.070912000000e+02 1.831104000000e+02 0.000000000000e+00
# 0.000000000000e+00 0.000000000000e+00 1.000000000000e+00 0.000000000000e+00
前三行三列是内参矩阵K, 第四列是平移相关的参数，但是不是直接给的平移，而是其像素距离；f 和b 相乘的负数；负数可能是由于右相机在左相机X的负半轴上；
```

具体图示：

![img](typora_image/20171025150247668)

所以，要求得b = -fubx/fx; 

参考链接 1：https://blog.csdn.net/yangziluomu/article/details/78339575

参考链接2： https://blog.csdn.net/weixin_39760368/article/details/110804044?utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7EBlogCommendFromBaidu%7Edefault-17.control&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7EBlogCommendFromBaidu%7Edefault-17.control

参考链接3：https://stackoverflow.com/questions/29407474/how-to-understand-the-kitti-camera-calibration-files

参考链接4 ：https://blog.csdn.net/YMWM_/article/details/107669394?utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7EBlogCommendFromBaidu%7Edefault-8.control&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7EBlogCommendFromBaidu%7Edefault-8.control
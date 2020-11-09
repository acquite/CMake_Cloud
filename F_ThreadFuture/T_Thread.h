
// C++多线程图像分块处理示例
// https://blog.csdn.net/qq_25847123/article/details/74779959

enum CutNum{Cn1, Cn2,Cn3,Cn4};

#include <iostream>
#include <time.h>
#include <thread> // 提供线程类
#include <mutex> // 提供互斥锁类(对部分和进行累加的时候需要加锁)，很好用

// 返回自系统开机以来的毫秒数（tick）
unsigned long GetTickCount();

//线程入口
void G_Cut(enum CutNum ec);

//分线程测试入口
void G_CutThreadTest();


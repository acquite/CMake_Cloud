#include"T_Thread.h"

// 返回自系统开机以来的毫秒数（tick）
unsigned long GetTickCount()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

std::mutex mtx;// 定义一个互斥锁
long totalSum;// 总和

void G_Cut(enum CutNum cn)
{
    int startRow, startCol, endRow, endCol;
    // std::cout << "G_Cut" << std::endl;
    switch (cn)
    {
    case Cn1:
        startRow = 0;
        endRow = 1000000;
        startCol = 0;
        endCol = 1000000;
        break;
    case Cn2:
        startRow = 1000000;
        endRow = 2000000;
        startCol = 1000000;
        endCol = 2000000;
        break;
    case Cn3:
        startRow = 2000000;
        endRow = 3000000;
        startCol = 2000000;
        endCol = 3000000;
        break;
    case Cn4:
        startRow = 3000000;
        endRow = 4000000;
        startCol = 3000000;
        endCol = 4000000;
        break;

    default:
        break;
    }
    double t = (double)GetTickCount();
    long sum = 0;
    for (int i = startRow; i < endRow; i++)
    {
        sum += i;
        // for (int j = startCol; j < endCol; j++)
        // {
        //     sum += j;
        // }
    }

    mtx.lock();// 在访问公共变量totalSum 之前对其进行加锁
    totalSum += sum;
    mtx.unlock();// 访问完毕立刻解锁

    std::cout << cn << " : " << sum << std::endl;
    std::cout << "task completed! Time elapsed " << (double)GetTickCount() -t << std::endl;
    // 打印本次线程时间花费
}



void G_CutThreadTest()
{
    double t = (double)GetTickCount();
    std::thread t0(G_Cut, Cn1);
    std::thread t1(G_Cut, Cn2);
    std::thread t2(G_Cut, Cn3);
    std::thread t3(G_Cut, Cn4);
    
    t0.join();// 等待子线程t0执行完毕
    t1.join();
    t2.join();
    t3.join();

    std::cout << std::endl <<"多线程总时间花费：" << (double)GetTickCount() - t << std::endl;
    std::cout << "总值(多线程）: " << totalSum<< std::endl << std::endl;


    // 验证准确性
    long sum =0;

    t = (double)GetTickCount();
    for (int i = 0; i < 4000000; i++) {
        sum += i;
    }

    std::cout << "参照时间花费：" << (double)GetTickCount() - t << std::endl;
    std::cout << "总值(多线程）: " << sum << std::endl;
}
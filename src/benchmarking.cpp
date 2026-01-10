#include <iostream>
#include <chrono>
#include <redisclient.h>
#include <eigen3/Eigen/Dense>

#include <signal.h>
#include <LoopTimer.h>

bool runloop = true;
void sig_handler(int signum) {runloop = false;}


int main(int argc, char const *argv[])
{
    signal(SIGINT,sig_handler);
    RedisClient redis_client;
    redis_client.connect();

    Eigen::Matrix4d mat = Eigen::Matrix4d::Ones()* 78.2475755;

    for (int i = 0; i < 10; i++)
    {
        std::string key = "benchmarking_key_" + std::to_string(i);
        // redis_client.createEigenReadCallback(key,mat);
        redis_client.createEigenWriteCallback(key,mat);
    }

    LoopTimer timer;
    timer.setLoopFrequency(2000);
    timer.InitializeTimer();
    while (runloop)
    {
        timer.WaitForNextLoop();
        auto t1 = std::chrono::high_resolution_clock::now();
        // redis_client.executeBatchAllReadCallbacks();
        redis_client.executeBatchAllWriteCallbacks();
        // redis_client.executeAllReadCallbacks();
        auto t2 = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
        // std::cout << "duration: " << duration.count() << std::endl;
        // std::cout << "mat: " << mat.transpose() << std::endl;
    }
    
    std::cout << "exited the loop" << std::endl;
    timer.printTimerHistory();
    return 0;
}

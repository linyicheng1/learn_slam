#ifndef __THREAD_H
#define __THREAD_H

#include <mutex>
#include <thread>
#include <zconf.h>

namespace my_slam
{
    // 多线程类，纯虚类
    class thread
    {
    public:
        std::mutex mutex_;
        thread() = default;
        ~thread() = default;
        void start(int delay)
        {
            delay_ = delay;
            pthread_create(&thread_, NULL, pthread_fun, (void *) this);
        }
        static void* pthread_fun(void* __this)
        {
            auto * _this =(thread *)__this;
            while (true)
            {
                try
                {
                    _this->process();
                }
                catch (...)
                {
                    break;
                }
                usleep(_this->delay_);
            }
        }
        int delay_{};
        virtual void process() = 0;
        pthread_t thread_{};
    };
}
#endif //__THREAD_H

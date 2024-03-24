#ifndef PACSIMDEADTIME_HPP
#define PACSIMDEADTIME_HPP

#include <mutex>
#include <queue>

template <typename T> class DeadTime
{
public:
    DeadTime(double deadtime) { this->deadTime = deadtime; }

    DeadTime(const DeadTime& in)
    {
        std::lock_guard<std::mutex> l(_mutex);
        this->deadTime = in.deadTime;
        this->deadTimeQueue = in.deadTimeQueue;
        this->times = in.times;
    }

    DeadTime& operator=(const DeadTime& in)
    {
        std::lock_guard<std::mutex> l(_mutex);
        this->deadTime = in.deadTime;
        this->deadTimeQueue = in.deadTimeQueue;
        this->times = in.times;
        return *this;
    }

    void updateDeadTime(double time) { this->deadTime = time; }

    T getOldest()
    {
        std::lock_guard<std::mutex> l(_mutex);
        T elem = this->deadTimeQueue.front();
        this->deadTimeQueue.pop();
        this->times.pop();
        return elem;
    }

    bool availableDeadTime(double time)
    {
        bool ret = false;
        std::lock_guard<std::mutex> l(_mutex);
        if (this->deadTimeQueue.size() >= 1)
        {
            T elem = this->deadTimeQueue.front();
            if (time >= (this->times.front() + this->deadTime))
            {
                ret = true;
            }
        }
        return ret;
    }

    void addVal(T val, double currTime)
    {
        std::lock_guard<std::mutex> l(_mutex);
        this->deadTimeQueue.push(val);
        this->times.push(currTime);
    }

private:
    std::mutex _mutex;
    double deadTime;
    std::queue<T> deadTimeQueue;
    std::queue<double> times;
};

#endif /* PACSIMDEADTIME_HPP */
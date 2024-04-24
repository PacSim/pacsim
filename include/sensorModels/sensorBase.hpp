#ifndef PACSIMISENSORBASE_HPP
#define PACSIMISENSORBASE_HPP

#include <queue>

template <typename T> class SensorBase
{
public:
    double getRate() { return this->rate; }

    T getOldest()
    {
        T elem = this->deadTimeQueue.front();
        this->deadTimeQueue.pop();
        return elem;
    }

    bool availableDeadTime(double time)
    {
        if (this->deadTimeQueue.size() >= 1)
        {
            T elem = this->deadTimeQueue.front();
            if (time >= (elem.timestamp + this->deadTime))
            {
                return true;
            }
        }
        return false;
    }

    bool sampleReady(double time) { return (time >= (this->lastSampleTime + 1 / this->rate)); }

    void registerSampling()
    {
        this->lastSampleTime += 1.0 / this->rate;
        return;
    }

    Eigen::Vector3d getPosition() { return this->position; }

    Eigen::Vector3d getOrientation() { return this->orientation; }

protected:
    Eigen::Vector3d position;
    Eigen::Vector3d orientation;

    double rate;
    double lastSampleTime;
    double deadTime;
    std::queue<T> deadTimeQueue;
    int numFrames;
};

#endif /* PACSIMISENSORBASE_HPP */
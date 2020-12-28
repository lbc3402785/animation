#ifndef THREADSAFEQUEUE_H
#define THREADSAFEQUEUE_H
#include <iostream>
#include <string>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <memory>

template<class T, class Container = std::queue<T>>
class ThreadSafeQueue {
public:
    ThreadSafeQueue(){
        size=1;
    }
    ThreadSafeQueue(int size):size(size){
    }

    template <class Element>
    bool TryPush(Element&& element) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.size()==size) {
            return false;
        }
        queue_.push(std::forward<Element>(element));
        notEmpty.notify_one();
        return true;
    }

    template <class Element>
    void WaitAndPush(Element&& element) {
        std::unique_lock<std::mutex> lock(mutex_);
        notFull.wait(lock, [this]() {
            return !(queue_.size()==size);
        });
        queue_.push(std::forward<Element>(element));
        notEmpty.notify_one();
    }

    void WaitAndPop(T& t) {
        std::unique_lock<std::mutex> lock(mutex_);
        notEmpty.wait(lock, [this]() {
            return !queue_.empty();
        });

        t = std::move(queue_.front());
        queue_.pop();
        notFull.notify_one();
    }

    std::shared_ptr<T> WaitAndPop() {
        std::unique_lock<std::mutex> lock(mutex_);
        notEmpty.wait(lock, [this]() {
            return !queue_.empty();
        });

        std::shared_ptr<T> t_ptr = std::make_shared<T>(queue_.front());
        queue_.pop();
        notFull.notify_one();
        return t_ptr;
    }

    bool TryPop(T& t) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) {
            return false;
        }

        t = std::move(queue_.front());
        queue_.pop();
        notFull.notify_one();
        return true;
    }

    std::shared_ptr<T> TryPop() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) {
            return std::shared_ptr<T>();
        }
        std::shared_ptr<T> t_ptr = std::make_shared<T>(queue_.front());
        queue_.pop();
        return t_ptr;
    }

    bool IsEmpty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }
    void clear(){
        std::lock_guard<std::mutex> lock(mutex_);
        while(!queue_.empty()){
            queue_.pop();
        }
    }
private:
    ThreadSafeQueue(const ThreadSafeQueue&) = delete;
    ThreadSafeQueue& operator=(const ThreadSafeQueue&) = delete;
    ThreadSafeQueue(ThreadSafeQueue&&) = delete;
    ThreadSafeQueue& operator=(ThreadSafeQueue&&) = delete;

private:
    Container queue_;
    size_t size;
    std::condition_variable notEmpty;
    std::condition_variable notFull;
    mutable std::mutex mutex_;
};

#endif // THREADSAFEQUEUE_H

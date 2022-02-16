#include "barrier.h"

namespace SEUTraffic{
    void Barrier::wait(){
        // 尝试获取 m_mutex锁，如果获取不到则堵塞
        std::unique_lock<std::mutex> lock(m_mutex);
        assert(0u!= *currCounter); // currCounter的值为0，意味着结束等待了
        if (!--*currCounter){ //当前counter的值为0
            // 如果当前的curCounter等于counter[0], 则变为counter[1]
            // 如果当前的curCounter等于counter[1], 则变为counter[0]
            currCounter += currCounter == counter ? 1 : -1;
            *currCounter = m_threads;
            // 通知所有堵塞的线程
            m_condition.notify_all();
        }
        else{
            size_t *currCounter_local = currCounter; // 本地指针指向curCounter全局计数
            // 只有当 m_condition.notify_all() && *currCounter_local == 0才停止堵塞
            m_condition.wait(lock, [currCounter_local] {return  *currCounter_local == 0;});
        }
    }
}

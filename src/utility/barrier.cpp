#include "barrier.h"

namespace SEUTraffic{
    void Barrier::wait(){
        std::unique_lock<std::mutex> lock(m_mutex);
        assert(0u!= *currCounter); // currCounter的值为0，意味着结束等待了
        if (!--*currCounter){
            currCounter += currCounter == counter ? 1 : -1;
            *currCounter = m_threads;
            m_condition.notify_all();
        }
        else{
            size_t *currCounter_local = currCounter; // 本地指针指向currCouter全局计数
            m_condition.wait(lock, [currCounter_local] {return  *currCounter_local == 0;});
        }
    }
}

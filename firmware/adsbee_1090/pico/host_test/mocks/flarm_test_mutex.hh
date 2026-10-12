#pragma once
#include <mutex>
#define PFB_MUTEX_CUSTOM
#define PFB_MUTEX_TYPE std::mutex
#define PFB_MUTEX_INIT(mtx) ((void)0)
#define PFB_MUTEX_LOCK(mtx) (mtx).lock()
#define PFB_MUTEX_UNLOCK(mtx) (mtx).unlock()

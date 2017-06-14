/// ---------------------------------------------------------------------------
/// Copyright (C) 2016 by SIA. All rights reserved.
/// @file SharedMemory.h
/// @date 2016/11/10
///
/// @author J.F.Xiong
/// Contact: xjf_whut@qq.com
///
/// @brief Shared memory
///
/// Map shared memory and initialize RWLock
///
/// @version 1.0.0
/// @note Camel case
/// ---------------------------------------------------------------------------

#ifdef __QNXNTO__

#pragma once

#include <fcntl.h>
#include <sys/mman.h>
#include <pthread.h>

class SharedMemory
{
public:
	/// @return 0: success, 1: unable to create shared memory
	template<typename T> int Initialize(const char *name, T **ptr)
	{
		int length = sizeof(T);
		int fildes;
		/// 1 If the shared memory exists, open it
		if (access(name, F_OK) == 0)
		{
			fildes = shm_open(name, O_RDWR, S_IRWXU);
			*ptr = (T *) mmap(NULL, length, PROT_READ | PROT_WRITE, MAP_SHARED, fildes, 0);
			return 0;
		}
		/// 2 Else, create shared memory and initialize it
		/// 2.1 Create
		fildes = shm_open(name, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU);
		if (fildes == -1)
			return 1;
		/// 2.2 Truncate
		ftruncate(fildes, length);
		/// 2.3 Map
		*ptr = (T *) mmap(NULL, length, PROT_READ | PROT_WRITE, MAP_SHARED, fildes, 0);
		/// 2.4 Set read/write lock
		pthread_rwlockattr_t attr;
		pthread_rwlockattr_init(&attr);
		pthread_rwlockattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
		pthread_rwlock_init(&(*ptr)->rwlock, &attr);

		return 0;
	}
};

#endif

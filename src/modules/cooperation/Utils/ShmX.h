/// ---------------------------------------------------------------------------
/// Copyright (C) 2016 by SIA. All rights reserved.
/// @file ShmX.h
/// @date 2016/11/14
///
/// @author J.F.Xiong
/// Contact: xjf_whut@qq.com
///
/// @brief Shared memory demo
///
/// Description of the shared memory
///
/// @version 1.0.0
/// @note Camel case
/// ---------------------------------------------------------------------------

#ifdef __QNXNTO__

#pragma once

typedef struct
{
	pthread_rwlock_t rwlock;
	double timestamp;
	int parameter;
} ShmX;

#endif

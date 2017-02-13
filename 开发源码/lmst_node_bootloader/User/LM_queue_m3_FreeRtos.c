/*
 * LM_queue_m3_FreeRtos.c
 *
 * Created: 2016/9/2 
 *  Author: YJ
 * Email: huangyj@lzrobot.com
 */ 
 
#include "LM_queue_m3_FreeRtos.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* =============================================
		构造一个空队列
		return
			句柄		成功
			NULL		失败
================================================*/
HiQueue HiCreateQueue(char *buff, int32_t bufflen)
{
	HiQueue q;

	q = (HiQueue)pvPortMalloc(sizeof(struct HiQueueRecord));

	if(q == NULL)
		return NULL;

	memset(q, 0, sizeof(struct HiQueueRecord));
	//pthread_mutex_init(&q->hMutex,NULL);
	q->Array = (unsigned char *)buff;

	if(q->Array == NULL)
	{
		free(q);
		return NULL;
	}
	q->Capacity = bufflen;
	q->mutex    = xSemaphoreCreateMutex();  
	return q;
}


/* =============================================
		销毁队列
================================================*/
void  HiDestroyQueue(HiQueue q)
{
	//free(q->Array);
	//pthread_mutex_destroy(&q->hMutex);
	vPortFree(q);
}

/* =============================================
		入列
		return
			1		成功
			-1		失败
================================================*/
int	HiEnQueue(char * buf, int len, HiQueue q)
{
	int i = 0;

	//pthread_mutex_lock(&q->hMutex);
	xSemaphoreTake( q->mutex, portMAX_DELAY );
	if(len > (q->Capacity - q->Size))
	{
		//pthread_mutex_unlock(&q->hMutex);
		xSemaphoreGive(q->mutex);
		return -1;
	}

	q->Size += len;
	while(1)
	{
		//循环
		q->tail = (q->tail) % q->Capacity;
		q->Array[q->tail++] = buf[i++];
		if(i == len)
		{
			//pthread_mutex_unlock(&q->hMutex);
			xSemaphoreGive(q->mutex);
			return 1;
		}
	}
}

/* =============================================
		入列（无锁）
		return
			1		成功
			-1		失败
================================================*/
int	HiEnQueue_nolock(char * buf, int len, HiQueue q)
{
	int i = 0;

	if(len > (q->Capacity - q->Size))
	{
		return -1;
	}

	while(1)
	{
		//循环
		q->tail = (q->tail) % q->Capacity;
		q->Array[q->tail++] = buf[i++];
		if(i == len)
		{
			q->Size += len;
			return 1;
		}
	}
}

/* =============================================
		出列
		return
			1		成功
			-1		失败 长度不足
================================================*/
int	HiDeQueue(char * buf, int len, HiQueue q)
{
	int i = 0;

	if(len <= 0)
		return -1;

	//pthread_mutex_lock(&q->hMutex);
	xSemaphoreTake( q->mutex, portMAX_DELAY );
	if(len > q->Size)
	{
		//pthread_mutex_unlock(&q->hMutex);
		xSemaphoreGive(q->mutex);
		return -1;
	}
	q->Size -= len;
	while(1)
	{
		//循环
		q->head  = (q->head) % q->Capacity;
		buf[i++] = q->Array[q->head++];
		if(i == len)
		{
			//pthread_mutex_unlock(&q->hMutex);
			xSemaphoreGive(q->mutex);
			return 1;
		}
	}
}

/* =============================================
		出列(无锁)
		return
			1		成功
			-1		失败 长度不足
================================================*/
int	HiDeQueue_nolock(char * buf, int len, HiQueue q)
{
	int i = 0;

	if(len <= 0)
		return -1;

	if(len > q->Size)
	{
		return -1;
	}

	while(1)
	{
		//循环
		q->head  = (q->head) % q->Capacity;
		buf[i++] = q->Array[q->head++];
		if(i == len)
		{
		  q->Size -= len;
			return 1;
		}
	}
}

/* =============================================
		窥视
		return
			1		成功
			-1		失败 长度不足

		int	offset 偏移offset个字节开始peak
================================================*/
int	HiPeakQueue(char * buf, int len, int offset, HiQueue q)
{
	int i = 0;
	int h;
	if(len <= 0)
		return -1;

	//pthread_mutex_lock(&q->hMutex);
	xSemaphoreTake( q->mutex, portMAX_DELAY );
	if(len > q->Size - offset)
	{
		//pthread_mutex_unlock(&q->hMutex);
		xSemaphoreGive(q->mutex);
		return -1;
	}
	//q->Size -= len;
	h = q->head;

	//虑掉偏移量
	for(i = 0; i < offset; i++)
	{
		h  = h % q->Capacity;
		h++;
	}

	i = 0;

	while(1)
	{
		//循环
		h  = h % q->Capacity;
		buf[i++] = q->Array[h++];
		if(i == len)
		{
			//pthread_mutex_unlock(&q->hMutex);
			xSemaphoreGive(q->mutex);
			return 1;
		}
	}
}


/* =============================================
			删除队列的部分字节
				return
					1		成功
					-1		失败
================================================*/
int HiDeleQueueData(int len,  HiQueue q)
{
	//pthread_mutex_lock(&q->hMutex);
	xSemaphoreTake( q->mutex, portMAX_DELAY );
	if(len > q->Size)
	{
		//pthread_mutex_unlock(&q->hMutex);
		xSemaphoreGive(q->mutex);
		return -1;
	}
	q->Size -= len;
	q->head += len;
	//pthread_mutex_unlock(&q->hMutex);
	xSemaphoreGive(q->mutex);
	return 1;
}

/* =============================================
		查询队列空闲的空间
		return
			空闲空间
================================================*/
int	HiGetQueueIdleNum(HiQueue q)
{
	int ret;
	//pthread_mutex_lock(&q->hMutex);
	//vTaskSuspendAll();
	ret = q->Capacity - q->Size;
	//pthread_mutex_unlock(&q->hMutex);
	//xTaskResumeAll();
	return ret;
}

/* =============================================
		查询队列数据长度
		return
			数据长度
================================================*/
int	HiGetQueueDataLen(HiQueue q)
{
	return q->Size;
}

/* =============================================
		队列数据循环右移

================================================*/
int	HiQueueRR(int32_t byte, HiQueue q)
{
	//pthread_mutex_lock(&q->hMutex);
	xSemaphoreTake( q->mutex, portMAX_DELAY );
	while(byte--)
	{
		q->head  			= (q->head) % q->Capacity;
		q->tail 			= (q->tail) % q->Capacity;

		q->Array[q->tail++] = q->Array[q->head++];
	}
	//pthread_mutex_unlock(&q->hMutex);
	xSemaphoreGive(q->mutex);
	return 1;
}

/* =============================================
			清空队列
================================================*/
void HiCleanQueue( HiQueue q)
{
	//pthread_mutex_lock(&q->hMutex);
	xSemaphoreTake( q->mutex, portMAX_DELAY );
	q->head = q->tail = q->Size = 0;
	//pthread_mutex_unlock(&q->hMutex);
	xSemaphoreGive(q->mutex);
	return;
}

#ifndef PENDINGMESSAGES_H
#define PENDINGMESSAGES_H

#define QUEUE_SIZE 500
#define WAIT_MESSAGE_SIZE 11


#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

typedef struct pendingMessages {
	char tempBuff[WAIT_MESSAGE_SIZE];
	QueueHandle_t queue;
	SemaphoreHandle_t queueLock;
	int size;
} pendingMessages_t;

pendingMessages_t globalQ;

int pendingMessagesCreate(pendingMessages_t* this);

// 1. Wait for notification for queue is appreciably full 
// 2. Copy data from queue into user buffer  
int pendingMessagesWaitAndPop(char* userBuff, pendingMessages_t* this);

//1. Create a new Entry struct
//2. Add it to the queue
//3. Check capcity of Queue
//4. Notify Bluetooth Task if Appropriate
int pendingMessagesPush(int dataSize, char* userBuff, pendingMessages_t* this);




#endif //PENDINGMESSAGES_H
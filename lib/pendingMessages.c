#include "pendingMessages.h"
#include "notification.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"



int pendingMessagesCreate(pendingMessages_t* this) {
	this->queue = xQueueCreate(QUEUE_SIZE, sizeof(char));
	this->queueLock = xSemaphoreCreateMutex();
	if (!this->queue || !this->queueLock) {
		NRF_LOG_ERROR("Not enough memory to create the queue or lock");
		NRF_LOG_FLUSH();
		return -1;
	} else {
		return 1;
	}
}

int pendingMessagesWaitAndPop(char* userBuff, pendingMessages_t* this) {
	int i;



	//Sleep until there is an appreciable amount of data
	
	waitForNotification(BLUETOOTH_NOTIFICATION);

	xSemaphoreTake(this->queueLock, portMAX_DELAY);

	// NRF_LOG_INFO("ready to pop, %d bytes in queue", this->size);
	// NRF_LOG_FLUSH();


	//get data from queue
	for (i = 0; i < WAIT_MESSAGE_SIZE; ++i) {
		if (xQueueReceive(this->queue, &this->tempBuff[i], (TickType_t) 10) == pdFALSE) {
			// NRF_LOG_ERROR("Was Not Able to Pop From Queue!");
			// NRF_LOG_FLUSH();
		}
	}

	xSemaphoreGive(this->queueLock);

	//copy data back to user
	memcpy(userBuff, this->tempBuff, sizeof(this->tempBuff));

	// NRF_LOG_INFO("end of pop");
	// NRF_LOG_FLUSH();

	return WAIT_MESSAGE_SIZE;

}


//1. Create a new Entry struct
//2. Add it to the queue
//3. Check capcity of Queue
//4. Notify Bluetooth Task if Appropriate
int pendingMessagesPush(int dataSize, char* userBuff, pendingMessages_t* this) {
	int i;
	int numNotifications;

	//push to queue
	xSemaphoreTake(this->queueLock, portMAX_DELAY);


	// NRF_LOG_INFO("push took mutex, %d bytes in queue", this->size);
	// NRF_LOG_FLUSH();

	for (i = 0; i < dataSize; ++i) {
		if (xQueueSendToBack(this->queue, (void *) userBuff + i, (TickType_t) 10 )!= pdPASS) {
			NRF_LOG_ERROR("Was Not Able to Push to Queue!");
			NRF_LOG_FLUSH();
		}
	}
	this->size += dataSize;
	numNotifications = this->size/WAIT_MESSAGE_SIZE;
	//Check to see if we need to notify the bluetooth task

	for (i = 0; i < numNotifications; ++i) {
		// NRF_LOG_INFO("Waking up thread %d", this->size);
		sendNotification(BLUETOOTH_NOTIFICATION);
		this->size -= WAIT_MESSAGE_SIZE;
		// NRF_LOG_INFO("The size is %d", this->size);
	}
	
	xSemaphoreGive(this->queueLock);


	// NRF_LOG_INFO("end of poush");
	// NRF_LOG_FLUSH();

	return 1;
}

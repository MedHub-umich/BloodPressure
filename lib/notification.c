#include "notification.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "portmacro_cmsis.h"



void waitForNotification(int notificationHandle) {
	xSemaphoreTake( semphMap[notificationHandle], portMAX_DELAY );
}

void initNotification() {
	int i;
	for (i = 0; i < NUM_NOTIFICATIONS; ++i) {
		semphMap[i] = xSemaphoreCreateCounting(10000, 0);
		//vSemaphoreCreateBinary(semphMap[i]);
	}
}


void sendNotification(int notificationHandle) {
	xSemaphoreGive(semphMap[notificationHandle]);
}

void sendNotificationFromISR(int notificationHandle, BaseType_t* xHigherPriortyTaskWoken) {
	xSemaphoreGiveFromISR(semphMap[notificationHandle], xHigherPriortyTaskWoken);
}


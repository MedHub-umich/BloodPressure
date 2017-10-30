/************************INIT***************************/

/* PARAMETERS: A pointer to the blood pressure device used to interface with HW
 * 
 */
void BPMonitorInit(BloodPressureDevice* myDevice);



 // ************** Tasks ************** //

/* Task that continually reads data packets from the blood pressure devices
 * and sets its latest reading paramter
 */
void BPmonitorReadTask(void*);

/* Task that recieves messages from other tasks. This could include stopping 
 * the update of data and putting the device to sleep.
 */
void BPMonitorIncomingMessageTask(void*);

/* Task that sends messages to other modules. This will mainly invlove 
 * formatting the raw data into a a data packet and transferring the data
 * to the communication node
 */ 
void BPMonitorOutgoingMessageTask(void*);


struct BloodPressureModule
{
    BloodPressureDevice* myBloodPressureDevice;

}; BloodPressureModule
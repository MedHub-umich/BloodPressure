/* PARAMETERS: Pointer to the blood pressure struct you want to initialze.
 *				Pins on the processor that will be connected to SDA and SCL
 *	This function sets up the device so that it knows which pins to check 
 *	for SDA and SCL readings
 */
void initializeBPDevice(BloodPressureDevice* this, int SDApin, int SCLpin);

/* PARAMETERS: Pointer to the blood pressure struct you will read from
 * The following functions are helper functions to get the values on individual
 * lines of the device. They can be used to read the blood pressure value 
 * being written to the microprocessor.
 */
int readSDA(BloodPressureDevice* this);
int readSCL(BloodPressureDevice* this);

/* Put the microprocssor to sleep. Returns non-zero if the device is alaready 
 * in sleep mode or if the operation failed
 */
int sleep();


typedef struct BloodPressureDevice
{
	int SDA; //pin mapping for SDA line 
	int SCL; //pin mapping for SCL line
	int latestReading; //last saved reading from the blood pressure monitor
} BloodPressureDevice;
///////////////////////
// Preprocessor defines
///////////////////////
#define SEND_RAW_SENSOR_VALUES 0x86
#define SEND_CALIBRATED_SENSOR_VALUES 0x87
#define SEND_BATTERY_MILLIVOLTS 0xB1
#define SEND_SIGNATURE 0x81
#define DO_PRINT 0xB8
#define M1_FORWARD 0xC1
#define M1_BACKWARD 0xC2
#define M2_FORWARD 0xC5
#define M2_BACKWARD 0xC6
#define AUTO_CALIBRATE 0xBA
#define CLEAR 0xB7

#define CONSOLE_SERIAL_BAUDRATE 9600	// [baud]
#define CONSOLE_INITIAL_DELAY 2000		// [ms] Initial delay after Serial.begin() to ensure communication is initialized on both ends, and we don't miss the first few Serial.print() lines.
#define CONSOLE_PRINTF_BUFFER_SIZE 1024 // [chars] Don't exceed buffersize in serialPrintf() calls (or enlarge buffersize).

#define POLOLU_SERIAL_BAUDRATE 115200	   // [baud]
#define POLOLU_SERIAL_DEFAULT_TIMEOUT 1500 // [ms] Default timeout in Stream.h is 1000 [ms]; but our longest operation (AUTO_CALIBRATE) takes ~1000 [ms], so we increase timeout to 1100 [ms].
#define POLOLU_SERIAL_POLL_DELAY 1		   // [ms]

#define POLOLU_SIGNATURE_BYTECOUNT 6 // [bytes]
#define POLOLU_SIGNATURE_TIMEOUT 2	 // [ms] Response time is around 0.576 [ms] from request sent to reply received.

#define POLOLU_VOLTAGE_BYTECOUNT 2 // [bytes]
#define BATTERY_VOLTAGE_TIMEOUT 3  // [ms] Response time is around 1.322 [ms] from request sent to reply received.

#define LINE_SENSORS_CALIBRATION_BYTECOUNT 1  // [byte]
#define LINE_SENSORS_CALIBRATION_TIMEOUT 1100 // [ms] Response time is around 1000 [ms] from request sent to reply received.
#define N_LINE_SENSORS 5					  // [sensors]
#define LINE_SENSORS_READ_BYTECOUNT 10		  // [bytes]
#define LINE_SENSORS_READ_TIMEOUT 4			  // [ms] Response time is around 2.37 [ms] from request sent to reply received.
#define MIN_CALIBRATED_SENSOR_VALUE 0
#define MAX_CALIBRATED_SENSOR_VALUE 1000
#define CALIBRATION_ERROR_COUNT_TRESHOLD 5 // [sensor readings that contain errors]

#define POLOLU_OFF_LOOP_DELAY 2000 // [ms]
#define POLOLU_ON_LOOP_DELAY 1000  // [ms]

///////////////////
// Global variables
///////////////////

char signature[POLOLU_SIGNATURE_BYTECOUNT + 1];
unsigned int sensorValues[N_LINE_SENSORS];
float m1Speed;
float m2Speed;
int loopCount;

/////////////////////////////////
// Serial communication functions
/////////////////////////////////

int serialClearReadBuffer()
{

	int numberOfClearedBytes = 0;

	while (Serial1.available())
	{
		Serial1.read();
		numberOfClearedBytes++;
	}

	return numberOfClearedBytes;
}

/**
 * @method: serialWaitForAvailableBytes(): Waits until there are expectedByteCount
 *          bytes available in the serial RX buffer, or until timeout occurs.
 * @arg: expectedByteCount [bytes], an int.
 * @arg: timeout [ms], an unsigned long int. If 0, then NO timeout.
 *                                              If >0, then use timeout.
 * @arg: pollDelay [ms], an unsigned long int.
 * @return: an int. If < 0, then a timeout occurred.
 *                  If > 0, then it's the number of available bytes in the serial RX buffer.
 */
int serialWaitForAvailableBytes(int expectedByteCount,
								unsigned long timeout = POLOLU_SERIAL_DEFAULT_TIMEOUT,
								unsigned long pollDelay = POLOLU_SERIAL_POLL_DELAY)
{

	int availableByteCount = Serial1.available();
	if (availableByteCount >= expectedByteCount)
	{
		return availableByteCount;
	}

	unsigned long start = millis();
	unsigned long now = start;
	unsigned long previous = now;
	unsigned long stop = start + timeout; // NB: This operation might overflow after a long time, but we handle wraparound, so it's ok.
	bool willWrapAround = stop < start;
	bool doesWrapAroundNow = false;
	bool isTimeoutNow = false;
	bool aTimeoutOccurred = false;

	while (!aTimeoutOccurred && availableByteCount < expectedByteCount)
	{
		delay(pollDelay);

		previous = now;
		now = millis();

		doesWrapAroundNow = now < previous;
		if (doesWrapAroundNow)
		{
			willWrapAround = false;
		}

		if (timeout != 0)
		{
			isTimeoutNow = !willWrapAround && now > stop;
			if (isTimeoutNow)
			{
				aTimeoutOccurred = true;
			}
		}

		availableByteCount = Serial1.available();
	}

	if (aTimeoutOccurred)
	{
		return -1;
	}

	return availableByteCount;
}

///////////////////////////
// Pololu library functions
///////////////////////////

void pololuReset()
{

	pinMode(5, OUTPUT);
	digitalWrite(5, LOW);
	delay(10);
	digitalWrite(5, HIGH);
	delay(100);
}

/**
 * @function: pololuSignature()
 *            Request Pololu signature
 * @arg: signature, a pointer to a char buffer of (at least) 6 characters.
 * @arg: timeout [ms], unsigned long.
 * @arg: verbosity, which is an enum of type EVerbosity. Determines how much errors/warnings/info we want to print.
 * @side: This has the side-effect of resetting the Pololu 3pi. See ...
 */
int pololuSignature(char *signature)
{

	serialClearReadBuffer();
	Serial1.write(SEND_SIGNATURE);
	Serial1.flush();
	int status = serialWaitForAvailableBytes(POLOLU_SIGNATURE_BYTECOUNT, POLOLU_SIGNATURE_TIMEOUT);
	if (status < 0)
	{
		return -1; // A timeout occurred.
	}
	Serial1.readBytes(signature, POLOLU_SIGNATURE_BYTECOUNT);
	signature[POLOLU_SIGNATURE_BYTECOUNT] = '\0'; // Add string 0-terminator
	Serial.println(signature);

	return 0;
}

void activateMotor(int motor, float speed)
{

	char opcode = 0x00;
	if (speed > 0.0)
	{
		if (motor == 0)
		{
			opcode = M1_FORWARD;
			m1Speed = speed;
		}
		else
		{
			opcode = M2_FORWARD;
			m2Speed = speed;
		}
	}
	else
	{ // speed <= 0.0
		if (motor == 0)
		{
			opcode = M1_BACKWARD;
			m1Speed = speed;
		}
		else
		{
			opcode = M2_BACKWARD;
			m2Speed = speed;
		}
	}
	unsigned char arg = 0x7f * abs(speed);
	Serial1.write(opcode);
	Serial1.write(arg);
	Serial1.flush();
}

void stopSmooth()
{

	while (m1Speed > 0 || m2Speed > 0)
	{
		if ((m1Speed - 0.05) >= 0)
		{
			m1Speed -= 0.05;
		}
		else
		{
			m1Speed = 0;
		}
		if ((m2Speed - 0.05) >= 0)
		{
			m2Speed -= 0.05;
		}
		else
		{
			m2Speed = 0;
		}

		activateMotor(0, m1Speed);
		activateMotor(1, m2Speed);
		delay(35);
	}
}

float battery()
{

	serialClearReadBuffer();
	Serial1.write(SEND_BATTERY_MILLIVOLTS);
	Serial1.flush();
	if (serialWaitForAvailableBytes(POLOLU_VOLTAGE_BYTECOUNT, BATTERY_VOLTAGE_TIMEOUT) < 0)
	{
		return -1.0; // A timeout occurred.
	}

	char loByte = Serial1.read();
	char hiByte = Serial1.read();
	float v = (((hiByte << 8) | loByte) / 1000.0);

	return (v);
}

char sensorAutoCalibrate()
{

	serialClearReadBuffer();
	Serial1.write(AUTO_CALIBRATE);
	Serial1.flush();
	if (serialWaitForAvailableBytes(LINE_SENSORS_CALIBRATION_BYTECOUNT, LINE_SENSORS_CALIBRATION_TIMEOUT) < 0)
	{
		return 0; // A timeout occurred.
	}

	return Serial1.read();
}

int calibratedSensors(unsigned int sensors[N_LINE_SENSORS])
{

	unsigned int sensor1 = 0;
	unsigned int sensor2 = 0;
	unsigned int sensor3 = 0;
	unsigned int sensor4 = 0;
	unsigned int sensor5 = 0;

	serialClearReadBuffer();
	Serial1.write(SEND_CALIBRATED_SENSOR_VALUES);
	Serial1.flush();
	if (serialWaitForAvailableBytes(LINE_SENSORS_READ_BYTECOUNT, LINE_SENSORS_READ_TIMEOUT) < 0)
	{
		return -1; // A timeout occurred.
	}

	char sensor1Low = Serial1.read();
	char sensor1High = Serial1.read();
	sensor1 = sensor1Low + (sensor1High << 8);
	char sensor2Low = Serial1.read();
	char sensor2High = Serial1.read();
	sensor2 = sensor2Low + (sensor2High << 8);
	char sensor3Low = Serial1.read();
	char sensor3High = Serial1.read();
	sensor3 = sensor3Low + (sensor3High << 8);
	char sensor4Low = Serial1.read();
	char sensor4High = Serial1.read();
	sensor4 = sensor4Low + (sensor4High << 8);
	char sensor5Low = Serial1.read();
	char sensor5High = Serial1.read();
	sensor5 = sensor5Low + (sensor5High << 8);

	sensors[0] = sensor1;
	sensors[1] = sensor2;
	sensors[2] = sensor3;
	sensors[3] = sensor4;
	sensors[4] = sensor5;

	return 0;
}

int sensorAutoCalibrateIfNeeded(const unsigned int sensorValues[N_LINE_SENSORS], bool forceNow = false)
{

	static size_t functionCallCount = 0;
	static size_t calibrationErrorCount = 0;
	static size_t calibrationCount = 0;
	unsigned int sensorValue;
	unsigned int minSensorValue = UINT32_MAX;
	unsigned int maxSensorValue = 0;
	bool hasCalibrationError = false;
	int status = 0;

	for (size_t i = 0; i < N_LINE_SENSORS; i++)
	{
		sensorValue = sensorValues[i];
		if (sensorValue < minSensorValue)
		{
			minSensorValue = sensorValue;
		}
		if (sensorValue > maxSensorValue)
		{
			maxSensorValue = sensorValue;
		}
	}

	hasCalibrationError =
		(maxSensorValue > MAX_CALIBRATED_SENSOR_VALUE) ||
		(maxSensorValue == MIN_CALIBRATED_SENSOR_VALUE) ||
		(minSensorValue == MAX_CALIBRATED_SENSOR_VALUE);

	if (hasCalibrationError)
	{
		calibrationErrorCount++;
	}

	functionCallCount++;
	if (forceNow || (calibrationCount == 0 && calibrationErrorCount > 0) || (calibrationErrorCount > CALIBRATION_ERROR_COUNT_TRESHOLD))
	{
		Serial.println("Initiating auto-calibration...");
		delay(POLOLU_ON_LOOP_DELAY);
		status = sensorAutoCalibrate();
		if (status < 0)
		{
			return -1;
		}
		else
		{ // success
			calibrationCount++;
			calibrationErrorCount = 0;
		}
	}

	return status;
}
//////////////////
// User code
// clang-format on
//////////////////

#define CALIBRATION_DELAY_MS 10
// Values definitions
#define MAX_PD_SPEED 0.60
#define BASE_SPEED 0.30
///////////////////////////////
#define LINE_THRESHOLD 400
#define MAX_ROTATION_ATTEMPTS 200
// for readability
#define LEFT 0
#define RIGHT 1
////////////////////////////////////////////
void setup()
{
	// Initialize serial communication
	Serial1.begin(POLOLU_SERIAL_BAUDRATE);
	Serial1.setTimeout(POLOLU_SERIAL_DEFAULT_TIMEOUT);
	Serial.begin(CONSOLE_SERIAL_BAUDRATE);
	delay(CONSOLE_INITIAL_DELAY);

	// Wait until 3pi is on and responds with correct signature
	while (pololuSignature(signature) < 0)
	{
		Serial.println("Timeout while requesting device signature. Please check if Pololu 3pi is on.");
		delay(POLOLU_OFF_LOOP_DELAY);
	}

	for (int i = 0; i < 10; i++)
		calibratedSensors(sensorValues);

	// Initial calibration
	delay(CALIBRATION_DELAY_MS);
	sensorAutoCalibrate();

	// Initialize global variables
	m1Speed = 0;
	m2Speed = 0;
	loopCount = 0;
}

// Line following functions (for AUTOMATIC mode)
// Counts how many sensors read black line: Return the total numbers of sensor above the threshold
inline unsigned int sensorsOverThresholdCount(unsigned int threshold) {
  unsigned int count = 0;
  for (size_t i = 0; i < N_LINE_SENSORS; i++) { // Loop through all the sensors form i = 0 to 4
    if (sensorValues[i] > threshold) { // if the sensor read value higher than the given threshold
      count++; // count them
    }
  }
  return count; 
}

// T-junction detector: Returns true/false
inline bool isTEnd() { 
  return sensorsOverThresholdCount(LINE_THRESHOLD) >= N_LINE_SENSORS - 1; 
  // If 4 or 5 sensors see black it's a T-junction
  // ≥4 sensors see black -> 1 1 0 1 1  or 1 1 1 1 1 
}

// Line lost detector: Returns true/false
inline bool isLineLost() { 
  return sensorsOverThresholdCount(LINE_THRESHOLD) == 0; 
  // if  0 sensors see black then the line is lost
  // 0 sensors see black -> 0 0 0 0 0
}

// Individual sensor checker: Return true/false
inline bool isSensorOnLine(size_t i) { 
  return i < N_LINE_SENSORS && sensorValues[i] > LINE_THRESHOLD; 
  // Check if i a valid sensor index (0-4) AND the sensor's value above threshold
}

// Center detector: Return true/false
inline bool isLineCentred() {
  return sensorsOverThresholdCount(LINE_THRESHOLD) == 1 && isSensorOnLine(N_LINE_SENSORS / 2);
  // Check if Only 1 sensor sees black AND That sensor is the middle one
  // Only middle sensor sees black -> 0 0 1 0 0
}

// Sharp left turn detector: Return true/false
inline bool isHardLeft() {
  for (size_t i = 0; i <= N_LINE_SENSORS / 2; i++) // Loop through left half + center (sensors 0, 1, 2)
    if (!isSensorOnLine(i)) return false; // If any left sensor DoesNOT see black, return (not a hard left)
  for (size_t i = N_LINE_SENSORS / 2 + 1; i < N_LINE_SENSORS; i++) // Loop through right half (sensors 3, 4)
    if (isSensorOnLine(i)) return false; // If any right sensor see black, return (not a hard left)
  return true; //  If left sensors see black AND right sensors don't, it's a sharp left
  // Left+center see black, right don't -> 1 1 1 0 0
}

// Sharp right turn detector: Return true/false
inline bool isHardRight() { 
  for (size_t i = 0; i < N_LINE_SENSORS / 2; i++) // Loop through left half (sensor 0, 1)
    if (isSensorOnLine(i)) return false; // If any left sensor sees black, return (not a sharp right)
  for (size_t i = N_LINE_SENSORS / 2; i < N_LINE_SENSORS; i++) // Loop through right half + center (sensor 2, 3, 4)
    if (!isSensorOnLine(i)) return false; // If any right sensor DoesNot sees balck, return (not a sharp right)
  return true; // If left sensors Don't see black AND right sensors do, it's a sharp right
  // Right+center see black, left don't -> 0 0 1 1 1
}

// Rotates robot in place until line is centered. Will be used when the line is not centered
// Takes direction (LEFT or RIGHT) as parameter. Return nothing
inline void rotate(bool direction) {
  float speedLeft = direction == LEFT ? (BASE_SPEED * -0.3) : BASE_SPEED * 0.7; // If turning LEFT -> left motor backwards at -40% 
  float speedRight = direction == RIGHT ? (BASE_SPEED * -0.3) : BASE_SPEED * 0.7; //  If turning RIGHT -> right motor backwards at -40% 
  // one motor forward + one backward = rotation in place
  loopCount = 0;
  while (!isLineCentred() && loopCount < MAX_ROTATION_ATTEMPTS) { 
    // Keep rotating while the line is not centered AND haven't exceeded max attempts to pevent infinite loops
    //  Exit when line is centered or max attempts reached.
    calibratedSensors(sensorValues); // Update sensor values to check current line position
    activateMotor(LEFT, speedLeft); // Drive left motor accordingly
    activateMotor(RIGHT, speedRight); // Drive right motor accordingly
    loopCount++; // Track number of rotation attempts
  }
  loopCount = 0; // Reset counter to zero once the line is found
}

#define SETPOINT ((N_LINE_SENSORS - 1) * 1000 / 2) // 2000
#define KP 0.0003
#define KD 0.030
// PD control system for following the line. Takes desired speed as a parameter
inline void followLine(float speed) {
  int sum = 0; // This will hold total of all sensor readings
  long weightedSum = 0; // This will hold (position-weighted sensor values) needs to be long for larg precise decimals
  for (size_t i = 0; i < N_LINE_SENSORS; i++) { // Loo through all the sensors
    sum += sensorValues[i]; // Accamulate and sum up all the sensors values
    weightedSum += (long)sensorValues[i] * (long)i * 1000L; // Accumulate and sum up all the 
                                                            //  Multiply each sensor value by its position (0-4) × 1000
  }
  int position = (sum == 0) ? SETPOINT : (weightedSum / sum); // If sum = 0 (no line detected) -> use SETPOINT
                                                              // else -> use weighted average position
  int error = position - SETPOINT; // error: How far from center? 
                                   // Negative = left of center
                                   // Positive = right of center 
                                   // 0 = perfectly centered
  
  static int lastError = 0; //  Hold last error between function calls
  static float filteredDerivative = 0.0f; 
  
  if (loopCount == 0) { // First loop after rotation/stop
    lastError = error;  // Set to CURRENT error
    filteredDerivative = 0.0f; // Set to CURRENT error
  }
  
  int rawDerivative = error - lastError; // How fast is error changing?
                                         // Positive = error increasing (moving right)
                                         // Negative = error decreasing (moving left)
                                         // 0 = error stable

  filteredDerivative = 0.7f * rawDerivative + 0.3f * filteredDerivative; //  70% new value, 30% old value to prevents jerky corrections
  lastError = error; // Save the current error
  
  float correction = KP * error + KD * filteredDerivative; // PD control system rule
  
  activateMotor(LEFT, constrain(speed + correction, -MAX_PD_SPEED, MAX_PD_SPEED)); 
  activateMotor(RIGHT, constrain(speed - correction, -MAX_PD_SPEED, MAX_PD_SPEED));
  // Add correction to base speed. For example, if line is right, increase left motor to steer right
  // constrain() limits speed between MIN and MAX to prevent overspeed
}

void loop()
{
	calibratedSensors(sensorValues);
	if (isTEnd())
	{
		stopSmooth();
		delay(10000); 
		loopCount = 0;
		return;
	}
	if (isHardLeft())
	{
		rotate(LEFT);
		return;
	}
	if (isHardRight())
	{
		rotate(RIGHT);
		return;
	}
	// check if the lined centered
	float speed = isLineCentred() ? MAX_PD_SPEED : BASE_SPEED;
	// Drive robot using PD
	followLine(speed);
	// prevent Integer overflows
	loopCount = (loopCount + 1) % 10000;
}

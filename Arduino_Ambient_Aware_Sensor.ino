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

#define CONSOLE_SERIAL_BAUDRATE 9600    // [baud]
#define CONSOLE_INITIAL_DELAY 2000      // [ms] Initial delay after Serial.begin() to ensure communication is initialized on both ends, and we don't miss the first few Serial.print() lines.
#define CONSOLE_PRINTF_BUFFER_SIZE 1024 // [chars] Don't exceed buffersize in serialPrintf() calls (or enlarge buffersize).

#define POLOLU_SERIAL_BAUDRATE 115200      // [baud]
#define POLOLU_SERIAL_DEFAULT_TIMEOUT 1500 // [ms] Default timeout in Stream.h is 1000 [ms]; but our longest operation (AUTO_CALIBRATE) takes ~1000 [ms], so we increase timeout to 1100 [ms].
#define POLOLU_SERIAL_POLL_DELAY 1         // [ms]

#define POLOLU_SIGNATURE_BYTECOUNT 6 // [bytes]
#define POLOLU_SIGNATURE_TIMEOUT 2   // [ms] Response time is around 0.576 [ms] from request sent to reply received.

#define POLOLU_VOLTAGE_BYTECOUNT 2 // [bytes]
#define BATTERY_VOLTAGE_TIMEOUT 3  // [ms] Response time is around 1.322 [ms] from request sent to reply received.

#define LINE_SENSORS_CALIBRATION_BYTECOUNT 1  // [byte]
#define LINE_SENSORS_CALIBRATION_TIMEOUT 1100 // [ms] Response time is around 1000 [ms] from request sent to reply received.
#define N_LINE_SENSORS 5                      // [sensors]
#define LINE_SENSORS_READ_BYTECOUNT 10        // [bytes]
#define LINE_SENSORS_READ_TIMEOUT 4           // [ms] Response time is around 2.37 [ms] from request sent to reply received.
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

////////////
// User code
////////////

// Define pins
#define LDR_PIN A3
#define EXT_LED_G_PIN 20
#define EXT_LED_R_PIN 2
// Set constants
#define MOTOR_SPEED 0.3
#define MAX_TUNNEL_BRIGHTNESS 30
// Alias for readability
#define LEFT 0
#define RIGHT 1

void setup()
{
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
  // Initial calibration
  for (int i = 0; i < 10; i++)
  {
    calibratedSensors(sensorValues);
  }
  sensorAutoCalibrate();

  pinMode(LDR_PIN, INPUT);        // Setup Light sensor pin
  pinMode(EXT_LED_G_PIN, OUTPUT); // Setup LED1 pins
  pinMode(EXT_LED_R_PIN, OUTPUT); // Setup LED2 pins, Assuming that D16 is used for the second LED

  // Initialize global variables
  m1Speed = 0;
  m2Speed = 0;
  loopCount = 1;
}

// Define pins
#define LDR_PIN A3
#define EXT_LED_G_PIN 20
#define EXT_LED_R_PIN 2
// Set constants
#define MOTOR_SPEED 0.3
#define MAX_TUNNEL_BRIGHTNESS 30
// Alias for readability
#define LEFT 0
#define RIGHT 1


void setup()
{
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
  // Initial calibration
  for (int i = 0; i < 10; i++)
  {
    calibratedSensors(sensorValues);
  }
  sensorAutoCalibrate();


  pinMode(LDR_PIN, INPUT);        // Setup Light sensor pin
  pinMode(EXT_LED_G_PIN, OUTPUT); // Setup LED1 pins
  pinMode(EXT_LED_R_PIN, OUTPUT); // Setup LED2 pins, Assuming that D16 is used for the second LED


  // Initialize global variables
  m1Speed = 0;
  m2Speed = 0;
  loopCount = 1;
}


double AmbientBrightness()   // Calibration
{                                    
  float output = analogRead(LDR_PIN); // Read light sensor
  int lightpercent = (1023 - output) / 1023 * 100;
  Serial.print("Light level: ");
  Serial.print(lightpercent);
  Serial.println("%");
  delay(500);
  return lightpercent;
}


void setLeds(bool state)
{
  digitalWrite(EXT_LED_G_PIN, state); //  Turn LEDs OFF
  digitalWrite(EXT_LED_R_PIN, state);
}


void setLeds(bool state)
{
  digitalWrite(EXT_LED_G_PIN, state); //   Turn LEDs ON
  digitalWrite(EXT_LED_R_PIN, state);
}
void driveForward(double speed)
{
  activateMotor(LEFT, MOTOR_SPEED);
  activateMotor(RIGHT, MOTOR_SPEED);
}


void loop()
{
  driveForward();
  if (AmbientBrightness() < MAX_TUNNEL_BRIGHTNESS)
  {
    setLeds(true);
  }
  else
  {
    setLeds(false);
  }
}


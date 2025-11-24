final code
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <util/atomic.h>

// ==========================================
//                CONFIGURATION
// ==========================================
const int BLDC_PIN  = 23;   
const int SERVO_PIN = 10;   

#define FRAME_LENGTH 20000UL 
#define MIN_PULSE    1000
#define MAX_PULSE    2000

enum SystemState {
  PWM_CRITICAL_HIGH,
  PWM_SAFE_LOW
};

// ==========================================
//              GLOBAL VARIABLES
// ==========================================
Servo myServo;
Adafruit_BNO055 bno = Adafruit_BNO055(55);

volatile int escPulseUS = 1500;

// SPI Buffers
// RX: [Sync(1), M_H, M_L, S_H, S_L] = 5 bytes used (rest ignored)
// TX: [AccX, AccY, AccZ, Yaw, YawRate] = 5 floats * 4 bytes = 20 bytes
volatile uint8_t rxBuffer[5];
volatile uint8_t txBuffer[20]; // INCREASED SIZE TO 20
volatile uint8_t byteIndex = 0;
volatile bool packetReady = false;

unsigned long previousImuReadMillis = 0;
const int imuReadInterval = 20; 

// New Variables
float yaw_val = 0;
float last_yaw = 0;
float yaw_rate = 0;
unsigned long lastYawTime = 0;

// ==========================================
//               SPI INTERRUPT
// ==========================================
ISR(SPI_STC_vect)
{
    uint8_t rx = SPDR;

    // 1. SYNC CHECK
    if (rx == 0xAA) { 
        byteIndex = 1;
        SPDR = txBuffer[1]; // Load 2nd byte
        rxBuffer[0] = rx; 
        return; 
    }

    // 2. STORE INCOMING COMMAND
    if (byteIndex < 5) {
       rxBuffer[byteIndex] = rx; 
    }

    // 3. LOAD NEXT OUTGOING BYTE
    byteIndex++;
    if (byteIndex < 20) { // UPDATED LIMIT TO 20
        SPDR = txBuffer[byteIndex];
    } 
    else {
        SPDR = txBuffer[0]; // Loop back
    }

    if (byteIndex == 5) {
        packetReady = true;
    }
}

// ==========================================
//                  SETUP
// ==========================================
void setup()
{
    pinMode(BLDC_PIN, OUTPUT);
    digitalWrite(BLDC_PIN, LOW);
    pinMode(MISO, OUTPUT); 

    SPCR = _BV(SPE) | _BV(SPIE); 
    
    // Clear and Pre-load buffer
    memset((void*)txBuffer, 0, 20);
    SPDR = txBuffer[0]; 

    myServo.attach(SERVO_PIN, 1040, 1960);
    myServo.writeMicroseconds(1500);

    Serial.begin(115200);
    Serial.println("System Init: 5-Float Telemetry Mode");

    if (!bno.begin()) {
        Serial.print("Error: No BNO055 detected!");
        while (1);
    }

    bno.setMode(OPERATION_MODE_NDOF); // NDOF required for Yaw
    delay(20);
    lastYawTime = millis();
}

// ==========================================
//                MAIN LOOP
// ==========================================
void loop()
{
    static unsigned long frameStart = micros();
    unsigned long now = micros();
    
    if (now - frameStart >= FRAME_LENGTH) {
        frameStart = now; 
        digitalWrite(BLDC_PIN, HIGH);
    }

    unsigned long elapsedInFrame = now - frameStart;
    SystemState currentState;

    if (elapsedInFrame < escPulseUS) {
        currentState = PWM_CRITICAL_HIGH;
    } 
    else {
        digitalWrite(BLDC_PIN, LOW); 
        currentState = PWM_SAFE_LOW;
    }

    switch (currentState) {
        
        case PWM_CRITICAL_HIGH:
            break; // Busy wait

        case PWM_SAFE_LOW:
            
            // --- PROCESS SPI COMMANDS ---
            if (packetReady) {
                uint16_t motorVal, servoVal;
                ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                    motorVal  = (rxBuffer[1] << 8) | rxBuffer[2];
                    servoVal  = (rxBuffer[3] << 8) | rxBuffer[4];
                    packetReady = false;
                }
                int rawPulse = map(motorVal, 0, 1023, MIN_PULSE, MAX_PULSE);
                escPulseUS = constrain(rawPulse, MIN_PULSE, MAX_PULSE);
                int servoPulse = map(servoVal, 0, 1000, 1040, 1960);
                myServo.writeMicroseconds(servoPulse);
            }

            // --- READ IMU & UPDATE SPI BUFFER ---
            if (millis() - previousImuReadMillis >= imuReadInterval) {

                if (elapsedInFrame < (FRAME_LENGTH - 3000)) { 
                    
                    previousImuReadMillis = millis();

                    // 1. Read Data
                    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
                    imu::Vector<3> eul = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
                    
                    // 2. Calculate Rate
                    unsigned long nowT = millis();
                    float dt = (nowT - lastYawTime) / 1000.0;
                    if (dt < 0.001) dt = 0.001; // Avoid divide by zero
                    
                    yaw_val = eul.x();
                    yaw_rate = (yaw_val - last_yaw) / dt;
                    
                    // Handle wrap-around (0 to 360 jump) artifact in rate
                    if (yaw_rate > 3000) yaw_rate = 0; // Simple filter for wrap-around spike
                    if (yaw_rate < -3000) yaw_rate = 0;

                    last_yaw = yaw_val;
                    lastYawTime = nowT;

                    // 3. Debug Print
                    Serial.print("BLDC: ");
                    Serial.print(escPulseUS);
                    Serial.print(" ");

                    Serial.print("TX-> Ax:"); Serial.print(accel.x());
                    Serial.print(" Ty-> Ay:"); Serial.print(accel.y());
                    Serial.print(" Tz-> Az:"); Serial.print(accel.z());
                    Serial.print(" Yaw:"); Serial.print(yaw_val);
                    Serial.print(" Rate:"); Serial.println(yaw_rate);

                    // 4. Pack 5 Floats (20 Bytes)
                    float dataToSend[5] = {
                        (float)accel.x(), 
                        (float)accel.y(), 
                        (float)accel.z(),
                        yaw_val,
                        yaw_rate
                    };

                    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                        memcpy((void*)txBuffer, (const void*)dataToSend, 20);
                        
                        // Pre-load byte 0 if bus is idle
                        if (digitalRead(SS) == HIGH) {
                             SPDR = txBuffer[0];
                        }
                    }
                }
            }
            break;
    }
}
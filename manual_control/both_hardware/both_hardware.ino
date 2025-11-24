#include <Servo.h>
#include <util/atomic.h>

// PWM Pins (keep your existing wiring)
#define BLDC_PIN 9   // ESC control
#define SERVO_PIN 10 // Servo control

Servo ESC;        // BLDC motor
Servo myServo;    // Standard servo

// SPI Variables
volatile uint8_t rxBuffer[4];   // [motor_high, motor_low, servo_high, servo_low]
volatile uint8_t byteIndex = 0;
volatile bool packetReady = false;

// SPI Interrupt Handler
ISR(SPI_STC_vect)
{
    static bool synced = false;
    uint8_t rx = SPDR;  // Read incoming byte

    // Always send 0xFF back (dummy response)
    SPDR = 0xFF;

    // Sync detection
    if (rx == 0xAA)
    {
        byteIndex = 0;
        synced = true;
        return;
    }

    // Store data bytes if synced
    if (synced && byteIndex < 4)
    {
        rxBuffer[byteIndex++] = rx;

        if (byteIndex >= 4)
        {
            packetReady = true;
            synced = false;
        }
    }
}

void setup()
{
    // SPI Slave Configuration
    pinMode(MISO, OUTPUT);
    pinMode(MOSI, INPUT);
    pinMode(SCK, INPUT);
    pinMode(SS, INPUT_PULLUP);  // SS must stay HIGH for slave to work

    // Enable SPI in slave mode + interrupt
    SPCR = _BV(SPE) | _BV(SPIE);

    SPDR = 0xFF; // Preload dummy byte

    // Attach servos using your exact physical tested limits
    ESC.attach(BLDC_PIN, 1000, 2000);         // ESC range
    myServo.attach(SERVO_PIN, 1040, 1960);    // Servo range

    // Initialize to safe state
    ESC.writeMicroseconds(1500);   // Neutral (stop)
    myServo.writeMicroseconds(1500); // Center

    Serial.begin(9600);
    Serial.println("Ready: BLDC (pin 9), Servo (pin 10)");
}

void loop()
{
    if (packetReady)
    {
        uint16_t motorValue, servoValue;

        // Atomic read to prevent corruption by SPI ISR
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
            motorValue = (rxBuffer[0] << 8) | rxBuffer[1];  // 0–1023
            servoValue = (rxBuffer[2] << 8) | rxBuffer[3];  // 0–1000
            packetReady = false;
        }

        // Convert directly to microseconds
        int escPulse   = map(motorValue, 0, 1023, 1000, 2000);
        int servoPulse = map(servoValue, 0, 1000, 1040, 1960);

        // Hard safety limits
        escPulse   = constrain(escPulse, 1000, 2000);
        servoPulse = constrain(servoPulse, 1040, 1960);

        // Write to motors
        ESC.writeMicroseconds(escPulse);
        myServo.writeMicroseconds(servoPulse);

        // Debug output
        Serial.print("BLDC: ");
        Serial.print(motorValue);
        Serial.print("   Servo: ");
        Serial.println(servoValue);


        // Serial.print(" → µs: ");
        // Serial.println(escPulse);
        // Serial.print(" → µs: ");
        // Serial.println(servoPulse);
    }
}

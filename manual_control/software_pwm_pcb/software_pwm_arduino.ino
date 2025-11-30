#include <Servo.h>
#include <util/atomic.h>

// --- PIN DEFINITIONS ---
#define BLDC_PIN 23      // Software PWM output
#define SERVO_PIN 10     // Hardware PWM (Timer1)

// --- CONSTANTS ---
#define FRAME_LENGTH 20000UL   // 20 ms frame (ESC requirement)
#define MIN_PULSE 1000
#define MAX_PULSE 2000

// software PWM state
volatile int escPulseUS = 1500;   // pulse width in microseconds

Servo myServo;

// SPI variables
volatile uint8_t rxBuffer[4];
volatile uint8_t byteIndex = 0;
volatile bool packetReady = false;

// --- SPI INTERRUPT ---
ISR(SPI_STC_vect)
{
    static bool synced = false;
    uint8_t rx = SPDR;
    SPDR = 0xFF;

    if (rx == 0xAA) { byteIndex = 0; synced = true; return; }

    if (synced && byteIndex < 4)
    {
        rxBuffer[byteIndex++] = rx;
        if (byteIndex == 4)
        {
            packetReady = true;
            synced = false;
        }
    }
}

// --- BLDC SOFTWARE PWM FUNCTION ---
void BLDC_writeMicroseconds(int pulse)
{
    if (pulse < MIN_PULSE) pulse = MIN_PULSE;
    if (pulse > MAX_PULSE) pulse = MAX_PULSE;

    escPulseUS = pulse;
}

void setup()
{
    pinMode(BLDC_PIN, OUTPUT);
    digitalWrite(BLDC_PIN, LOW);

    // SPI SLAVE CONFIG
    pinMode(MISO, OUTPUT);
    SPCR = _BV(SPE) | _BV(SPIE);
    SPDR = 0xFF;

    // SERVO (hardware PWM)
    myServo.attach(SERVO_PIN, 1040, 1960);
    myServo.writeMicroseconds(1500);

    Serial.begin(9600);
    Serial.println("Ready with improved BLDC Software PWM");
}

// --- MAIN LOOP ---
void loop()
{
    if (packetReady)
    {
        uint16_t motorVal, servoVal;

        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
            motorVal  = (rxBuffer[0] << 8) | rxBuffer[1];
            servoVal  = (rxBuffer[2] << 8) | rxBuffer[3];
            packetReady = false;
        }

        // map ESC pulse
        int escPulse = map(motorVal, 0, 1023, 1000, 2000);
        BLDC_writeMicroseconds(escPulse);

        // servo stays same
        int servoPulse = map(servoVal, 0, 1000, 1040, 1960);
        myServo.writeMicroseconds(servoPulse);
    }

    // ======== BLDC SOFTWARE PWM ENGINE =========
    static unsigned long frameStart = micros();
    unsigned long now = micros();

    if (now - frameStart >= FRAME_LENGTH)
        frameStart += FRAME_LENGTH;   // restart new frame

    if (now - frameStart < escPulseUS)
        digitalWrite(BLDC_PIN, HIGH);
    else
        digitalWrite(BLDC_PIN, LOW);
}

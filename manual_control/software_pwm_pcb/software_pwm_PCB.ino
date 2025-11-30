#include <Servo.h>
#include <util/atomic.h>

// --- PIN DEFINITIONS ---
#define SERVO_PIN 3     // Hardware PWM using Timer2
#define BLDC_PORT PORTE
#define BLDC_DDR  DDRE
#define BLDC_PIN_BIT PE6  // PE6 = Arduino Digital Pin 7

// --- ESC CONSTANTS ---
#define FRAME_LENGTH 20000UL   // 20 ms ESC frame
#define MIN_PULSE 1000
#define MAX_PULSE 2000

volatile int escPulseUS = 1500;   // software PWM width for ESC

Servo myServo;

// SPI variables
volatile uint8_t rxBuffer[4];
volatile uint8_t byteIndex = 0;
volatile bool packetReady = false;

// ---------- SPI INTERRUPT ----------
ISR(SPI_STC_vect)
{
    static bool synced = false;
    uint8_t rx = SPDR; 
    SPDR = 0xFF;        // dummy response

    if (rx == 0xAA) {
        synced = true;
        byteIndex = 0;
        return;
    }

    if (synced && byteIndex < 4) {
        rxBuffer[byteIndex++] = rx;
        if (byteIndex == 4) {
            packetReady = true;
            synced = false;
        }
    }
}

// ---------- ESC SOFTWARE PWM ----------
void BLDC_writeMicroseconds(int pulse)
{
    if (pulse < MIN_PULSE) pulse = MIN_PULSE;
    if (pulse > MAX_PULSE) pulse = MAX_PULSE;
    escPulseUS = pulse;
}

void setup()
{
    // ---------- SOFTWARE PWM OUTPUT PE6 ----------
    BLDC_DDR |= (1 << BLDC_PIN_BIT);      // PE6 output
    BLDC_PORT &= ~(1 << BLDC_PIN_BIT);    // start LOW

    // ---------- SPI SLAVE SETUP ----------
    pinMode(MISO, OUTPUT);
    SPCR = _BV(SPE) | _BV(SPIE);
    SPDR = 0xFF;

    // ---------- HARDWARE SERVO ----------
    myServo.attach(SERVO_PIN, 1040, 1960);
    myServo.writeMicroseconds(1500);

    Serial.begin(9600);
    Serial.println("ATmega2560 READY: Servo=Pin10, BLDC=PE6 (SW PWM)");
}

void loop()
{
    // --- PROCESS SPI PACKET ---
    if (packetReady)
    {
        uint16_t motorVal, servoVal;

        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
            motorVal  = (rxBuffer[0] << 8) | rxBuffer[1];
            servoVal  = (rxBuffer[2] << 8) | rxBuffer[3];
            packetReady = false;
        }

        int escPulse = map(motorVal, 0, 1023, 1000, 2000);
        BLDC_writeMicroseconds(escPulse);

        int servoPulse = map(servoVal, 0, 1000, 1040, 1960);
        myServo.writeMicroseconds(servoPulse);
    }

    // ---------- SOFTWARE PWM ENGINE FOR ESC ----------
    static unsigned long frameStart = micros();
    unsigned long now = micros();

    if (now - frameStart >= FRAME_LENGTH) {
        frameStart += FRAME_LENGTH;
    }

    if (now - frameStart < escPulseUS)
        BLDC_PORT |= (1 << BLDC_PIN_BIT);    // HIGH
    else
        BLDC_PORT &= ~(1 << BLDC_PIN_BIT);   // LOW
}
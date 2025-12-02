/*
 * FINAL FIXED FIRMWARE: ATmega2560
 * 1. Solves "__vector_32" error by removing Servo.h
 * 2. Uses TIMER3 to drive BOTH BLDC (Pin 23) and Servo (Pin 10)
 * 3. Blocking I2C for BNO055 (Stable)
 * 4. SPI Slave for RPi Communication
 */

#define F_CPU 16000000UL
#include <Arduino.h>
#include <util/atomic.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// --- PIN DEFINITIONS ---
#define BLDC_PIN 23      // Software PWM Channel A
#define SERVO_PIN 10     // Software PWM Channel B

// --- CONSTANTS ---
#define MIN_PULSE 1000
#define MAX_PULSE 2000
#define SERVO_MIN 1040
#define SERVO_MAX 1960

// --- BNO055 REGISTERS ---
#define BNO_ADDR            0x28
#define REG_ACCEL_X_LSB     0x08
#define REG_GYRO_Z_LSB      0x18
#define REG_EULER_H_LSB     0x1A 
#define REG_EULER_P_LSB     0x1E 

// --- VARIABLES ---
// IMU Data (Outgoing to RPi)
volatile uint8_t imu_bytes[12];
volatile uint8_t tx_index = 0;

// Motor Data (Incoming from RPi)
volatile uint8_t rxBuffer[4];
volatile uint8_t rx_index = 0;
volatile bool packetReady = false;

// PWM Pulse States (Volatile for ISR)
// Default to mid-point (1500us * 2 ticks/us = 3000 ticks)
volatile uint16_t bldc_ocr_val = 3000; 
volatile uint16_t servo_ocr_val = 3000; 

// ================================================================
// ==================== 1. TWI / I2C DRIVER =======================
// ================================================================

void TWI_init(void) {
    TWSR = 0x00;
    TWBR = 32; // ~100kHz
    TWCR = (1 << TWEN);
}

void TWI_start(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

void TWI_stop(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
    _delay_us(10);
}

void TWI_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

uint8_t TWI_read_NACK(void) {
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

static uint16_t BNO_read16(uint8_t reg) {
    uint8_t l = 0, h = 0;
    TWI_start(); TWI_write((BNO_ADDR << 1) | 0); TWI_write(reg);
    TWI_start(); TWI_write((BNO_ADDR << 1) | 1); l = TWI_read_NACK(); TWI_stop();
    
    TWI_start(); TWI_write((BNO_ADDR << 1) | 0); TWI_write(reg + 1);
    TWI_start(); TWI_write((BNO_ADDR << 1) | 1); h = TWI_read_NACK(); TWI_stop();
    return (uint16_t)((h << 8) | l);
}

void BNO_init(void) {
    _delay_ms(800);
    TWI_start(); TWI_write((BNO_ADDR << 1) | 0); TWI_write(0x3D); TWI_write(0x00); TWI_stop();
    _delay_ms(25);
    TWI_start(); TWI_write((BNO_ADDR << 1) | 0); TWI_write(0x3D); TWI_write(0x0C); TWI_stop();
    _delay_ms(25);
}

// ================================================================
// ==================== 2. SPI SLAVE ==============================
// ================================================================

void SPI_init_slave(void) {
    pinMode(MISO, OUTPUT);
    SPCR = _BV(SPE) | _BV(SPIE); 
    SPDR = 0x00; 
}

ISR(SPI_STC_vect) {
    uint8_t incoming = SPDR;

    // Load next byte to send
    SPDR = imu_bytes[tx_index];
    tx_index++;
    if (tx_index >= 12) tx_index = 0;

    // Receive Logic
    static bool synced = false;
    if (incoming == 0xAA) {
        rx_index = 0;
        synced = true;
        return; 
    }

    if (synced && rx_index < 4) {
        rxBuffer[rx_index++] = incoming;
        if (rx_index == 4) {
            packetReady = true;
            synced = false;
        }
    }
}

// ================================================================
// ==================== 3. DUAL PWM TIMER DRIVER ==================
// ================================================================
/* * Timer 3 (16-bit) controls BOTH pins now.
 * Prescaler 8 -> 0.5us per tick.
 * Frame: 20ms (40,000 ticks).
 * COMPA handles BLDC (Pin 23).
 * COMPB handles Servo (Pin 10).
 */

void PWM_Timer_Init() {
    pinMode(BLDC_PIN, OUTPUT);
    pinMode(SERVO_PIN, OUTPUT);
    digitalWrite(BLDC_PIN, LOW);
    digitalWrite(SERVO_PIN, LOW);

    cli(); 
    TCCR3A = 0; 
    TCCR3B = 0; 
    TCNT3  = 0;

    // Enable Overflow, Compare A (BLDC), Compare B (Servo)
    TIMSK3 = (1 << TOIE3) | (1 << OCIE3A) | (1 << OCIE3B);

    // Start Timer (Prescaler 8)
    TCCR3B |= (1 << CS31); 
    sei(); 
}

// 1. Frame Start (Every 20ms) - Turn BOTH Pins ON
ISR(TIMER3_OVF_vect) {
    digitalWrite(BLDC_PIN, HIGH);
    digitalWrite(SERVO_PIN, HIGH);
    
    // Reset timer to count exactly 20ms (65536 - 40000)
    TCNT3 = 25536; 
    
    // Set Turn-off times based on calculated values
    OCR3A = 25536 + bldc_ocr_val; 
    OCR3B = 25536 + servo_ocr_val; 
}

// 2. BLDC Turn Off
ISR(TIMER3_COMPA_vect) {
    digitalWrite(BLDC_PIN, LOW);
}

// 3. Servo Turn Off
ISR(TIMER3_COMPB_vect) {
    digitalWrite(SERVO_PIN, LOW);
}

// Helper to set BLDC Pulse
void set_BLDC_pulse(int pulse) {
    if (pulse < MIN_PULSE) pulse = MIN_PULSE;
    if (pulse > MAX_PULSE) pulse = MAX_PULSE;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        bldc_ocr_val = pulse * 2; // Convert us to ticks
    }
}

// Helper to set Servo Pulse
void set_Servo_pulse(int pulse) {
    if (pulse < SERVO_MIN) pulse = SERVO_MIN;
    if (pulse > SERVO_MAX) pulse = SERVO_MAX;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        servo_ocr_val = pulse * 2; // Convert us to ticks
    }
}

// ================================================================
// ==================== MAIN LOOP =================================
// ================================================================

void setup() {
    Serial.begin(9600);

    PWM_Timer_Init(); // Starts PWM for Pin 23 and Pin 10
    
    // Arming / Init positions
    set_BLDC_pulse(1000); 
    set_Servo_pulse(1500);

    TWI_init();
    BNO_init();
    
    for(int i=0; i<12; i++) imu_bytes[i] = 0;
    SPI_init_slave();

    Serial.println("System Ready: Dual Manual PWM");
}

void loop() {
    // 1. READ SENSORS (Blocking I2C is now safe)
    uint16_t acc_x = BNO_read16(REG_ACCEL_X_LSB);
    uint16_t acc_y = BNO_read16(REG_ACCEL_X_LSB + 2);
    uint16_t acc_z = BNO_read16(REG_ACCEL_X_LSB + 4);
    uint16_t yaw   = BNO_read16(REG_EULER_H_LSB);
    uint16_t yawr  = BNO_read16(REG_GYRO_Z_LSB);
    uint16_t pitch = BNO_read16(REG_EULER_P_LSB);

    // 2. UPDATE SPI BUFFER
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        imu_bytes[0] = (acc_x >> 8); imu_bytes[1] = (acc_x & 0xFF);
        imu_bytes[2] = (acc_y >> 8); imu_bytes[3] = (acc_y & 0xFF);
        imu_bytes[4] = (acc_z >> 8); imu_bytes[5] = (acc_z & 0xFF);
        imu_bytes[6] = (yaw >> 8);   imu_bytes[7] = (yaw & 0xFF);
        imu_bytes[8] = (yawr >> 8);  imu_bytes[9] = (yawr & 0xFF);
        imu_bytes[10]= (pitch >> 8); imu_bytes[11]= (pitch & 0xFF);

        if (PINB & (1 << PB0)) {
            SPDR = imu_bytes[0];
            tx_index = 1; 
        }
    }

    // 3. APPLY MOTOR COMMANDS
    if (packetReady) {
        uint16_t motorVal, servoVal;

        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            motorVal = (rxBuffer[0] << 8) | rxBuffer[1];
            servoVal = (rxBuffer[2] << 8) | rxBuffer[3];
            packetReady = false;
        }

        int escPulse = map(motorVal, 0, 1023, 1000, 2000);
        set_BLDC_pulse(escPulse);

        int servoPulse = map(servoVal, 0, 1000, 1040, 1960);
        set_Servo_pulse(servoPulse);
    }
}

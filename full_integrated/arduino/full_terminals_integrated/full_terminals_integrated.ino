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
// #define BLDC_PIN 23, PA1      // Software PWM Channel A
// #define SERVO_PIN 10, PB4     // Software PWM Channel B

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

// ================================================================
// ==================== LED CONFIGURATION =========================
// ================================================================

// --- Static LED 1 (Currently Pin 26 / PA4) ---
#define LED1_DDR    DDRA
#define LED1_PORT   PORTA
#define LED1_BIT    PA4

// --- Static LED 2 (Currently Pin 27 / PA5) ---
#define LED2_DDR    DDRA
#define LED2_PORT   PORTA
#define LED2_BIT    PA5

// --- Static LED 3 (Currently Pin 28 / PA6) ---
#define LED3_DDR    DDRA
#define LED3_PORT   PORTA
#define LED3_BIT    PA6

// --- Blinking LED (Currently Pin 29 / PA7) ---
#define LED_BLINK_DDR   DDRA
#define LED_BLINK_PORT  PORTA
#define LED_BLINK_PIN   PINA  // Input register needed for fast toggle
#define LED_BLINK_BIT   PA7

// --- Helper Macros for Readability ---
// These ensure the code below stays clean even if you change ports
#define LEDS_INIT() do { \
    LED1_DDR |= (1 << LED1_BIT); \
    LED2_DDR |= (1 << LED2_BIT); \
    LED3_DDR |= (1 << LED3_BIT); \
    LED_BLINK_DDR |= (1 << LED_BLINK_BIT); \
} while(0)

#define LEDS_STATIC_ON() do { \
    LED1_PORT |= (1 << LED1_BIT); \
    LED2_PORT |= (1 << LED2_BIT); \
    LED3_PORT |= (1 << LED3_BIT); \
} while(0)

#define LED_BLINK_TOGGLE() do { \
    LED_BLINK_PIN |= (1 << LED_BLINK_BIT); \
} while(0)

// --- VARIABLES ---
// IMU Data (Outgoing to RPi)
volatile uint8_t imu_bytes[14];
volatile uint8_t tx_index = 0;

// Motor Data (Incoming from RPi)
volatile uint8_t rxBuffer[4];
volatile uint8_t rx_index = 0;
volatile bool packetReady = false;

// PWM Pulse States (Volatile for ISR)
// Default to mid-point (1500us * 2 ticks/us = 3000 ticks)
volatile uint16_t bldc_ocr_val = 3000; 
volatile uint16_t servo_ocr_val = 3000; 

// I2C Safety Flag
volatile bool i2c_timeout_flag = false;

// Debugging Timer
unsigned long last_print_time = 0;

// --- RPM COUNTER VARIABLES ---
const byte SENSOR_PIN = 2;       // Hardware Interrupt Pin (INT4 on Mega)
const float SLOTS = 18.0;        // 6 Slots * 3 (Sensor Constant) = 18 effective

volatile unsigned long last_pulse_time = 0;
volatile unsigned long pulse_interval = 0;
volatile bool new_pulse_available = false;

// RPM Smoothing
const int RPM_SAMPLES = 20; 
float rpm_history[RPM_SAMPLES];
int rpm_idx = 0;
float total_rpm_sum = 0;
float current_rpm = 0.0;         // Holds the final calculated RPM

// ================================================================
// ==================== 1. SAFE TWI / I2C DRIVER ==================
// ================================================================

// Helper: Waits for hardware flag with timeout (Prevents locking up)
void TWI_wait_safe(void) {
    if (i2c_timeout_flag) return; // If already failed, skip

    uint16_t count = 5000; // Timeout counter (~5-10ms)
    while (!(TWCR & (1 << TWINT))) {
        if (--count == 0) {
            i2c_timeout_flag = true; // Declare system failure
            return;
        }
    }
}

void TWI_init(void) {
    TWSR = 0x00;
    TWBR = 32; // ~100kHz
    TWCR = (1 << TWEN);
}

void TWI_start(void) {
    if (i2c_timeout_flag) return; // Fast fail
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    TWI_wait_safe();
}

void TWI_stop(void) {
    // Stop condition does not set TWINT, so no wait needed
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
    _delay_us(10);
}

void TWI_write(uint8_t data) {
    if (i2c_timeout_flag) return; // Fast fail
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    TWI_wait_safe();
}

uint8_t TWI_read_NACK(void) {
    if (i2c_timeout_flag) return 0; // Fast fail
    TWCR = (1 << TWINT) | (1 << TWEN);
    TWI_wait_safe();
    return TWDR;
}

static uint16_t BNO_read16(uint8_t reg) {
    // 1. Reset Error Flag before starting a new transaction
    i2c_timeout_flag = false;

    uint8_t l = 0, h = 0;

    // --- Low Byte Transaction ---
    TWI_start(); 
    TWI_write((BNO_ADDR << 1) | 0); 
    TWI_write(reg);
    
    TWI_start(); 
    TWI_write((BNO_ADDR << 1) | 1); 
    l = TWI_read_NACK(); 
    TWI_stop();

    // --- High Byte Transaction ---
    TWI_start(); 
    TWI_write((BNO_ADDR << 1) | 0); 
    TWI_write(reg + 1);
    
    TWI_start(); 
    TWI_write((BNO_ADDR << 1) | 1); 
    h = TWI_read_NACK(); 
    TWI_stop();

    // 2. Final Safety Check
    // If ANY part of the sequence above timed out, return 0.
    if (i2c_timeout_flag) {
        return 0; // Sensor Read Failed -> Return safe "0"
    }

    return (uint16_t)((h << 8) | l);
}

void BNO_init(void) {
    _delay_ms(800);
    // Even init needs to be safe, though we ignore the result here
    i2c_timeout_flag = false; 
    TWI_start(); TWI_write((BNO_ADDR << 1) | 0); TWI_write(0x3D); TWI_write(0x00); TWI_stop();
    _delay_ms(25);
    
    i2c_timeout_flag = false;
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
    if (tx_index >= 14) tx_index = 0;

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
    DDRA |= (1 << PA1); // Set Pin 23 (Port A, Bit 1) as Output, replaces pinMode(BLDC_PIN, OUTPUT);
    DDRB |= (1 << PB4); // Set Pin 10 (Port B, Bit 4) as Output, replaces pinMode(SERVO_PIN, OUTPUT);
    PORTA &= ~(1 << PA1); // Executes in 125ns (2 cycles)
    PORTB &= ~(1 << PB4); // Replaces: digitalWrite(SERVO_PIN, LOW);

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
    PORTA |= (1 << PA1); // Executes in 125ns (2 cycles)
    PORTB |= (1 << PB4);
    
    // Reset timer to count exactly 20ms (65536 - 40000)
    TCNT3 = 25536; 
    
    // Set Turn-off times based on calculated values
    OCR3A = 25536 + bldc_ocr_val; 
    OCR3B = 25536 + servo_ocr_val; 

    // --- 2. LED Blink Logic (5 Hz) ---
    static uint8_t blink_counter = 0;
    blink_counter++;
    
    // Toggle every 5th interrupt (100ms)
    if (blink_counter >= 5) {
        blink_counter = 0;
        LED_BLINK_TOGGLE(); // Uses the PINA trick automatically
    }
}

// 2. BLDC Turn Off
ISR(TIMER3_COMPA_vect) {
    PORTA &= ~(1 << PA1); // Executes in 125ns (2 cycles)
}

// 3. Servo Turn Off
ISR(TIMER3_COMPB_vect) {
    PORTB &= ~(1 << PB4); // Replaces: digitalWrite(SERVO_PIN, LOW);
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

    // 2. LED Setup (Uses the flexible macros defined above)
    LEDS_INIT();      // Sets all 4 LED pins to OUTPUT
    LEDS_STATIC_ON(); // Turns on the 3 static LEDs

    TWI_init();
    BNO_init();
    
    for(int i=0; i<12; i++) imu_bytes[i] = 0;
    SPI_init_slave();

    // --- RPM Init ---
    pinMode(SENSOR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), sensor_isr, RISING);
    
    // Initialize smoothing array
    for (int i = 0; i < RPM_SAMPLES; i++) rpm_history[i] = 0;

    Serial.println("System Ready: F1-14th Car Control");
}

void loop() {
// --- 0. RPM CALCULATION LOGIC ---
    unsigned long local_interval = 0;
    bool data_ready = false;
    bool timeout_detected = false; // NEW FLAG
    unsigned long current_time_micros = micros();

    // 1. Safe Read using ATOMIC_BLOCK
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (new_pulse_available) {
            local_interval = pulse_interval;
            new_pulse_available = false;
            data_ready = true;
        }
        
        // Timeout Logic: Check if motor stopped
        if ((current_time_micros - last_pulse_time) > 250000UL) {
             timeout_detected = true; // Set flag to handle outside
        }
    }

    // 2. Compute RPM (Outside Atomic Block)
    if (data_ready && local_interval > 0) {
        // CASE 1: New Data Available -> Calculate & Update Average
        float instantaneous = 60000000.0 / (local_interval * SLOTS);
        
        total_rpm_sum -= rpm_history[rpm_idx];
        rpm_history[rpm_idx] = instantaneous;
        total_rpm_sum += instantaneous;
        rpm_idx = (rpm_idx + 1) % RPM_SAMPLES;
        
        current_rpm = total_rpm_sum / RPM_SAMPLES;
    } 
    else if (timeout_detected) {
        // CASE 2: Motor Stopped (>1s silence) -> Force 0
        current_rpm = 0.0;
        
        // Reset Smoothing History immediately
        if (total_rpm_sum > 0) {
            total_rpm_sum = 0;
            for(int k=0; k<RPM_SAMPLES; k++) rpm_history[k] = 0;
        }
    }
    // CASE 3: No new data, but no timeout -> Do NOTHING. Keep old current_rpm.

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

        // Pack RPM as int16
        int16_t rpm_int = (int16_t)current_rpm;
        imu_bytes[12]= (rpm_int >> 8); imu_bytes[13]= (rpm_int & 0xFF);

        if (PINB & (1 << PB0)) {
            SPDR = imu_bytes[0];
            tx_index = 1; 
        }
    }

    // --- 3. PRINT DEBUG VALUES (Every 100ms) ---
    // Note: Cast to (int16_t) to see negative values correctly
    if (millis() - last_print_time > 100) {
        last_print_time = millis();
        
        Serial.print("AX:"); Serial.print((int16_t)acc_x/100.0);
        Serial.print("\tAY:"); Serial.print((int16_t)acc_y/100.0);
        Serial.print("\tAZ:"); Serial.print((int16_t)acc_z/100.0);
        
        // Euler angles in BNO055 are usually 16 LSB = 1 Degree (Check datasheet config)
        // For now, we print raw signed integers.
        Serial.print("\tYAW:"); Serial.print((int16_t)yaw/16.0);
        Serial.print("\tPITCH:"); Serial.print((int16_t)pitch/16.0);
        Serial.print("\tY_RATE:"); Serial.print((int16_t)yawr/16.0);

        // Debug RPM
        Serial.print("\tRPM:"); Serial.println(current_rpm);
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


// ==================== 4. RPM SENSOR ISR =========================
void sensor_isr() {
    unsigned long now = micros();
    // Debounce: Ignore pulses closer than 100us
    if (now - last_pulse_time > 100) {
        pulse_interval = now - last_pulse_time;
        last_pulse_time = now;
        new_pulse_available = true;
    }
}
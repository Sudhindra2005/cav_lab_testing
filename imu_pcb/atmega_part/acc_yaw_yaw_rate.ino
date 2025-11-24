/*
  ATmega2560 IMU -> RPi (SPI slave)
  - Pre-loads SPDR to eliminate SPI shift/lag
  - Sends MSB first to match RPi reconstruction logic
  - Includes Pitch data
*/

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <stdint.h>

// === BNO055 registers ===
#define BNO_ADDR            0x28
#define REG_ACCEL_X_LSB     0x08
#define REG_GYRO_Z_LSB      0x18
#define REG_EULER_H_LSB     0x1A // Yaw
#define REG_EULER_P_LSB     0x1E // Pitch

// === Packet layout (12 bytes) ===
// MSB First to match RPi logic:
// [Ax_H, Ax_L, Ay_H, Ay_L, Az_H, Az_L, Yaw_H, Yaw_L, Rate_H, Rate_L, Pit_H, Pit_L]
volatile uint8_t imu_bytes[12];
volatile uint8_t spi_byte_index = 0;

// ----------------- TWI (I2C) -----------------
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
    // Read LSB
    TWI_start(); TWI_write((BNO_ADDR << 1) | 0); TWI_write(reg);
    TWI_start(); TWI_write((BNO_ADDR << 1) | 1); l = TWI_read_NACK(); TWI_stop();
    // Read MSB
    TWI_start(); TWI_write((BNO_ADDR << 1) | 0); TWI_write(reg + 1);
    TWI_start(); TWI_write((BNO_ADDR << 1) | 1); h = TWI_read_NACK(); TWI_stop();
    return (uint16_t)((h << 8) | l);
}

void BNO_init(void) {
    _delay_ms(800); // Boot time
    // Config Mode
    TWI_start(); TWI_write((BNO_ADDR << 1) | 0); TWI_write(0x3D); TWI_write(0x00); TWI_stop();
    _delay_ms(25);
    // NDOF Mode
    TWI_start(); TWI_write((BNO_ADDR << 1) | 0); TWI_write(0x3D); TWI_write(0x0C); TWI_stop();
    _delay_ms(25);
}

// ----------------- SPI Slave -----------------
void SPI_init_slave(void) {
    DDRB |= (1 << PB3);  // MISO output
    DDRB &= ~((1 << PB0) | (1 << PB1) | (1 << PB2)); // SS, SCK, MOSI inputs
    SPCR = (1 << SPE) | (1 << SPIE); // Enable SPI + Interrupt
}

ISR(SPI_STC_vect) {
    uint8_t rx = SPDR; // Read to clear flag

    // Load the NEXT byte immediately
    SPDR = imu_bytes[spi_byte_index];
    
    spi_byte_index++;
    if (spi_byte_index >= 12) spi_byte_index = 0;
}

// ---------------------- MAIN ----------------------
int main(void) {
    TWI_init();
    SPI_init_slave();
    BNO_init();

    // Init buffer
    for (uint8_t i = 0; i < 12; ++i) imu_bytes[i] = 0;
    
    // Initial pre-load to ensure Byte 0 is ready for the very first transaction
    SPDR = imu_bytes[0];
    spi_byte_index = 1;

    sei();

    while (1) {
        uint16_t acc_x = BNO_read16(REG_ACCEL_X_LSB);
        uint16_t acc_y = BNO_read16(REG_ACCEL_X_LSB + 2);
        uint16_t acc_z = BNO_read16(REG_ACCEL_X_LSB + 4);
        uint16_t yaw   = BNO_read16(REG_EULER_H_LSB);
        uint16_t yawr  = BNO_read16(REG_GYRO_Z_LSB);
        uint16_t pitch = BNO_read16(REG_EULER_P_LSB); // Added Pitch

        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            // PACK MSB FIRST (Matches RPi Logic)
            imu_bytes[0] = (uint8_t)((acc_x >> 8) & 0xFF);
            imu_bytes[1] = (uint8_t)(acc_x & 0xFF);

            imu_bytes[2] = (uint8_t)((acc_y >> 8) & 0xFF);
            imu_bytes[3] = (uint8_t)(acc_y & 0xFF);

            imu_bytes[4] = (uint8_t)((acc_z >> 8) & 0xFF);
            imu_bytes[5] = (uint8_t)(acc_z & 0xFF);

            imu_bytes[6] = (uint8_t)((yaw >> 8) & 0xFF);
            imu_bytes[7] = (uint8_t)(yaw & 0xFF);

            imu_bytes[8] = (uint8_t)((yawr >> 8) & 0xFF);
            imu_bytes[9] = (uint8_t)(yawr & 0xFF);

            imu_bytes[10] = (uint8_t)((pitch >> 8) & 0xFF);
            imu_bytes[11] = (uint8_t)(pitch & 0xFF);

            // --- CRITICAL SYNC FIX ---
            // If the RPi is NOT talking (SS pin PB0 is HIGH), 
            // reset the index and pre-load the first byte.
            // This guarantees that when RPi pulls SS Low, Byte 0 is waiting in SPDR.
            if (PINB & (1 << PB0)) {
                SPDR = imu_bytes[0];
                spi_byte_index = 1; // Prepare index for the second byte
            }
        }
        _delay_ms(20);
    }
    return 0;
}
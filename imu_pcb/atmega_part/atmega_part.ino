#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

void BNO_init(void) {
// ---- Include your BNO_init() from earlier ----
#define BNO055_ADDR           0x28

#define BNO055_PAGE_ID        0x07
#define BNO055_OPR_MODE       0x3D
#define BNO055_PWR_MODE       0x3E

#define BNO055_MODE_CONFIG    0x00
#define BNO055_MODE_NDOF      0x0C

#define BNO055_POWER_NORMAL   0x00

#define EULER_H_LSB           0x1A
#define EULER_H_MSB           0x1B
}

// ----------- SPI PACKET BUFFER -----------
volatile uint16_t yaw_raw_spi = 0;    // shared between I2C task and SPI ISR

// -------------- SPI SLAVE INIT --------------
void SPI_init_slave(void)
{
    DDRB |= (1 << PB3);  // MISO output
    DDRB &= ~((1 << PB0) | (1 << PB1) | (1 << PB2)); // SS, SCK, MOSI as input

    SPCR = (1 << SPE) | (1 << SPIE);  // Enable SPI + SPI interrupt
}

// -------------- SPI ISR ----------------------
ISR(SPI_STC_vect)
{
    // When the Pi clocks SPI, we send the current yaw_low first, then yaw_high
    static uint8_t state = 0;

    if (state == 0) {
        SPDR = yaw_raw_spi & 0xFF;       // LSB first
        state = 1;
    } else {
        SPDR = (yaw_raw_spi >> 8) & 0xFF; // MSB next
        state = 0;
    }
}

// ================= I2C CODE (same as before) =================
void TWI_init(void) {
    TWSR = 0x00;
    TWBR = 32;
    TWCR = (1<<TWEN);
}

void TWI_start(void) {
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
}

void TWI_stop(void) {
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
}

void TWI_write(uint8_t data) {
    TWDR = data;
    TWCR = (1<<TWINT)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
}

uint8_t TWI_read_NACK(void) {
    TWCR = (1<<TWINT)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
    return TWDR;
}

uint16_t BNO_read16(uint8_t reg)
{
    TWI_start();
    TWI_write(0x28 << 1);
    TWI_write(reg);

    TWI_start();
    TWI_write((0x28 << 1) | 1);
    uint8_t l = TWI_read_NACK();
    TWI_stop();

    TWI_start();
    TWI_write(0x28 << 1);
    TWI_write(reg + 1);

    TWI_start();
    TWI_write((0x28 << 1) | 1);
    uint8_t h = TWI_read_NACK();
    TWI_stop();

    return (h << 8) | l;
}


// --------------------- MAIN ---------------------
int main()
{
    TWI_init();
    BNO_init();
    SPI_init_slave();
    sei();   // enable interrupts

    while (1)
    {
        uint16_t yaw_raw = BNO_read16(0x1A);
        yaw_raw_spi = yaw_raw;  // <-- shared with SPI ISR

        _delay_ms(30);
    }
}

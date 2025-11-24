#include <iostream>
#include <iomanip>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstring>
#include <cstdint>
#include <cmath>

#define SPI_DEVICE "/dev/spidev0.0"
#define SPI_SPEED  500000
#define SLEEP_US   20000    // 20 ms (50Hz)

// Helper to combine bytes based on your observed Endianness
// Previous logs showed rx[0]=MSB, rx[1]=LSB due to SPI lag
static inline int16_t combine(uint8_t msb, uint8_t lsb) {
    return (int16_t)((msb << 8) | lsb);
}

int main() {
    int fd = open(SPI_DEVICE, O_RDWR);
    if (fd < 0) {
        std::cerr << "Failed to open SPI device.\n";
        return -1;
    }

    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    uint32_t speed = SPI_SPEED;

    ioctl(fd, SPI_IOC_WR_MODE, &mode);
    ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    // We need to read 12 bytes (6 values * 2 bytes)
    // tx is just dummy zeros to push the clock
    uint8_t tx[12] = {0};
    uint8_t rx[12] = {0};

    std::cout << "Starting Full IMU Reader..." << std::endl;
    std::cout << "Format: Ax  Ay  Az  (m/s^2) | Yaw  Rate  Pitch (deg)" << std::endl;

    while (true) {
        struct spi_ioc_transfer tr;
        std::memset(&tr, 0, sizeof(tr));
        tr.tx_buf = (unsigned long)tx;
        tr.rx_buf = (unsigned long)rx;
        tr.len = 12; // 12 bytes total
        tr.speed_hz = SPI_SPEED;
        tr.bits_per_word = 8;

        if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
            std::cerr << "SPI Transfer failed\n";
            return -1;
        }

        // --- RECONSTRUCTION ---
        // Array Layout from ATmega: 
        // 0:Ax, 1:Ay, 2:Az, 3:Yaw, 4:Rate, 5:Pitch
        // Because of the 1-byte SPI lag observed previously, 
        // we treat the pair as [MSB, LSB].
        
        int16_t raw_ax    = combine(rx[0], rx[1]);
        int16_t raw_ay    = combine(rx[2], rx[3]);
        int16_t raw_az    = combine(rx[4], rx[5]);
        int16_t raw_yaw   = combine(rx[6], rx[7]);
        int16_t raw_rate  = combine(rx[8], rx[9]);
        int16_t raw_pitch = combine(rx[10], rx[11]);

        // --- CONVERSION ---
        // Accel: 1 m/s^2 = 100 LSB
        // Angles/Rate: 1 Degree = 16 LSB
        float ax = raw_ax / 100.0f;
        float ay = raw_ay / 100.0f;
        float az = raw_az / 100.0f;
        
        float yaw = raw_yaw / 16.0f;
        float rate = raw_rate / 16.0f;
        float pitch = raw_pitch / 16.0f;

        // --- PRINTING ---
        // Using fixed width for clean dashboard effect
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "Ax:" << std::setw(6) << ax << " "
                  << "Ay:" << std::setw(6) << ay << " "
                  << "Az:" << std::setw(6) << az << " | "
                  << "Y:"  << std::setw(6) << yaw << " "
                  << "R:"  << std::setw(6) << rate << " "
                  << "P:"  << std::setw(6) << pitch 
                  << "      \r" << std::flush;

        usleep(SLEEP_US);
    }

    close(fd);
    return 0;
}
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstring>
#include <vector>
#include <algorithm>
#include <cmath>     // Added for std::abs
#include <cstdint>   // Added for uint8_t types

#define SPI_DEVICE "/dev/spidev0.0"
#define SPI_SPEED  500000
#define SLEEP_US   50000    // 50 ms

// Helper to combine bytes
// param lsb: The Low Byte
// param msb: The High Byte
static inline uint16_t combine_lsb_msb(uint8_t lsb, uint8_t msb) {
    return (uint16_t)lsb | ((uint16_t)msb << 8);
}

int main() {
    int fd = open(SPI_DEVICE, O_RDWR);
    if (fd < 0) {
        std::cerr << "Failed to open SPI device " << SPI_DEVICE << " (are you root?)\n";
        return -1;
    }

    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    uint32_t speed = SPI_SPEED;

    ioctl(fd, SPI_IOC_WR_MODE, &mode);
    ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    uint8_t tx[2] = {0x00, 0x00};
    uint8_t rx[2] = {0,0};

    int prev_raw = -1;
    int64_t continuous_raw = 0;

    std::cout << "Starting Yaw Debugger (Byte Order Fixed)..." << std::endl;

    while (true) {
        // Take 3 quick samples and use median to reduce spikes
        std::vector<uint16_t> samples;
        for (int i=0; i<3; i++){
            struct spi_ioc_transfer tr;
            std::memset(&tr, 0, sizeof(tr));
            tr.tx_buf = (unsigned long)tx;
            tr.rx_buf = (unsigned long)rx;
            tr.len = 2;
            tr.speed_hz = SPI_SPEED;
            tr.bits_per_word = 8;

            if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
                std::cerr << "SPI Transfer failed\n";
                close(fd);
                return -1;
            }

            // --- THE FIX IS HERE ---
            // Based on your logs: rx[0] is MSB (stable), rx[1] is LSB (volatile).
            // combine_lsb_msb expects (LSB, MSB).
            uint16_t raw = combine_lsb_msb(rx[1], rx[0]); 
            
            samples.push_back(raw);
            usleep(1000); // short gap between the 3 samples
        }

        // Median of 3
        std::sort(samples.begin(), samples.end());
        uint16_t raw = samples[1];

        // Convert to degrees (16 LSB = 1 Degree)
        float deg = raw / 16.0f; 
        
        float deg_signed = deg;
        if (deg_signed > 180.0f) deg_signed -= 360.0f;

        // Detect out-of-range (BNO055 Yaw 0-360 maps to 0-5760 raw)
        // We allow slightly over 5760 for jitter, but not much.
        bool out_of_range = (raw > 5800); 

        // Detect very large jumps
        int delta = 0;
        if (prev_raw >= 0) delta = (int)raw - prev_raw;

        bool big_jump = false;
        // If absolute raw delta > 2000 (~125 degrees) between successive averaged samples
        if (prev_raw >= 0 && std::abs(delta) > 2000) big_jump = true;

        // Unwrap for continuous angle logic
        if (prev_raw < 0) {
            continuous_raw = raw;
        } else {
            int d = (int)raw - prev_raw;
            // Handle wrap-around at 0/360 boundary (Raw 0 <-> 5760)
            if (d > 2880) d -= 5760;
            else if (d < -2880) d += 5760;
            continuous_raw += d;
        }
        prev_raw = raw;
        double deg_continuous = continuous_raw / 16.0;

        // Print debug info
        // We use \r to overwrite line if you prefer, or endl for scrolling log
        std::cout << " Yaw angle =" << deg;

        if (out_of_range) std::cout << "  [OUT_OF_RANGE]";
        if (big_jump) std::cout << "  [BIG_JUMP delta=" << delta << "]";
        
        std::cout << std::endl;

        usleep(SLEEP_US);
    }

    close(fd);
    return 0;
}

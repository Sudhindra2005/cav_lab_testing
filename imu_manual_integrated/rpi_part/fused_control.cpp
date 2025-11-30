#include <iostream>

#include <iomanip>

#include <cstring>

#include <unistd.h>

#include <sys/socket.h>

#include <netinet/in.h>

#include <arpa/inet.h>

#include <fcntl.h>

#include <linux/spi/spidev.h>

#include <sys/ioctl.h>

#include <cmath>


#define UDP_PORT 8888

#define SPI_DEVICE "/dev/spidev0.0"

#define SPI_SPEED  500000 // 500kHz matches ATmega capabilities


// Helper to combine MSB (High) and LSB (Low) bytes into signed 16-bit integer

static inline int16_t combine(uint8_t msb, uint8_t lsb) {

    return (int16_t)((msb << 8) | lsb);

}


int main() {

    // ---------------------------------------------------------

    // 1. UDP SOCKET SETUP

    // ---------------------------------------------------------

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);

    if (sockfd < 0) {

        perror("Socket creation failed");

        return 1;

    }


    sockaddr_in serv_addr{};

    serv_addr.sin_family = AF_INET;

    serv_addr.sin_port = htons(UDP_PORT);

    serv_addr.sin_addr.s_addr = INADDR_ANY;


    if (bind(sockfd, (sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {

        perror("Socket bind failed (Port 8888 busy?)");

        close(sockfd);

        return 1;

    }


    // ---------------------------------------------------------

    // 2. SPI SETUP

    // ---------------------------------------------------------

    int spi_fd = open(SPI_DEVICE, O_RDWR);

    if (spi_fd < 0) {

        perror("Failed to open SPI (Did you run with sudo?)");

        close(sockfd);

        return 1;

    }


    uint8_t mode = SPI_MODE_0;

    uint8_t bits = 8;

    uint32_t speed = SPI_SPEED;


    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) == -1 ||

        ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) == -1 ||

        ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1) {

        perror("SPI Configuration failed");

        close(spi_fd);

        close(sockfd);

        return 1;

    }


    std::cout << "==========================================" << std::endl;

    std::cout << " RPI MASTER: Timer2 BLDC + Full IMU Sync  " << std::endl;

    std::cout << " Waiting for Joystick...                  " << std::endl;

    std::cout << "==========================================" << std::endl;


    // ---------------------------------------------------------

    // 3. MAIN LOOP

    // ---------------------------------------------------------

    while (true) {

        // --- A. Receive Joystick Data (Blocking) ---

        float commands[2]; // [Motor, Servo]

        sockaddr_in client_addr{};

        socklen_t client_len = sizeof(client_addr);


        // Blocks here until PC sends packet

        int n = recvfrom(sockfd, commands, sizeof(commands), 0,

                         (sockaddr*)&client_addr, &client_len);


        if (n == sizeof(commands)) {

            float motor_in = commands[0];

            float servo_in = commands[1];


            // --- B. Prepare Outgoing Data (TX) ---

            // Map float [-1, 1] -> PWM integers

            // Motor: 0-1023 (Mapped to 1000-2000us on AVR)

            // Servo: 0-1000 (Mapped to 1040-1960us on AVR)

            

            // Safety Clamp

            if (motor_in > 1.0f) motor_in = 1.0f; if (motor_in < -1.0f) motor_in = -1.0f;

            if (servo_in > 1.0f) servo_in = 1.0f; if (servo_in < -1.0f) servo_in = -1.0f;


            uint16_t motor_val = (uint16_t)((motor_in + 1.0f) * 511.5f);

            uint16_t servo_val = (uint16_t)((servo_in + 1.0f) * 500.0f);


            // Construct 12-byte TX buffer

            // We must send 12 bytes to clock out the 12 bytes of IMU data.

            // Bytes 0-4 contain the command. Bytes 5-11 are dummy zeros.

            uint8_t tx[12] = {0};

            uint8_t rx[12] = {0};


            tx[0] = 0xAA;                   // Sync Byte

            tx[1] = (motor_val >> 8) & 0xFF; // Motor High

            tx[2] = motor_val & 0xFF;        // Motor Low

            tx[3] = (servo_val >> 8) & 0xFF; // Servo High

            tx[4] = servo_val & 0xFF;        // Servo Low

            // tx[5]..tx[11] are 0


            // --- C. SPI Transfer (Full Duplex) ---

   // --- C. SPI Transfer (Byte-by-Byte with Delay) ---

            // Instead of sending 12 bytes in one blast, we send them one by one

            // with a small pause to let the ATmega ISR finish.

            

            struct spi_ioc_transfer tr[12]; // Array of transfers

            memset(tr, 0, sizeof(tr));


            for (int i = 0; i < 12; i++) {

                tr[i].tx_buf = (unsigned long)&tx[i];

                tr[i].rx_buf = (unsigned long)&rx[i];

                tr[i].len = 1;              // Send 1 byte at a time

                tr[i].speed_hz = speed;

                tr[i].bits_per_word = bits;

                tr[i].delay_usecs = 20;     // WAIT 20us between bytes!

                tr[i].cs_change = 0;        // Keep CS low continuously

                

                // For the very last byte, normally CS goes high automatically, 

                // but setting cs_change=0 on earlier packets with spidev usually keeps it low.

                // If your driver is weird, you might need cs_change=1 for intermediate bytes 

                // to force it to stay selected, but usually default behavior + delay is fine.

            }


            // Send all 12 struct messages in one ioctl call

            if (ioctl(spi_fd, SPI_IOC_MESSAGE(12), tr) < 0) {

                perror("SPI Transfer Error");

                break;

            }


            // --- D. Process Incoming IMU Data (RX) ---

            // ATmega Order: Ax, Ay, Az, Yaw, RateZ, Pitch

            // All MSB first.

            

            int16_t raw_ax    = combine(rx[0], rx[1]);

            int16_t raw_ay    = combine(rx[2], rx[3]);

            int16_t raw_az    = combine(rx[4], rx[5]);

            int16_t raw_yaw   = combine(rx[6], rx[7]);

            int16_t raw_ratez = combine(rx[8], rx[9]);

            int16_t raw_pitch = combine(rx[10], rx[11]);


            // Convert to physical units (matching ATmega scale logic)

            float ax = raw_ax / 100.0f;

            float ay = raw_ay / 100.0f;

            float az = raw_az / 100.0f;

            

            float yaw   = raw_yaw / 16.0f;

            if (yaw < 0) yaw += 360.0f; // Normalize 0-360


            float ratez = raw_ratez / 16.0f;

            float pitch = raw_pitch / 16.0f;


            // --- E. Dashboard Display ---

            printf("\033[2J\033[H"); // ANSI Clear Screen + Home

            printf("=== RPi Robot Controller ===\n");

            printf("[UDP Input] Motor: %6.3f | Servo: %6.3f\n", motor_in, servo_in);

            printf("[SPI Sent]  Motor: %4d   | Servo: %4d\n", motor_val, servo_val);

            printf("--------------------------------\n");

            printf("[IMU Data]  (Received simultaneously via SPI)\n");

            printf("   ACCEL (m/s^2) : X=%6.2f  Y=%6.2f  Z=%6.2f\n", ax, ay, az);

            printf("   ATTITUDE (deg): Yaw=%6.2f Pitch=%6.2f\n", yaw, pitch);

            printf("   GYRO (deg/s)  : RateZ=%6.2f\n", ratez);

            printf("--------------------------------\n");

            

            fflush(stdout);

        }

    }


    close(spi_fd);

    close(sockfd);

    return 0;

}

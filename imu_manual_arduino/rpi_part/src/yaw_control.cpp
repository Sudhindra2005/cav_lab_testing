#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <cstdint>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>

#define PORT 8888

int main() {
    // --- UDP SETUP (Same as before) ---
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in serv_addr{};
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    bind(sockfd, (sockaddr*)&serv_addr, sizeof(serv_addr));

    // --- SPI SETUP ---
    const char *dev = "/dev/spidev0.0";
    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    uint32_t speed = 50000; // Keep at 50kHz for stability

    int spi_fd = open(dev, O_RDWR);
    ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
    ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    printf("RPi SPI Receiver Running. Expecting 5 Floats (20 Bytes)...\n");

    while (true) {
        float commands[2];
        sockaddr_in client_addr{};
        socklen_t client_len = sizeof(client_addr);

        int n = recvfrom(sockfd, commands, sizeof(commands), 0,
                        (sockaddr*)&client_addr, &client_len);

        if (n == sizeof(commands)) {
            float motor = commands[0];
            float servo = commands[1];

            uint16_t motor_pwm = (uint16_t)((motor + 1.0) * 511.5);
            uint16_t servo_pwm = (uint16_t)((servo + 1.0) * 500);

            // --- SPI TRANSACTION ---
            // 1 Byte Sync + 20 Bytes Data = 21 Bytes Total
            uint8_t tx[21] = { 0 };
            uint8_t rx[21] = { 0 };

            // Command Header
            tx[0] = 0xAA;
            tx[1] = (motor_pwm >> 8) & 0xFF;
            tx[2] = motor_pwm & 0xFF;
            tx[3] = (servo_pwm >> 8) & 0xFF;
            tx[4] = servo_pwm & 0xFF;

            struct spi_ioc_transfer tr = {
                .tx_buf = (unsigned long)tx,
                .rx_buf = (unsigned long)rx,
                .len = sizeof(tx),
                .speed_hz = speed,
                .bits_per_word = bits,
            };

            if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) == -1) {
                perror("SPI Error");
                break;
            }

            // --- DECODE DATA ---
            // We expect 5 floats (20 bytes). 
            // Due to pre-loading on Arduino, valid data starts at rx[0].
            float sensorData[5];
            memcpy(sensorData, &rx[0], 20);

            float ax = sensorData[0];
            float ay = sensorData[1];
            float az = sensorData[2];
            float yaw = sensorData[3];
            float rate = sensorData[4];

            // Print Clean Output
            // \r allows overwriting the line for a dashboard effect
            printf("MOT:%4d SRV:%4d | Ax:%5.2f Ay:%5.2f Az:%5.2f | Yaw:%6.1f Rate:%6.1f\n", 
                   motor_pwm, servo_pwm, ax, ay, az, yaw, rate);

            // Small delay to prevent flooding Arduino
            usleep(5000); 
        }
    }

    close(spi_fd);
    close(sockfd);
    return 0;
}

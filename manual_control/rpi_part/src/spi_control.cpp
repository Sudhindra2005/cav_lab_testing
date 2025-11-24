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
#include <string.h>
#include <sys/ioctl.h>   #include <unistd.h>

#define PORT 8888

int main() {
    // UDP Socket Setup
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("socket");
        return 1;
    }

    sockaddr_in serv_addr{};
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);
    serv_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sockfd, (sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        perror("bind");
        close(sockfd);
        return 1;
    }

    // SPI Setup (ONCE, outside the loop)
    const char *dev = "/dev/spidev0.0";
    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    uint32_t speed = 500000;

    int spi_fd = open(dev, O_RDWR);
    if (spi_fd < 0) {
        perror("open");
        close(sockfd);
        return 1;
    }

    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) == -1) {
        perror("ioctl(SPI_IOC_WR_MODE)");
        close(spi_fd);
        close(sockfd);
        return 1;
    }
    if (ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) == -1) {
        perror("ioctl(SPI_IOC_WR_BITS_PER_WORD)");
        close(spi_fd);
        close(sockfd);
        return 1;
    }
    if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1) {
        perror("ioctl(SPI_IOC_WR_MAX_SPEED_HZ)");
        close(spi_fd);
        close(sockfd);
        return 1;
    }

    // Main Loop
    while (true) {
        float commands[2];
        sockaddr_in client_addr{};
        socklen_t client_len = sizeof(client_addr);

        int n = recvfrom(sockfd, commands, sizeof(commands), 0,
                        (sockaddr*)&client_addr, &client_len);

        if (n == sizeof(commands)) {
            float motor = commands[0];
            float servo = commands[1];

            // Scale [-1, 1] → [0, 1023] (BLDC) and [0, 1000] (servo)
            uint16_t motor_pwm = (uint16_t)((motor + 1.0) * 511.5);  // 0–1023
            uint16_t servo_pwm = (uint16_t)((servo + 1.0) * 500);  // 0–1000

            // Packet: Sync + Motor (16-bit) + Servo (16-bit)
            uint8_t tx[5] = {
                0xAA,                       // Sync byte
                (motor_pwm >> 8) & 0xFF,    // Motor high byte
                motor_pwm & 0xFF,           // Motor low byte
                (servo_pwm >> 8) & 0xFF,    // Servo high byte
                servo_pwm & 0xFF            // Servo low byte
            };
            uint8_t rx[5] = { 0 };

            struct spi_ioc_transfer tr = {
                .tx_buf = (unsigned long)tx,
                .rx_buf = (unsigned long)rx,
                .len = sizeof(tx),
                .speed_hz = speed,
                .bits_per_word = bits,
            };

            if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) == -1) {
                perror("SPI_IOC_MESSAGE");
                break;  // Exit loop on SPI failure
            }

            // Verify response (Arduino echoes sync byte)
            if (rx[0] == 0xAA) {
                printf("Sent Motor: %d, Servo: %d\n", motor_pwm, servo_pwm);
            }
        }
    }

    // Cleanup
    close(spi_fd);
    close(sockfd);
    return 0;
}

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
#include <sys/ioctl.h>

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

            uint16_t motor_pwm = (uint16_t)((motor + 1.0) * 511.5);
            uint16_t servo_pwm = (uint16_t)((servo + 1.0) * 500);

// ... (Includes and Setup same as before) ...

            // PACKET STRUCTURE:
            // TX: [0xAA, M_H, M_L, S_H, S_L, 0, 0, 0, 0, 0, 0, 0, 0]
            // RX: [D_0,  D_1, ... D_11,  Ign, Ign... ] 
            // With pre-loading, the byte we receive WHILE sending 0xAA is valid Data[0]!
            
            uint8_t tx[13] = { 0 };
            uint8_t rx[13] = { 0 };

            // Fill Command Header
            tx[0] = 0xAA;
            tx[1] = (motor_pwm >> 8) & 0xFF;
            tx[2] = motor_pwm & 0xFF;
            tx[3] = (servo_pwm >> 8) & 0xFF;
            tx[4] = servo_pwm & 0xFF;

            struct spi_ioc_transfer tr = {
                .tx_buf = (unsigned long)tx,
                .rx_buf = (unsigned long)rx,
                .len = sizeof(tx),
                .speed_hz = 50000, // Keep this slow (50kHz) for now!
                .bits_per_word = bits,
            };

            if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) == -1) {
                perror("SPI Error");
                break;
            }

            // --- DEBUG: PRINT RAW BYTES (Crucial Step) ---
            printf("RAW HEX: ");
            for(int i=0; i<12; i++) {
                printf("%02X ", rx[i]);
            }
            printf("\n");

            // --- DECODE ---
            // Because of pre-loading, valid data starts at rx[0]
            float received_accel[3];
            memcpy(received_accel, &rx[0], 12); 

            printf("Sent -> M:%d S:%d | Accel -> X: %.2f Y: %.2f Z: %.2f\n", 
                   motor_pwm, servo_pwm, 
                   received_accel[0], received_accel[1], received_accel[2]);

            usleep(5000);
        }
    }


    // Cleanup
    close(spi_fd);
    close(sockfd);
    return 0;
}

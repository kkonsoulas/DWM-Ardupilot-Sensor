#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include <signal.h>
#include "dwm_api.h"
#include "hal.h"
#include "hal_log.h"
#include "../test_util/test_util.h"

#define UDP_IP "127.0.0.1" // Python script IP
#define UDP_PORT 5005      // Python script port

struct timeval tv;
uint64_t ts_curr = 0;
uint64_t ts_last = 0;
volatile uint8_t data_ready;
static volatile sig_atomic_t keep_running = 1;
int udp_fd = -1; // UDP socket file descriptor

void signal_handler(int sig)
{
    printf("Signal %d received, closing UDP socket\n", sig);
    if (udp_fd != -1) {
        close(udp_fd);
        udp_fd = -1;
    }
    keep_running = 0;
}

// Helper to write 64-bit integer in big endian (network byte order)
void write_int64_be(uint8_t* buffer, int64_t value) {
    for (int i = 7; i >= 0; i--) {
        buffer[i] = value & 0xFF;
        value >>= 8;
    }
}

// Helper to write 32-bit integer in big endian
void write_int32_be(uint8_t* buffer, int32_t value) {
    buffer[0] = (value >> 24) & 0xFF;
    buffer[1] = (value >> 16) & 0xFF;
    buffer[2] = (value >> 8) & 0xFF;
    buffer[3] = value & 0xFF;
}

void send_packed_data(int udp_fd, struct sockaddr_in* server_addr,
                      int64_t tv_sec, int64_t tv_usec,
                      int32_t x, int32_t y, int32_t z, uint8_t qf) {
    uint8_t buffer[29];  // Exact size

    int offset = 0;

    // Write tv_sec (8 bytes)
    write_int64_be(buffer + offset, tv_sec);
    offset += 8;

    // Write tv_usec (8 bytes)
    write_int64_be(buffer + offset, tv_usec);
    offset += 8;

    // Write x, y, z (4 bytes each)
    write_int32_be(buffer + offset, x); offset += 4;
    write_int32_be(buffer + offset, y); offset += 4;
    write_int32_be(buffer + offset, z); offset += 4;

    // Write qf (1 byte)
    buffer[offset++] = qf;

    // Send over UDP
    sendto(udp_fd, buffer, offset, 0, (struct sockaddr*)server_addr, sizeof(*server_addr));
}

int get_loc(struct sockaddr_in* server_addr)
{
    int rv, err_cnt = 0;
    dwm_loc_data_t loc;
    dwm_pos_t pos;
    loc.p_pos = &pos;
    rv = Test_CheckTxRx(dwm_loc_get(&loc));
    // dwm_loc_get(&loc);

    gettimeofday(&tv, NULL);

    
    // char buffer[128];
    // snprintf(buffer, sizeof(buffer), "%ld.%06ld,%d,%d,%d,%u,",
        // tv.tv_sec, tv.tv_usec, loc.p_pos->x, loc.p_pos->y, loc.p_pos->z, loc.p_pos->qf);
        
    // Send data over UDP

    // sendto(udp_fd, buffer, strlen(buffer), 0, (struct sockaddr*)&server_addr, sizeof(server_addr));
    send_packed_data(udp_fd, server_addr,
                 tv.tv_sec, tv.tv_usec,
                 loc.p_pos->x, loc.p_pos->y, loc.p_pos->z,
                 loc.p_pos->qf);
    // printf("Sent: %s", buffer);
    
    err_cnt += rv;

    return err_cnt;
    // return 0;
}

void spi_les_test(void)
{
    dwm_init();
    dwm_int_cfg_set(DWM1001_INTR_LOC_READY);
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(UDP_PORT);
    inet_pton(AF_INET, UDP_IP, &server_addr.sin_addr);

    while (keep_running) {
        struct timeval start, end;
        gettimeofday(&start, NULL);
        get_loc(&server_addr);
        gettimeofday(&end, NULL);
        long elapsed = (end.tv_sec - start.tv_sec) * 1000 +
                       (end.tv_usec - start.tv_usec) / 1000;
        long remaining = 100 - elapsed;
        if (remaining > 0)
            HAL_Delay(remaining);
    }
}

int main(int argc, char *argv[])
{
    // Set up sigaction for SIGINT
    struct sigaction sa;
    sa.sa_handler = signal_handler;
    sa.sa_flags = 0;
    sigemptyset(&sa.sa_mask);
    if (sigaction(SIGINT, &sa, NULL) == -1) {
        HAL_Log("hal: *** ERROR *** Failed to set sigaction for SIGINT\n");
        return 1;
    }

    // UDP socket setup
    udp_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_fd == -1) {
        perror("Unable to create UDP socket");
        return 1;
    }

    int k = 1;
    while (k-- > 0 && keep_running) {
        spi_les_test();
    }

    if (udp_fd != -1) {
        close(udp_fd);
    }
    return 0;
}
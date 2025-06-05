#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include <signal.h>
#include <semaphore.h>
#include <pthread.h>
#include "dwm_api.h"
#include "hal.h"
#include "hal_log.h"
#include "../test_util/test_util.h"

// #define DEBUG_PRINT
// #define DEBUG_RANDOM_LOC
#define UDP_IP "127.0.0.1" // Python script IP
#define UDP_PORT 5005      // Python script port
#define INTERVAL_MS 100    // period of location sampling 


int udp_fd = -1; // UDP socket file descriptor

void signal_handler(int sig)
{
    printf("Signal %d received, closing UDP socket\n", sig);
    if (udp_fd != -1) {
        close(udp_fd);
        udp_fd = -1;
    }
    exit(0);
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


void add_ms(struct timespec *t, int ms) {
    t->tv_nsec += (ms % 1000) * 1000000;
    t->tv_sec  += ms / 1000;

    if (t->tv_nsec >= 1000000000) {
        t->tv_nsec -= 1000000000;
        t->tv_sec += 1;
    }
}

void print_usec_now(struct timeval *tv){
    gettimeofday(tv, NULL);
    printf("Time: %06ld\n",tv->tv_usec);
}

//send real time location
void send_rtl(void)
{
    //init dwm
    dwm_init();
    dwm_int_cfg_set(DWM1001_INTR_LOC_READY);
    
    //init socket
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(UDP_PORT);
    inet_pton(AF_INET, UDP_IP, &server_addr.sin_addr);

    struct timeval tv;
    dwm_status_t status;
    dwm_loc_data_t loc;
    dwm_pos_t pos;
    loc.p_pos = &pos;
    struct timespec next;
    char loc_ready;

    //send loc data at 10Hz
    clock_gettime(CLOCK_MONOTONIC, &next);  // Get current time
    while (1) {
        #ifndef DEBUG_PRINT 
        #ifndef DEBUG_RANDOM_LOC
        while(1){
            loc_ready = dwm_status_get(&status) == RV_OK && status.loc_data;
            if(loc_ready){
                break;
            }
            //if we are so unlucky that we sample around the update time point just wait 1ms to skip it
            usleep(1000); 
            clock_gettime(CLOCK_MONOTONIC, &next);  // Set new sampling point by getting the current time
        }

        
        dwm_loc_get(&loc);
        // Test_CheckTxRx(dwm_loc_get(&loc));
        gettimeofday(&tv, NULL);
        send_packed_data(udp_fd, &server_addr,
            tv.tv_sec, tv.tv_usec,
            loc.p_pos->x, loc.p_pos->y, loc.p_pos->z,
            loc.p_pos->qf);
        #else
        gettimeofday(&tv, NULL);
        send_packed_data(udp_fd, &server_addr,
            tv.tv_sec, tv.tv_usec,
            rand()%1000, rand()%1000, rand()%1000,
            rand()%100);
        #endif
        #endif
        // get_loc(&server_addr);
  
        #ifdef DEBUG_PRINT
        print_usec_now(&tv);
        #endif

        add_ms(&next, INTERVAL_MS);
        // Sleep until the exact next time
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
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

    send_rtl();

    if (udp_fd != -1) {
        close(udp_fd);
    }
    return 0;
}
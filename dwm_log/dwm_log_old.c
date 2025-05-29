/*! ------------------------------------------------------------------------------------------------------------------
 * @file    test_les.c
 * @brief   Setup the node as a tag and print out the RTLS info.
 *          In this example, the steps to configure the Tag include:
 *          Tag mode, PANID, encryption key, update rate.
 *          According to the network settings, some of these settings
 *          are not necessary.
 *
 * @attention
 *
 * Copyright 2017 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */

#include <string.h>
#include "dwm_api.h"
#include "hal.h"
#include "hal_log.h"
// #include "hal_gpio.h"
#include "../test_util/test_util.h"
#include <sys/time.h>
#include <math.h>
#include <dirent.h>
#include <stdlib.h>
#include <signal.h>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

typedef enum
{
   MAVLINK_CONN_NONE,
   MAVLINK_CONN_SERIAL,
   MAVLINK_CONN_UDP
} mavlink_conn_type_t;

typedef struct
{
   mavlink_conn_type_t type;
   int fd;                     // for serial or UDP socket
   struct sockaddr_in udp_src; // for UDP source filtering (optional)
} mavlink_conn_t;

struct timeval tv;
// struct timeval last_tv;
uint64_t ts_curr = 0;
uint64_t ts_last = 0;
volatile uint8_t data_ready;
FILE *fp = NULL;
static volatile sig_atomic_t keep_running = 1;

void signal_handler(int sig)
{
   printf("Signal %d received, closing file\n", sig);
   if (fp != NULL)
   {
      fclose(fp);
      fp = NULL;
   }
   keep_running = 0;
   // Clean up pigpio resources
   //  gpioTerminate();
}

mavlink_conn_t open_mavlink(const char *conn_string)
{
   mavlink_conn_t conn = {0};

   if (strncmp(conn_string, "udp://", 6) == 0)
   {
      // Parse "udp://IP:PORT"
      const char *ip_port = conn_string + 6;
      char ip[64];
      int port;
      sscanf(ip_port, "%63[^:]:%d", ip, &port);

      int sock = socket(AF_INET, SOCK_DGRAM, 0);
      if (sock < 0)
      {
         perror("socket");
         return conn;
      }

      struct sockaddr_in addr = {0};
      addr.sin_family = AF_INET;
      addr.sin_addr.s_addr = INADDR_ANY;
      addr.sin_port = htons(port);

      if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0)
      {
         perror("bind");
         close(sock);
         return conn;
      }

      conn.type = MAVLINK_CONN_UDP;
      conn.fd = sock;
   }
   else
   {
      int fd = open(conn_string, O_RDWR | O_NOCTTY | O_NDELAY);
      if (fd == -1)
      {
         perror("open serial");
         return conn;
      }

      struct termios options;
      tcgetattr(fd, &options);
      cfsetispeed(&options, B57600);
      cfsetospeed(&options, B57600);
      options.c_cflag |= (CLOCAL | CREAD);
      options.c_cflag &= ~CSIZE;
      options.c_cflag |= CS8;
      options.c_cflag &= ~PARENB;
      options.c_cflag &= ~CSTOPB;
      options.c_cflag &= ~CRTSCTS;
      tcsetattr(fd, TCSANOW, &options);

      conn.type = MAVLINK_CONN_SERIAL;
      conn.fd = fd;
   }

   return conn;
}

int read_mavlink_altitude(mavlink_conn_t *conn, int32_t *altitude_mm)
{
   uint8_t c;
   mavlink_message_t msg;
   mavlink_status_t status;
   ssize_t len;

   while (1)
   {
      if (conn->type == MAVLINK_CONN_SERIAL)
      {
         len = read(conn->fd, &c, 1);
         if (len <= 0)
            break;
      }
      else if (conn->type == MAVLINK_CONN_UDP)
      {
         uint8_t buf[512];
         struct sockaddr_in src;
         socklen_t src_len = sizeof(src);
         len = recvfrom(conn->fd, buf, sizeof(buf), MSG_DONTWAIT, (struct sockaddr *)&src, &src_len);
         if (len <= 0)
            break;

         for (ssize_t i = 0; i < len; ++i)
         {
            c = buf[i];
            if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
            {
               if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT)
               {
                  mavlink_global_position_int_t pos;
                  mavlink_msg_global_position_int_decode(&msg, &pos);
                  *altitude_mm = pos.alt;
                  return 1;
               }
            }
         }
         continue;
      }
   }

   return 0;
}

// int frst(void)
// {
//    int rv;
//    int err_cnt = 0;
//    int delay_ms = 2000;

//    HAL_Log("dwm_factory_reset().\n");
//    rv = Test_CheckTxRx(dwm_factory_reset());
//    HAL_Log("Wait %d ms for node to reset.\n", delay_ms);
//    HAL_Delay(delay_ms);
//    err_cnt += rv;

//    if (rv == RV_OK)
//    {
//       dwm_deinit();
//       dwm_init();
//    }

//    printf("%s %s: err_cnt = %d\n", (err_cnt >0)? "ERR" : "   ", __FUNCTION__, err_cnt);
//    return err_cnt;
// }

int get_loc(void)
{
   // ========== dwm_loc_get ==========
   int rv, err_cnt = 0;
   // int i;

   dwm_loc_data_t loc;
   dwm_pos_t pos;
   loc.p_pos = &pos;
   //    HAL_Log("dwm_loc_get(&loc)\n");
   rv = Test_CheckTxRx(dwm_loc_get(&loc));

   gettimeofday(&tv, NULL);

   int32_t altitude = -1;
    read_mavlink_altitude(mav_fd, &altitude);
   // long ms_diff = (tv.tv_sec - last_tv.tv_sec) * 1000 +
   //                     (tv.tv_usec - last_tv.tv_usec) / 1000;

   if (rv == RV_OK /*&& ms_diff >= 100*/)
   {
      //   HAL_Log("ts:%ld.%06ld [%d,%d,%d,%u]\n", tv.tv_sec, tv.tv_usec, loc.p_pos->x, loc.p_pos->y, loc.p_pos->z, loc.p_pos->qf);
      //   printf("ts:%ld.%06ld [%d,%d,%d,%u]", tv.tv_sec, tv.tv_usec, loc.p_pos->x, loc.p_pos->y, loc.p_pos->z, loc.p_pos->qf);
      // fprintf(fp, "ts:%ld.%06ld [%d,%d,%d,%u]\n", tv.tv_sec, tv.tv_usec, loc.p_pos->x, loc.p_pos->y, loc.p_pos->z, loc.p_pos->qf);
      fprintf(fp, "%ld.%06ld,%d,%d,%d,%u,%d,\n", tv.tv_sec, tv.tv_usec, loc.p_pos->x, loc.p_pos->y, loc.p_pos->z, loc.p_pos->qf,altitude);

      // last_tv = tv;
      //   for (i = 0; i < loc.anchors.dist.cnt; ++i)
      //   {
      //      HAL_Log("#%u)", i);
      //      printf("#%u)", i);
      //      HAL_Log("a:0x%08x", loc.anchors.dist.addr[i]);
      //      printf("a:0x%08x", loc.anchors.dist.addr[i]);
      //      if (i < loc.anchors.an_pos.cnt)
      //      {
      //         HAL_Log("[%d,%d,%d,%u]", loc.anchors.an_pos.pos[i].x,
      //               loc.anchors.an_pos.pos[i].y,
      //               loc.anchors.an_pos.pos[i].z,
      //               loc.anchors.an_pos.pos[i].qf);
      //         printf("[%d,%d,%d,%u]", loc.anchors.an_pos.pos[i].x,
      //               loc.anchors.an_pos.pos[i].y,
      //               loc.anchors.an_pos.pos[i].z,
      //               loc.anchors.an_pos.pos[i].qf);
      //      }
      //      HAL_Log("d=%u,qf=%u\n", loc.anchors.dist.dist[i], loc.anchors.dist.qf[i]);
      //      printf("d=%u,qf=%u", loc.anchors.dist.dist[i], loc.anchors.dist.qf[i]);
      //   }
      //      HAL_Log("\n");
      //      printf("\n");
   }
   err_cnt += rv;

   return err_cnt;
}

// configure the tag and restart the node
// int setup_tag()
// {
//    int rv, err_cnt = 0;
//    int delay_ms = 1500;

//    dwm_cfg_tag_t cfg_tag;
//    dwm_cfg_t cfg;
//    cfg_tag.stnry_en = 1;
//    cfg_tag.low_power_en = 0;
//    cfg_tag.meas_mode = DWM_MEAS_MODE_TWR;
//    cfg_tag.loc_engine_en = 1;
//    cfg_tag.common.enc_en = 0;
//    cfg_tag.common.led_en = 1;
//    cfg_tag.common.ble_en = 0;
//    cfg_tag.common.uwb_mode = DWM_UWB_MODE_ACTIVE;
//    cfg_tag.common.fw_update_en = 0;

//    HAL_Log("dwm_cfg_get(&cfg)\n");
//    err_cnt += Test_CheckTxRx(dwm_cfg_get(&cfg));

//    while((cfg_tag.low_power_en!= cfg.low_power_en)
//    || (cfg_tag.meas_mode      != cfg.meas_mode)
//    || (cfg_tag.loc_engine_en  != cfg.loc_engine_en)
//    || (cfg_tag.stnry_en       != cfg.stnry_en)
//    || (cfg_tag.common.enc_en  != cfg.common.enc_en)
//    || (cfg_tag.common.led_en  != cfg.common.led_en)
//    || (cfg_tag.common.ble_en  != cfg.common.ble_en)
//    || (cfg_tag.common.uwb_mode != cfg.common.uwb_mode)
//    || (cfg_tag.common.fw_update_en != cfg.common.fw_update_en))
//    {
//       printf("Comparing set vs. get.\n");
// 		if(cfg.mode 			!= DWM_MODE_TAG) 		      printf("mode: get = %d, set = %d\n", cfg.mode, 		DWM_MODE_TAG);
// 		if(cfg.stnry_en     	!= cfg_tag.stnry_en)  		printf("acce: get = %d, set = %d\n", cfg.stnry_en, 	cfg_tag.stnry_en);
// 		if(cfg.loc_engine_en != cfg_tag.loc_engine_en) 	printf("le  : get = %d, set = %d\n", cfg.loc_engine_en, cfg_tag.loc_engine_en);
// 		if(cfg.low_power_en	!= cfg_tag.low_power_en)	printf("lp  : get = %d, set = %d\n", cfg.low_power_en, 	cfg_tag.low_power_en);
// 		if(cfg.meas_mode 		!= cfg_tag.meas_mode) 		printf("meas: get = %d, set = %d\n", cfg.meas_mode, 	cfg_tag.meas_mode);
// 		if(cfg.common.fw_update_en != cfg_tag.common.fw_update_en)
//                                                       printf("fwup: get = %d, set = %d\n", cfg.common.fw_update_en, cfg_tag.common.fw_update_en);
// 		if(cfg.common.uwb_mode!=cfg_tag.common.uwb_mode)printf("uwb : get = %d, set = %d\n", cfg.common.uwb_mode, cfg_tag.common.uwb_mode);
// 		if(cfg.common.ble_en != cfg_tag.common.ble_en)	printf("ble : get = %d, set = %d\n", cfg.common.ble_en, cfg_tag.common.ble_en);
// 		if(cfg.common.led_en != cfg_tag.common.led_en)	printf("led : get = %d, set = %d\n", cfg.common.led_en, cfg_tag.common.led_en);

//       HAL_Log("dwm_cfg_tag_set(&cfg_tag)\n");
//       rv = Test_CheckTxRx(dwm_cfg_tag_set(&cfg_tag));
//       err_cnt += rv;

//       HAL_Log("dwm_reset()\n");
//       rv = Test_CheckTxRx(dwm_reset());
//       HAL_Log("Wait %d ms for node to reset.\n", delay_ms);
//       err_cnt += rv;
//       HAL_Delay(delay_ms);

//       HAL_Log("dwm_cfg_get(&cfg)\n");
//       rv = Test_CheckTxRx(dwm_cfg_get(&cfg));
//       err_cnt += rv;
//    }
//    HAL_Log("Done.\n");

//    printf("%s %s: err_cnt = %d\n", (err_cnt >0)? "ERR" : "   ", __FUNCTION__, err_cnt);
//    return err_cnt;
// }

// int setup_enc()
// {
//    int rv;
//    int err_cnt = 0;
//    int i = 0;

//    // clear the previously used key
//    HAL_Log("dwm_enc_key_clear(void)\n");
//    rv = Test_CheckTxRx(dwm_enc_key_clear());
//    Test_Report("dwm_enc_key_clear(void):\t\t\t%s\n", rv==0 ? "pass":"fail");
//    err_cnt += rv;

//    dwm_enc_key_t key;

//    // the key used in the example network is:
//    // 0x 12 34 12 34 12 34 12 34 12 34 12 34 12 34 12 34
//    while (i < DWM_ENC_KEY_LEN)
//    {
//       key.byte[i++] = 0x12;
//       key.byte[i++] = 0x34;
//    }

//    HAL_Log("dwm_enc_key_set(&key)\n");
//    rv = Test_CheckTxRx(dwm_enc_key_set(&key));
//    Test_Report("dwm_enc_key_set(&key):\t\t\t%s\n", rv==0 ? "pass":"fail");
//    err_cnt += rv;

//    HAL_Delay(3000);

//    printf("%s %s: err_cnt = %d\n", (err_cnt >0)? "ERR" : "   ", __FUNCTION__, err_cnt);
//    return err_cnt;
// }
// void gpio_cb(int gpio, int level, uint32_t tick)
// {
//     if (level == 1) // Trigger on rising edge
//     {
//         data_ready = 1;
//         HAL_Log("hal: GPIO %d interrupt, level=%d, tick=%u\n", gpio, level, tick);
//     }
// }

void spi_les_test(void)
{
   // int err_cnt = 0;
   // uint16_t panid;
   // uint16_t ur_set;
   // uint16_t ur_s_set;

   { // init
     //    printf("Initializing...\n");
     //    HAL_Log("Initializing...\n");
      dwm_init();
      //   err_cnt += frst();
      //   HAL_Log("Done\n");
   }

   /* ========= published APIs =========*/

   // setup tag mode configurations
   //    setup_tag();

   // setup tag update rate
   //    ur_s_set = ur_set = 1;
   //    HAL_Log("dwm_upd_rate_set(ur_set, ur_s_set);\n");
   //    dwm_upd_rate_set(ur_set, ur_s_set);

   // setup PANID
   //    panid = 1;
   //    HAL_Log("dwm_panid_set(panid);\n");
   //    dwm_panid_set(panid);

   // setup GPIO interrupt from "LOC_READY" event
   //    HAL_Log("dwm_int_cfg_set(DWM1001_INTR_LOC_READY);\n");
   dwm_int_cfg_set(DWM1001_INTR_LOC_READY);
   // HAL_GPIO_SetupCb(HAL_GPIO_DRDY, HAL_GPIO_INT_EDGE_RISING, &gpio_cb);
   // HAL_UART_SetRxCb(uart_rx_callback); // Register UART callback

   // setup encryption key for network
   //    setup_enc();

   while (keep_running)
   {
      struct timeval start, end;
      gettimeofday(&start, NULL);
      // if(data_ready == 1)
      // {
      // data_ready = 0;
      get_loc();
      // }
      gettimeofday(&end, NULL);
      long elapsed = (end.tv_sec - start.tv_sec) * 1000 +
                     (end.tv_usec - start.tv_usec) / 1000;

      long remaining = 100 - elapsed;

      if (remaining > 0)
         HAL_Delay(remaining);
   }

   //    HAL_Log("err_cnt = %d \n", err_cnt);

   //    Test_End();
}

int main(int argc, char *argv[])
{

   char *mavlink_target = NULL;
   for (int i = 1; i < argc; ++i)
   {
      if (strcmp(argv[i], "--mavlink") == 0 && i + 1 < argc)
      {
         mavlink_target = argv[++i];
      }
   }

   if (!mavlink_target)
   {
      printf("Usage: %s --mavlink <udp://IP:port or /dev/ttyUSBX>\n", argv[0]);
      return 1;
   }

   mavlink_conn_t mav_conn = open_mavlink(mavlink_target);
   if (mav_conn.type == MAVLINK_CONN_NONE)
   {
      fprintf(stderr, "Failed to connect to MAVLink source\n");
      return 1;
   }

   // open log file
   //  struct timeval tv;
   //  gettimeofday(&tv, NULL);
   //  long now = tv.tv_sec;
   //  unsigned digits = floor(log10(now < 0 ? -now : now)) + 1;
   //  char filename[digits + 9];
   //  snprintf(filename, sizeof(filename), "log_%ld.log", now);
   //  fp = fopen(filename,"w");
   //  signal(SIGINT, signal_handler);
   //  gettimeofday(&last_tv,NULL);
   //  Set up sigaction for SIGINT
   struct sigaction sa;
   sa.sa_handler = signal_handler;
   sa.sa_flags = 0;          // No special flags
   sigemptyset(&sa.sa_mask); // No signals blocked during handler
   if (sigaction(SIGINT, &sa, NULL) == -1)
   {
      HAL_Log("hal: *** ERROR *** Failed to set sigaction for SIGINT\n");
      return 1;
   }

   DIR *dir;
   struct dirent *entry;
   int max_number = -1; // Initialize to -1 to handle no matches
   char new_filename[12];

   // Open current directory
   dir = opendir(".");
   if (dir == NULL)
   {
      perror("Error opening directory");
      return 1;
   }

   // Read directory contents
   while ((entry = readdir(dir)) != NULL)
   {
      int number;
      // Check if file matches pattern "log_%ld.log"
      if (sscanf(entry->d_name, "log_%03d.log", &number) == 1)
      {
         if (number > max_number)
         {
            max_number = number;
         }
      }
   }

   closedir(dir);

   // Check if any matching files were found
   if (max_number == -1)
   {
      printf("No files matching 'log_%%ld.log' found. Creating log_0.log\n");
      // max_number = -1; // Start with log_0.log
   }

   // Create new filename with incremented number
   snprintf(new_filename, sizeof(new_filename), "log_%03d.log", max_number + 1);

   // Create new file
   fp = fopen(new_filename, "w");
   if (fp == NULL)
   {
      perror("Error creating new file");
      return 1;
   }
   printf("Created file: %s\n", new_filename);

   int k = 1;
   while (k-- > 0 && keep_running)
   {
      spi_les_test();
   }
   return 0;
}

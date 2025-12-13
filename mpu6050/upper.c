/*
 * upper.c
 * 简单的上位机 C 版本，功能：
 * - 通过串口向下位机发送速度命令（功能码 0x10，payload: 2 floats 小端）
 * - 接收并解析下位机回传帧（帧头 0xAA 0x55），支持 IMU(0x10, 3 floats) 与 MOTOR(0x20, 2 floats)
 * - 非阻塞键盘控制：w/s 增减线速度，a/d 改变角速度，空格停止，q 退出
 *
 * 编译: gcc -o upper upper.c -lpthread
 * 运行: sudo ./upper /dev/ttyACM0 115200
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <pthread.h>
#include <sys/select.h>
#include <sys/time.h>

// 默认串口与波特率
static const char *DEFAULT_PORT = "/dev/ttyACM0";
static int DEFAULT_BAUD = 115200;

// 协议定义
#define HDR1 0xAA
#define HDR2 0x55
#define CMD_SPEED 0x10 // 发送给下位机的设置速度命令

// 接收功能码
#define FUNC_IMU   0x10
#define FUNC_MOTOR 0x20

static int serial_fd = -1;
static volatile int running = 1;

// 目标速度（由键盘控制）
static float target_vx = 0.0f; // 线速度
static float target_vw = 0.0f; // 角速度

// 简单的 checksum（累加后取 8 bit）
static uint8_t calc_checksum(const uint8_t *data, size_t len) {
    uint32_t s = 0;
    for (size_t i = 0; i < len; ++i) s += data[i];
    return (uint8_t)(s & 0xFF);
}

// 设置串口参数（8N1，原始模式）
static int open_serial(const char *path, int baud) {
    struct termios tty;
    int fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        perror("open serial");
        return -1;
    }

    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr"); close(fd); return -1;
    }

    cfmakeraw(&tty);

    // 设置波特率
    speed_t speed;
    switch (baud) {
        case 115200: speed = B115200; break;
        case 57600: speed = B57600; break;
        case 38400: speed = B38400; break;
        case 19200: speed = B19200; break;
        case 9600:  speed = B9600;  break;
        default: speed = B115200; break;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_cflag |= CLOCAL | CREAD;    // ignore modem controls
    tty.c_cflag &= ~(PARENB | PARODD); // no parity
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag &= ~CRTSCTS; // no flow control

    // Non canonical, no echo
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_iflag = 0;

    // read returns as soon as at least 1 byte is available
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1; // 0.1s

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr"); close(fd); return -1;
    }

    // set blocking mode off (we use select)
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) & ~O_NONBLOCK);

    return fd;
}

// 发送速度帧 (func + len + payload + crc)
static int send_speed_frame(int fd, float vx, float vw) {
    uint8_t payload[8];
    memcpy(&payload[0], &vx, 4);
    memcpy(&payload[4], &vw, 4);

    uint8_t func = CMD_SPEED;
    uint8_t len = sizeof(payload);

    uint8_t header[2] = {HDR1, HDR2};
    uint8_t check_src[1 + 1 + sizeof(payload)];
    check_src[0] = func;
    check_src[1] = len;
    memcpy(&check_src[2], payload, sizeof(payload));
    uint8_t crc = calc_checksum(check_src, sizeof(check_src));

    // 组包并写
    uint8_t pkt[2 + sizeof(check_src) + 1];
    memcpy(pkt, header, 2);
    memcpy(pkt + 2, check_src, sizeof(check_src));
    pkt[2 + sizeof(check_src)] = crc;

    ssize_t w = write(fd, pkt, sizeof(pkt));
    if (w != (ssize_t)sizeof(pkt)) {
        // 非致命错误，可能短写
        return -1;
    }
    return 0;
}

// 接收线程：解析下位机发送的帧并打印
static void *recv_thread(void *arg) {
    (void)arg;
    enum {S_HDR1, S_HDR2, S_FUNC, S_LEN, S_PAYLOAD, S_CRC} state = S_HDR1;
    uint8_t func = 0, len = 0; size_t pcount = 0;
    uint8_t payload[256];
    uint8_t checksum_acc = 0;

    while (running) {
        fd_set rfds; FD_ZERO(&rfds); FD_SET(serial_fd, &rfds);
        struct timeval tv = {0, 200000}; // 200ms
        int ret = select(serial_fd + 1, &rfds, NULL, NULL, &tv);
        if (ret <= 0) continue;
        uint8_t b;
        ssize_t r = read(serial_fd, &b, 1);
        if (r <= 0) continue;

        switch (state) {
            case S_HDR1:
                if (b == HDR1) state = S_HDR2;
                break;
            case S_HDR2:
                if (b == HDR2) { state = S_FUNC; checksum_acc = 0; }
                else if (b == HDR1) { state = S_HDR2; }
                else state = S_HDR1;
                break;
            case S_FUNC:
                func = b; checksum_acc = b; state = S_LEN; break;
            case S_LEN:
                len = b; checksum_acc += b; pcount = 0;
                if (len == 0) state = S_CRC; else state = S_PAYLOAD;
                break;
            case S_PAYLOAD:
                payload[pcount++] = b; checksum_acc += b;
                if (pcount >= len) state = S_CRC;
                break;
            case S_CRC: {
                uint8_t recv_crc = b;
                if ((checksum_acc & 0xFF) == recv_crc) {
                    // 解析
                    if (func == FUNC_IMU && len == 12) {
                        float ax, ay, az;
                        memcpy(&ax, &payload[0], 4);
                        memcpy(&ay, &payload[4], 4);
                        memcpy(&az, &payload[8], 4);
                        printf("\r[IMU] Accel: X=%6.2f Y=%6.2f Z=%6.2f", ax, ay, az);
                        fflush(stdout);
                    } else if (func == FUNC_MOTOR && len == 8) {
                        float l, r;
                        memcpy(&l, &payload[0], 4);
                        memcpy(&r, &payload[4], 4);
                        printf(" | [Motor] L=%6.2f R=%6.2f", l, r);
                        fflush(stdout);
                    } else {
                        // 未知帧，忽略
                    }
                } else {
                    fprintf(stderr, "\nCRC Error! Calc:%02X Recv:%02X\n", checksum_acc & 0xFF, recv_crc);
                }
                state = S_HDR1;
                break; }
        }
    }
    return NULL;
}

// 键盘控制线程：在终端置为 raw 模式后读取按键
static struct termios orig_term;
static void enable_raw_mode() {
    struct termios t;
    tcgetattr(STDIN_FILENO, &orig_term);
    t = orig_term;
    cfmakeraw(&t);
    tcsetattr(STDIN_FILENO, TCSANOW, &t);
}
static void disable_raw_mode() {
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_term);
}

static void *kbd_thread(void *arg) {
    (void)arg;
    enable_raw_mode();
    printf("\n=== Robot Control Panel (C) ===\n W/S: +/- Vx  A/D: +/- Vw  Space: STOP  Q: Quit\n===============================\n");
    while (running) {
        fd_set rfds; FD_ZERO(&rfds); FD_SET(STDIN_FILENO, &rfds);
        struct timeval tv = {0, 100000};
        int ret = select(STDIN_FILENO+1, &rfds, NULL, NULL, &tv);
        if (ret > 0 && FD_ISSET(STDIN_FILENO, &rfds)) {
            char c; if (read(STDIN_FILENO, &c, 1) <= 0) continue;
            if (c == 'w' || c == 'W') target_vx += 0.1f;
            else if (c == 's' || c == 'S') target_vx -= 0.1f;
            else if (c == 'a' || c == 'A') target_vw += 0.5f;
            else if (c == 'd' || c == 'D') target_vw -= 0.5f;
            else if (c == ' ') { target_vx = 0.0f; target_vw = 0.0f; }
            else if (c == 'q' || c == 'Q') { running = 0; break; }

            if (target_vx > 2.0f) target_vx = 2.0f;
            if (target_vx < -2.0f) target_vx = -2.0f;

            printf("\n>> Cmd: Vx=%.2f Vw=%.2f\n", target_vx, target_vw);
            fflush(stdout);
        }
        usleep(50000);
    }
    disable_raw_mode();
    return NULL;
}

// 发送线程：周期性发送速度帧
static void *send_thread(void *arg) {
    (void)arg;
    while (running) {
        // 每 100ms 发送一次
        send_speed_frame(serial_fd, target_vx, target_vw);
        usleep(100000);
    }
    return NULL;
}

int main(int argc, char **argv) {
    const char *port = DEFAULT_PORT;
    int baud = DEFAULT_BAUD;
    if (argc >= 2) port = argv[1];
    if (argc >= 3) baud = atoi(argv[2]);

    serial_fd = open_serial(port, baud);
    if (serial_fd < 0) {
        fprintf(stderr, "Failed to open serial %s\n", port);
        return 1;
    }
    printf("Connected to %s at %d\n", port, baud);

    pthread_t th_recv, th_send, th_kbd;
    pthread_create(&th_recv, NULL, recv_thread, NULL);
    pthread_create(&th_send, NULL, send_thread, NULL);
    pthread_create(&th_kbd,  NULL, kbd_thread,  NULL);

    // 等待键盘线程退出（按 q）
    pthread_join(th_kbd, NULL);

    // 结束其他线程
    running = 0;
    pthread_join(th_send, NULL);
    pthread_join(th_recv, NULL);

    if (serial_fd >= 0) close(serial_fd);
    printf("\nBye!\n");
    return 0;
}

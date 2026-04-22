#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "lwip/ip4_addr.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

// WiFi tenging
#define WIFI_SSID "oli-iphone"
#define WIFI_PASSWORD "Manchester"
#define SERVER_IP "172.20.10.5"
#define SERVER_PORT 5005
#define WIFI_RETRY_INTERVAL_MS 30000

// I2C tenging og pinnar
#define I2C_PORT i2c0
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
#define BNO_INT_PIN 6
#define BNO_ADDR 0x4B
#define I2C_FREQ_HZ 100000

// BNO055 tenging
#define SHTP_HEADER_LEN 4
#define CHANNEL_CONTROL 2
#define CHANNEL_REPORTS 3

#define REPORT_SET_FEATURE_COMMAND 0xFD
#define REPORTID_ROTATION_VECTOR 0x05

//Staðlaðar breytur
static uint8_t seq_num[6] = {0};
static absolute_time_t last_wifi_retry_at;
static struct udp_pcb *udp_client = NULL;
static ip_addr_t server_ip_addr;

static bool time_is_set(absolute_time_t t) {
    return to_us_since_boot(t) != 0;
}
// Hjálparföll
static int16_t i16_le(uint8_t b0, uint8_t b1) {
    int32_t v = b0 | (b1 << 8);
    if (v & 0x8000) {
        v -= 65536;
    }
    return (int16_t)v;
}

static float q14_to_float(int16_t v) {
    return (float)v / 16384.0f;
}

static void init_i2c_bus(void) {
    sleep_ms(2000);
    i2c_init(I2C_PORT, I2C_FREQ_HZ);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
}
// BNO055 tenging og gagnavinnsla
static bool packet_available(void) {
    return gpio_get(BNO_INT_PIN) == 0;
}
// SHTP pakkar hafa 4 bita header og allt að 60 bita payload
static bool send_packet(uint8_t channel, const uint8_t *payload, size_t payload_len) {
    uint8_t packet[64];
    size_t packet_len = payload_len + SHTP_HEADER_LEN;

    if (packet_len > sizeof(packet)) {
        return false;
    }
// SHTP header: [0-1] = packet length, [2] = channel, [3] = sequence number
    packet[0] = (uint8_t)(packet_len & 0xFF);
    packet[1] = (uint8_t)((packet_len >> 8) & 0xFF);
    packet[2] = channel;
    packet[3] = seq_num[channel];
    memcpy(packet + SHTP_HEADER_LEN, payload, payload_len);

//  senda pakkann í allt að 3 tilraunum með smá töf á milli
    for (int attempt = 0; attempt < 3; ++attempt) {
        int written = i2c_write_blocking(I2C_PORT, BNO_ADDR, packet, (int)packet_len, false);
        if (written == (int)packet_len) {
            seq_num[channel] = (uint8_t)((seq_num[channel] + 1) & 0xFF);
            return true;
        }
        sleep_ms(20);
    }

    return false;
}
// Lesa SHTP
static bool read_packet(uint8_t *channel, uint8_t *body, size_t *body_len) {
    uint8_t buf[64];

    for (int attempt = 0; attempt < 3; ++attempt) {
        int read = i2c_read_blocking(I2C_PORT, BNO_ADDR, buf, sizeof(buf), false);
        if (read == (int)sizeof(buf)) {
            uint16_t packet_len = (uint16_t)((buf[0] | (buf[1] << 8)) & 0x7FFF);
            *channel = buf[2];

            if (packet_len < 4) {
                *body_len = 0;
                return true;
            }

            if (packet_len > sizeof(buf)) {
                packet_len = sizeof(buf);
            }

            *body_len = packet_len - 4;
            memcpy(body, buf + 4, *body_len);
            return true;
        }
        sleep_ms(20);
    }

    return false;
}

static bool wait_for_packet(uint32_t timeout_ms) {
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    while (absolute_time_diff_us(get_absolute_time(), deadline) > 0) {
        if (packet_available()) {
            return true;
        }
        sleep_ms(1);
    }
    return false;
}
// Senda SHTP "set feature" skipun til að biðja um reglulegan aflestur
static bool set_feature(uint8_t report_id, uint32_t interval_ms) {
    uint32_t interval_us = interval_ms * 1000u;
    uint8_t payload[17] = {
        REPORT_SET_FEATURE_COMMAND,
        report_id,
        0x00,
        0x00, 0x00,
        (uint8_t)(interval_us & 0xFF),
        (uint8_t)((interval_us >> 8) & 0xFF),
        (uint8_t)((interval_us >> 16) & 0xFF),
        (uint8_t)((interval_us >> 24) & 0xFF),
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00
    };

    return send_packet(CHANNEL_CONTROL, payload, sizeof(payload));
}
// Fá fram Eulerhorn 
static void quat_to_euler_deg(float x, float y, float z, float w, float *roll, float *pitch, float *yaw) {
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    *roll = atan2f(sinr_cosp, cosr_cosp) * 57.2957795f;

    float sinp = 2.0f * (w * y - z * x);
    if (sinp > 1.0f) {
        sinp = 1.0f;
    } else if (sinp < -1.0f) {
        sinp = -1.0f;
    }
    *pitch = asinf(sinp) * 57.2957795f;

    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    *yaw = atan2f(siny_cosp, cosy_cosp) * 57.2957795f;
}
// Fá fram roll, pitch og yaw 
static bool extract_rotation_vector(const uint8_t *body, size_t body_len, float *roll, float *pitch, float *yaw) {
    size_t start = SIZE_MAX;

    if (body_len == 0) {
        return false;
    }

    if (body[0] == 0xFB) {
        if (body_len >= 19 && body[5] == REPORTID_ROTATION_VECTOR) {
            start = 5;
        }
    } else if (body[0] == REPORTID_ROTATION_VECTOR) {
        if (body_len >= 14) {
            start = 0;
        }
    }

    if (start == SIZE_MAX || body_len < start + 14) {
        return false;
    }

    const uint8_t *rpt = body + start;
    float qi = q14_to_float(i16_le(rpt[4], rpt[5]));
    float qj = q14_to_float(i16_le(rpt[6], rpt[7]));
    float qk = q14_to_float(i16_le(rpt[8], rpt[9]));
    float qr = q14_to_float(i16_le(rpt[10], rpt[11]));
    quat_to_euler_deg(qi, qj, qk, qr, roll, pitch, yaw);
    return true;
}
// Wi-Fi tenging og UDP sending
static bool connect_wifi(uint32_t timeout_ms) {
    for (int attempt = 1; attempt <= 2; ++attempt) {
        cyw43_arch_enable_sta_mode();
        printf("Connecting to Wi-Fi: %s (attempt %d)\n", WIFI_SSID, attempt);

        int err = cyw43_arch_wifi_connect_timeout_ms(
            WIFI_SSID,
            WIFI_PASSWORD,
            CYW43_AUTH_WPA2_AES_PSK,
            timeout_ms
        );

        if (err == 0) {
            printf("Wi-Fi connected\n");
            return true;
        }

        printf("Wi-Fi connect timeout/error: %d\n", err);
        sleep_ms(1200);
    }

    return false;
}
// Tryggja að WiFi sé tengt áður en reynt er að senda UDP pakkann
static bool ensure_wireless_ready(void) {
    absolute_time_t now = get_absolute_time();
    int link_status = cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA);

    if (link_status != CYW43_LINK_UP) {
        if (!time_is_set(last_wifi_retry_at) ||
            absolute_time_diff_us(last_wifi_retry_at, now) >= (int64_t)WIFI_RETRY_INTERVAL_MS * 1000) {
            last_wifi_retry_at = now;
            if (connect_wifi(10000)) {
                printf("UDP sender ready -> %s:%d\n", SERVER_IP, SERVER_PORT);
            } else {
                printf("Wi-Fi retry failed; next retry in %d s\n", WIFI_RETRY_INTERVAL_MS / 1000);
            }
        }
        return false;
    }

    return true;
}
// Senda eina línu sem UDP 
static void send_udp_line(const char *line) {
    if (!ensure_wireless_ready()) {
        return;
    }

    cyw43_arch_lwip_begin();

    if (!udp_client) {
        udp_client = udp_new_ip_type(IPADDR_TYPE_V4);
        if (!udp_client) {
            cyw43_arch_lwip_end();
            return;
        }
        ip4addr_aton(SERVER_IP, ip_2_ip4(&server_ip_addr));
    }

    size_t line_len = strlen(line);
    struct pbuf *buf = pbuf_alloc(PBUF_TRANSPORT, (u16_t)(line_len + 1), PBUF_RAM);
    if (!buf) {
        cyw43_arch_lwip_end();
        return;
    }

    memcpy(buf->payload, line, line_len);
    ((char *)buf->payload)[line_len] = '\n';
    udp_sendto(udp_client, buf, &server_ip_addr, SERVER_PORT);
    pbuf_free(buf);

    cyw43_arch_lwip_end();
}
// Tryggja að WiFi sé tengt áður en reynt er að senda UDP pakkann
static void send_wireless(const char *line) {
    if (ensure_wireless_ready()) {
        send_udp_line(line);
    }
}
// Aðal forritið!!!!
int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    printf("STARTING COCKPIT.C\n");

    init_i2c_bus();
    gpio_init(BNO_INT_PIN);
    gpio_set_dir(BNO_INT_PIN, GPIO_IN);
    gpio_pull_up(BNO_INT_PIN);

    last_wifi_retry_at = (absolute_time_t){0};

    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return -1;
    }

    sleep_ms(3000);
    if (connect_wifi(15000)) {
        printf("UDP sender ready -> %s:%d\n", SERVER_IP, SERVER_PORT);
    }

    printf("Biða eftir startup packet...\n");
    bool startup_ok = false;
    uint8_t channel = 0;
    uint8_t body[64];
    size_t body_len = 0;

    for (int attempt = 1; attempt <= 3; ++attempt) {
        if (wait_for_packet(3000) && read_packet(&channel, body, &body_len)) {
            printf("Startup packet móttekið á channel %u\n", channel);
            startup_ok = true;
            break;
        }
        printf("Enginn startup packet (tilraun %d)\n", attempt);
        init_i2c_bus();
        sleep_ms(200);
    }

    printf("Draining startup packets...\n");
    if (startup_ok) {
        for (int i = 0; i < 20; ++i) {
            if (wait_for_packet(100) && read_packet(&channel, body, &body_len)) {
                printf("BOOT CH: %u LEN: %u\n", channel, (unsigned)body_len);
            } else {
                break;
            }
        }
    }

    printf("Enabling rotation vector...\n");
    for (int attempt = 1; attempt <= 3; ++attempt) {
        if (set_feature(REPORTID_ROTATION_VECTOR, 25)) {
            break;
        }
        printf("set_feature error (attempt %d)\n", attempt);
        init_i2c_bus();
        sleep_ms(200);
        if (attempt == 3) {
            printf("Failed to enable rotation vector\n");
        }
    }
// Aðal lykkja: lesa pakka og senda út sem UDP
    while (true) {
        if (wait_for_packet(1000)) {
            if (read_packet(&channel, body, &body_len) && channel == CHANNEL_REPORTS) {
                float roll, pitch, yaw;
                if (extract_rotation_vector(body, body_len, &roll, &pitch, &yaw)) {
                    char line[96];
                    snprintf(line, sizeof(line), "ROLL:%.2f,PITCH:%.2f,YAW:%.2f", roll, pitch, yaw);
                    printf("%s\n", line);
                    send_wireless(line);
                }
            }
        } else {
            printf("No packet, INT = %d\n", gpio_get(BNO_INT_PIN));
            init_i2c_bus();
            sleep_ms(200);
            for (int attempt = 1; attempt <= 3; ++attempt) {
                if (set_feature(REPORTID_ROTATION_VECTOR, 25)) {
                    printf("Re-enabled rotation vector\n");
                    break;
                }
                printf("re-enable error (attempt %d)\n", attempt);
                sleep_ms(200);
            }
        }

        sleep_ms(10);
    }

    cyw43_arch_deinit();
    return 0;
}

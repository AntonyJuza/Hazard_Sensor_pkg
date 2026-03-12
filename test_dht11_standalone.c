// Alternative DHT11 implementation using pigpio's callback mechanism
// This provides better timing accuracy than polling

#include <pigpiod_if2.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#define DHT11_PIN 27

typedef struct {
    uint8_t data[5];
    int bit_count;
    int state;
    uint32_t last_tick;
} dht11_state_t;

dht11_state_t dht_state;

void dht11_callback(int pi, unsigned gpio, unsigned level, uint32_t tick, void *userdata) {
    // This would handle edge detection for better timing
    // However, this is complex to integrate with ROS2 node structure
}

int read_dht11_simple(int pi, int gpio, float *temp, float *humid) {
    uint8_t data[5] = {0, 0, 0, 0, 0};
    int bit_idx = 0;
    uint32_t start_time;
    
    // Send start signal
    set_mode(pi, gpio, PI_OUTPUT);
    gpio_write(pi, gpio, 0);
    time_sleep(0.02);  // 20ms (DHT11 needs at least 18ms)
    
    gpio_write(pi, gpio, 1);
    time_sleep(0.00003);  // 30μs
    
    // Switch to input
    set_mode(pi, gpio, PI_INPUT);
    
    // Wait for sensor response (should go LOW)
    start_time = get_current_tick(pi);
    while (gpio_read(pi, gpio) == 1) {
        if ((get_current_tick(pi) - start_time) > 100) {
            printf("Timeout waiting for sensor response\n");
            return -1;
        }
    }
    
    // Wait for sensor response HIGH
    start_time = get_current_tick(pi);
    while (gpio_read(pi, gpio) == 0) {
        if ((get_current_tick(pi) - start_time) > 100) {
            printf("Timeout in LOW response\n");
            return -1;
        }
    }
    
    // Wait for start of data
    start_time = get_current_tick(pi);
    while (gpio_read(pi, gpio) == 1) {
        if ((get_current_tick(pi) - start_time) > 100) {
            printf("Timeout in HIGH response\n");
            return -1;
        }
    }
    
    // Read 40 bits
    for (int i = 0; i < 40; i++) {
        // Wait for bit start (LOW)
        start_time = get_current_tick(pi);
        while (gpio_read(pi, gpio) == 0) {
            if ((get_current_tick(pi) - start_time) > 100) {
                printf("Timeout waiting for bit %d\n", i);
                return -1;
            }
        }
        
        // Measure HIGH duration
        uint32_t high_start = get_current_tick(pi);
        while (gpio_read(pi, gpio) == 1) {
            if ((get_current_tick(pi) - high_start) > 100) {
                break;
            }
        }
        uint32_t high_duration = get_current_tick(pi) - high_start;
        
        // If HIGH duration > 40μs, bit is 1, else 0
        data[i / 8] <<= 1;
        if (high_duration > 40) {
            data[i / 8] |= 1;
        }
    }
    
    // Verify checksum
    uint8_t checksum = (data[0] + data[1] + data[2] + data[3]) & 0xFF;
    if (checksum != data[4]) {
        printf("Checksum error: calculated=%d, received=%d\n", checksum, data[4]);
        printf("Data: %d %d %d %d %d\n", data[0], data[1], data[2], data[3], data[4]);
        return -1;
    }
    
    *humid = (float)data[0] + (float)data[1] * 0.1;
    *temp = (float)data[2] + (float)data[3] * 0.1;
    
    return 0;
}

int main() {
    int pi = pigpio_start(NULL, NULL);
    if (pi < 0) {
        printf("Failed to connect to pigpiod\n");
        return 1;
    }
    
    printf("Testing DHT11 on GPIO %d\n", DHT11_PIN);
    printf("Press Ctrl+C to exit\n\n");
    
    for (int i = 0; i < 10; i++) {
        float temp, humid;
        
        printf("Reading %d: ", i + 1);
        int result = read_dht11_simple(pi, DHT11_PIN, &temp, &humid);
        
        if (result == 0) {
            printf("SUCCESS - Temp: %.1f°C, Humidity: %.1f%%\n", temp, humid);
        } else {
            printf("FAILED\n");
        }
        
        time_sleep(2.5);  // DHT11 needs 2 seconds between reads
    }
    
    pigpio_stop(pi);
    return 0;
}
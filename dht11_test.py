import pigpio
import time

# GPIO pin where DHT11 data pin is connected
DHT_GPIO = 4

pi = pigpio.pi()

if not pi.connected:
    print("Could not connect to pigpio daemon!")
    exit()

def read_dht11():
    pi.set_mode(DHT_GPIO, pigpio.OUTPUT)
    pi.write(DHT_GPIO, 0)
    time.sleep(0.018)  # 18ms low

    pi.set_mode(DHT_GPIO, pigpio.INPUT)

    data = []
    count = 0

    while pi.read(DHT_GPIO) == 1:
        count += 1

    for i in range(85):
        count = 0
        while pi.read(DHT_GPIO) == 0:
            count += 1
        count = 0
        while pi.read(DHT_GPIO) == 1:
            count += 1
        data.append(count)

    bits = []
    for i in range(0, len(data), 2):
        bits.append(1 if data[i] > 16 else 0)

    humidity = int("".join(map(str, bits[0:8])), 2)
    temperature = int("".join(map(str, bits[16:24])), 2)

    return humidity, temperature

try:
    humidity, temperature = read_dht11()
    print(f"Humidity: {humidity}%")
    print(f"Temperature: {temperature}°C")

except Exception as e:
    print("Error reading DHT11:", e)

finally:
    pi.stop()

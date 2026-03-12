#!/bin/bash

echo "==================================================="
echo "DHT11 Standalone Test"
echo "==================================================="

cd "$(dirname "$0")"

echo -e "\nCompiling test program..."
if gcc test_dht11_standalone.c -o test_dht11_standalone -lpigpiod_if2 -pthread -lrt; then
    echo "✓ Compilation successful"
    echo ""
    echo "Running test (10 readings with 2.5s interval)..."
    echo "This will help diagnose if DHT11 is working at all."
    echo ""
    ./test_dht11_standalone
    rm -f test_dht11_standalone
else
    echo "✗ Compilation failed"
    exit 1
fi

echo ""
echo "==================================================="
echo "Analysis:"
echo "==================================================="
echo ""
echo "If you see SUCCESS readings:"
echo "  → DHT11 hardware is working"
echo "  → Problem is in the ROS2 driver timing"
echo "  → Try increasing delays in the driver"
echo ""
echo "If you see FAILED readings:"
echo "  → Check wiring (run ./test_dht11_wiring.sh)"
echo "  → Add 4.7kΩ pull-up resistor"
echo "  → DHT11 may be faulty"
echo "  → Try DHT22 instead (more reliable)"
echo ""
echo "If you see timeout errors:"
echo "  → DHT11 not responding (check power/wiring)"
echo "  → Wrong GPIO pin (should be GPIO 4)"
echo ""
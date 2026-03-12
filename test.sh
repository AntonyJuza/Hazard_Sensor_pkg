#!/bin/bash

# pigpiod Connection Diagnostic Tool

echo "==================================================="
echo "pigpiod Connection Diagnostic"
echo "==================================================="

# Check if pigpiod is running
echo -e "\n[1/5] Checking if pigpiod is running..."
if systemctl is-active --quiet pigpiod; then
    echo "✓ pigpiod service is active"
    systemctl status pigpiod --no-pager | grep -E "(Active|Main PID)"
else
    echo "✗ pigpiod service is NOT running"
    echo ""
    echo "Starting pigpiod..."
    sudo systemctl start pigpiod
    sleep 1
    if systemctl is-active --quiet pigpiod; then
        echo "✓ pigpiod started successfully"
    else
        echo "✗ Failed to start pigpiod"
        exit 1
    fi
fi

# Check if pigpiod is listening
echo -e "\n[2/5] Checking if pigpiod is listening on port 8888..."
if netstat -tln 2>/dev/null | grep -q ":8888 "; then
    echo "✓ pigpiod is listening on port 8888"
elif ss -tln 2>/dev/null | grep -q ":8888 "; then
    echo "✓ pigpiod is listening on port 8888"
else
    echo "⚠️  Cannot detect port 8888 listener (netstat/ss not available or port not open)"
fi

# Check pigpiod process
echo -e "\n[3/5] Checking pigpiod process..."
if pgrep pigpiod > /dev/null; then
    echo "✓ pigpiod process is running"
    ps aux | grep pigpiod | grep -v grep
else
    echo "✗ pigpiod process not found"
fi

# Test connection with C program
echo -e "\n[4/5] Testing pigpiod connection with test program..."
cat > /tmp/test_pigpiod.c << 'EOF'
#include <stdio.h>
#include <pigpiod_if2.h>

int main() {
    int pi = pigpio_start(NULL, NULL);
    
    if (pi < 0) {
        printf("✗ Failed to connect to pigpiod daemon\n");
        printf("   Error code: %d\n", pi);
        if (pi == -1) printf("   Error: gpioInitialise failed\n");
        if (pi == -2) printf("   Error: Socket error\n");
        return 1;
    }
    
    printf("✓ Successfully connected to pigpiod daemon\n");
    printf("   Connection handle: %d\n", pi);
    
    // Try to set a GPIO mode
    int result = set_mode(pi, 4, PI_INPUT);
    if (result == 0) {
        printf("✓ GPIO operations working\n");
    } else {
        printf("✗ GPIO operation failed: %d\n", result);
    }
    
    pigpio_stop(pi);
    return 0;
}
EOF

if gcc /tmp/test_pigpiod.c -o /tmp/test_pigpiod -lpigpiod_if2 -pthread -lrt 2>/dev/null; then
    /tmp/test_pigpiod
    rm -f /tmp/test_pigpiod.c /tmp/test_pigpiod
else
    echo "✗ Failed to compile test program"
    echo "   Make sure libpigpiod-if-dev is installed:"
    echo "   sudo apt-get install libpigpiod-if-dev"
fi

# Check library installation
echo -e "\n[5/5] Checking pigpio libraries..."
echo "Checking for libpigpiod_if2:"
if ldconfig -p | grep -q libpigpiod_if2; then
    echo "✓ libpigpiod_if2 found"
    ldconfig -p | grep libpigpiod_if2
else
    echo "✗ libpigpiod_if2 NOT found"
    echo "   Install with: sudo apt-get install libpigpiod-if-dev"
fi

echo -e "\n==================================================="
echo "Diagnostic Complete"
echo "==================================================="
echo ""
echo "If all checks passed, your setup should work."
echo "If not, try:"
echo "  1. sudo systemctl restart pigpiod"
echo "  2. sudo apt-get install --reinstall pigpio libpigpiod-if-dev"
echo "  3. Check /var/log/syslog for pigpiod errors"
echo "==================================================="
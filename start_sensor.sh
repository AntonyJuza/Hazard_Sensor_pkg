#!/bin/bash

# Sensor Package Launcher
# Automatically checks and starts pigpiod daemon before launching sensors

echo "==================================================="
echo "ROS2 Sensor Package Launcher"
echo "==================================================="

# Check if pigpiod is running
if ! systemctl is-active --quiet pigpiod; then
    echo ""
    echo "⚠️  pigpiod daemon is not running!"
    echo "   Starting pigpiod daemon..."
    echo ""
    sudo systemctl start pigpiod
    sleep 1
    
    if systemctl is-active --quiet pigpiod; then
        echo "✓ pigpiod daemon started successfully"
    else
        echo "✗ Failed to start pigpiod daemon"
        echo ""
        echo "Please run manually:"
        echo "  sudo systemctl start pigpiod"
        exit 1
    fi
else
    echo "✓ pigpiod daemon is running"
fi

# Enable pigpiod on boot if not already enabled
if ! systemctl is-enabled --quiet pigpiod; then
    echo ""
    echo "Enabling pigpiod to start on boot..."
    sudo systemctl enable pigpiod
    echo "✓ pigpiod will start automatically on boot"
fi

echo ""
echo "==================================================="
echo "Launching sensor nodes..."
echo "==================================================="
echo ""

# Source ROS2 workspace
if [ -f ~/flame_robot_ws/install/setup.bash ]; then
    source ~/flame_robot_ws/install/setup.bash
elif [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
else
    echo "⚠️  ROS2 workspace not found in ~/flame_robot_ws or ~/ros2_ws"
    echo "   Please source your workspace manually first"
fi

# Launch the sensors
ros2 launch sensor_pkg sensors.launch.py
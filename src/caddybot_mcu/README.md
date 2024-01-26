# caddybot_mcu
caddybot_mcu node on MCU

## micro-ROS
[micro-ROS](https://micro.ros.org/) bridges the gap between resource-constrained microcontrollers (MCUs)
and larger processors that use ROS. micro-ROS agent brokers messages between ROS nodes.

![micro-ROS architecture](/img/micro-ROS.png)

## caddybot_mcu
caddybot_mcu node subscribes to velocity message and controls motors accordingly. it publishes IMU and odometry messages.<BR>
caddybot_mcu node subscribes to led message and controls leds accordingly.<BR>
caddybot_mcu handles buttons and publishes mode messages. it provides get_mode service to get the current mode.

![caddybot_mcu definition](/img/mcu.png)

# About
A better ROS2 driver for the [WaterLinked DVL-A50](https://store.waterlinked.com/product/dvl-a50/). Although there are a couple of other ROS2 drivers out there, I found that all of them either were very basic (e.g. no configuration, no service calls), had bugs or did not work at all. 

I based my version on [paagutie/dvl-a50](https://github.com/paagutie/dvl-a50), but more or less rewrote the entire driver and node, implementing the following features:

- Driver itself is ROS independent and can be used as a shared library
- Proper lifecycle node
- Using [marine_acoustic_msgs](https://github.com/apl-ocean-engineering/marine_msgs/tree/ros2/marine_acoustic_msgs) and standard messages instead of custom message types
- Provides services for documented commands
- Allows configuration of DVL on startup


# Dependencies
- [marine_acoustic_msgs](https://github.com/apl-ocean-engineering/marine_msgs/tree/ros2/marine_acoustic_msgs)
- [JSON for Modern C++](https://github.com/nlohmann/json) (submodule, see `include/dvl_a50/json/`)


# Basic Use
To start the driver with the default configuration (see below), find out the IP address of the DVL and then run:
```bash
$ ros2 run dvl_a50 dvl_a50_node --ros-args -p ip_address:='192.168.194.95'
```

This method will require you to manually transition to its active state. To do this, execute the following in a separate terminal:
```bash
$ ros2 lifecycle set /dvl_a50 configure
$ ros2 lifecycle set /dvl_a50 activate
```

Finally, the driver will disable automatic pinging by default to prevent overheating when out of water. Use either of the following service calls to start receiving pings. Reports will be published as per the topics defined further down.
```bash
# Request a single ping
$ ros2 service call dvl_a50/trigger_ping
```

```bash
# Automatic pinging
$ ros2 service call dvl_a50/enable
```


# Topics & Services
Data from the DVL is published on the following topics:
- `dvl/velocity`: _marine_acoustic_msgs/Dvl_
- `dvl/dead_reckoning`: _geometry_msgs/PoseWithCovarianceStamped_
- `dvl/odometry`: _nav_msgs/Odometry_

The velocity report also fills in the `beam_quality` array of the `Dvl` message using the Received Signal Strength Indicator (RSSI) reported for each beam. The values are in dBm and thus negative. Going counterclockwise from the cable, the transducers' indices are `1, 2, 3, 0`. See also the [official documentation](https://waterlinked.github.io/dvl/dvl-a50/).

Since the DVL-A50 reports the velocity and dead reckoning at different frequencies, the odometry is published every time either of them is received, with only the `pose` or `twist` updated respectively.

Furthermore, the node will provide the following services. All services use `std_msgs/Trigger`, i.e. they don't take any parameters and return a success state and error/result message.
- `enable`: Enable automatic pinging. The DVL will turn off when it's close to overheating, but this should still only be done when it is submerged. Also note the `enable_on_activate` parameter.
- `disable`: Disable automatic pinging.
- `get_config`: Return the DVL's configuration. If successful, the result will be encoded as json in the `message` field.
- `calibrate_gyro`: Trigger calibration of the gyro.
- `reset_dead_reckoning`: Reset the dead reckoning position estimator.
- `trigger_ping`: Trigger a single ping. This will disable automatic pinging.


# Configuration
When using the default launch file, the configuration will be loaded from `config/dvl_a50.yml`. In general, the following parameters are recognized:
- `ip_address`: IP address of the DVL. *string, **Required***.
- `frame`: The DVL's measuring and publishing frame. *string, default=dvl_a50_link*.
- `rate`: Rate at which to handle messages. Even though the DVL-A50 takes velocity measurements at <=15Hz it is good to set a higher rate here so that additional messages can be handled as well (e.g. dead reckoning reports, command responses). *double, default=30.0*.
- `enable_on_activate`: Enable automatic pinging when the lifecycle node is activated. Automatic pinging will *always* be disabled when the driver is deactivated. *boolean, default=false*.
- `speed_of_sound`: The speed of sound the DVL should assume (m/s). *int, default=1500*.
- `enable_led`: Whether the LED on the side of the DVL should be enabled. *boolean, default=true*.
- `mounting_rotation_offset`: Clockwise rotation of the DVL in degrees relative to the vehicle frame. *int, default=0*.
- `range_mode`: See [range mode configuration](https://waterlinked.github.io/dvl/dvl-protocol/#range-mode-configuration). *string, default=auto*.

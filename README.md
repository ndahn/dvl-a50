# About
This is a ROS2 driver for the [WaterLinked DVL-A50](https://store.waterlinked.com/product/dvl-a50/). Although there are a couple of other ROS2 drivers out there, I found that all of them either were very basic (e.g. no configuration, no service calls), had bugs or did not work at all. 

I based my version on [paagutie/dvl-a50](https://github.com/paagutie/dvl-a50), but more or less rewrote the entire driver and node, implementing the following features:

- Driver compiles as a ROS-independent shared library
- Proper lifecycle node
- Using [marine_acoustic_msgs](https://github.com/apl-ocean-engineering/marine_msgs/tree/ros2/marine_acoustic_msgs) and standard messages instead of custom message types
- Provides services for documented commands
- Allows configuration of DVL on startup


# Dependencies
- [marine_acoustic_msgs](https://github.com/apl-ocean-engineering/marine_msgs/tree/ros2/marine_acoustic_msgs)
- [JSON for Modern C++](https://github.com/nlohmann/json) (submodule, see `include/dvl_a50/json/`)


# Topics & Services
Data from the DVL is published on the following topics:
- _dvl/velocity_ - `marine_acoustic_msgs/Dvl`
- _dvl/dead_reckoning_ - `geometry_msgs/PoseWithCovarianceStamped`
- _dvl/odometry_ - `nav_msgs/Odometry`

The velocity report also fills in the `beam_quality` array of the _Dvl_ message using the Received Signal Strength Indicator (RSSI) reported for each beam. The valids are in dBm and thus negative. Going counterclockwise from the cable, the beam pads' indices are `1, 2, 3, 0`. See also the [official documentation](https://waterlinked.github.io/dvl/dvl-a50/).

Since the DVL-A50 reports the velocity and dead reckoning at different frequencies, the odometry is published every time either of them is received, with only the `pose` or `twist` updated respectively.

Furthermore, the node will provide the following services. All services use `std_msgs/Trigger`, i.e. they don't take any parameters and return a success state and error message.
- _enable_: Enable automatic pinging. The DVL will turn off when it's close to overheating, but this should still only be done when it is submerged.
- _disable_: Disable automatic pinging.
- _get_config_: Return the DVL's configuration. If successful, the result will be encoded as json in the `message` field.
- _calibrate_gyro_: Trigger calibration of the gyro.
- _reset_dead_reckoning_: Reset the dead reckoning position estimator.
- _trigger_ping_: Trigger a single ping. This will disable automatic pinging.


# Configuration
When using the default launch file, the configuration will be loaded from `config/dvl_a50.yml`. In general, the following parameters are recognized:
- _ip_address_: IP address of the DVL. **Required**.
- _frame_: The DVL's measuring and publishing frame. Default is `dvl_a50`.
- _rate_: Rate at which to handle messages. Even though the DVL-A50 takes velocity measurements at <=15Hz it is good to set a higher rate here so that additional messages can be handled as well (e.g. dead reckoning reports, command responses). Default is `30.0`.
- _speed_of_sound_: The speed of sound to assume (m/s). Default is `1500`.
- _enable_led_: Whether the LED on the side of the DVL should be enabled. Default is `true`.
- _mountig_rotation_offset_: Rotation of the DVL in degrees relative to the vehicle frame. Default is `0`.
- _range_mode_: See [range mode configuration](https://waterlinked.github.io/dvl/dvl-protocol/#range-mode-configuration). Default is `auto`.

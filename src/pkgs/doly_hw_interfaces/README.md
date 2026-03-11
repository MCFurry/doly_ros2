# doly_hw_interfaces

Hardware interface nodes for Doly exposed as ROS 2 topics, services, and actions.

### drive_interface_node
Bridges `cmd_vel` wheel commands to Doly drive motors and also publishes IMU data from the IMU managed by `DriveControl`.

#### Subscribed Topics

* **`cmd_vel`** ([`geometry_msgs/msg/TwistStamped`]) Desired robot linear/angular velocity command.

#### Published Topics

* **`imu_data`** ([`sensor_msgs/msg/Imu`]) IMU orientation and linear acceleration estimated by Doly IMU callbacks.

#### Services

* None.

#### Actions

* None.

#### Parameters

* **`imu_freq`** (`double`)[Hz] Publish rate for `imu_data`.
* **`imu_frame_id`** (`string`)[-] Frame id used in IMU messages.

### arm_interface_node
Controls left and right arm target angles and publishes current arm state angles.

#### Subscribed Topics

* **`left_arm_angle`** ([`std_msgs/msg/Float32`]) Target left arm angle in radians.
* **`right_arm_angle`** ([`std_msgs/msg/Float32`]) Target right arm angle in radians.

#### Published Topics

* **`left_arm_state`** ([`std_msgs/msg/Float32`]) Current left arm angle in radians.
* **`right_arm_state`** ([`std_msgs/msg/Float32`]) Current right arm angle in radians.

#### Services

* None.

#### Actions

* None.

#### Parameters

* **`freq`** (`double`)[Hz] State publish rate for arm state topics.

### led_interface_node
Sets left and right RGB LEDs to an instant solid color based on topic input.

#### Subscribed Topics

* **`led/left`** ([`std_msgs/msg/ColorRGBA`]) Target color for left LED (`r`,`g`,`b` in 0.0-1.0 range, alpha ignored).
* **`led/right`** ([`std_msgs/msg/ColorRGBA`]) Target color for right LED (`r`,`g`,`b` in 0.0-1.0 range, alpha ignored).

#### Published Topics

* None.

#### Services

* None.

#### Actions

* None.

#### Parameters

* None.

### tof_interface_node
Polls both ToF sensors at a configurable rate and publishes range messages for each side.

#### Subscribed Topics

* None.

#### Published Topics

* **`tof/left`** ([`sensor_msgs/msg/Range`]) Left ToF distance reading.
* **`tof/right`** ([`sensor_msgs/msg/Range`]) Right ToF distance reading.

#### Services

* None.

#### Actions

* None.

#### Parameters

* **`freq`** (`double`)[Hz] Poll/publish rate for ToF sensor data.

### edge_interface_node
Publishes per-edge-sensor ground detection state from edge sensor change events.

#### Subscribed Topics

* None.

#### Published Topics

* **`edge/front_left`** ([`std_msgs/msg/Bool`]) Front-left sensor ground state (`true` = ground detected).
* **`edge/front_right`** ([`std_msgs/msg/Bool`]) Front-right sensor ground state (`true` = ground detected).
* **`edge/back_left`** ([`std_msgs/msg/Bool`]) Back-left sensor ground state (`true` = ground detected).
* **`edge/back_right`** ([`std_msgs/msg/Bool`]) Back-right sensor ground state (`true` = ground detected).

#### Services

* None.

#### Actions

* None.

#### Parameters

* None.

### fan_interface_node
Initializes fan control in automatic mode and offers a service to switch to manual percentage or back to automatic mode.

#### Subscribed Topics

* None.

#### Published Topics

* None.

#### Services

* **`set_fan_speed`** ([`doly_msgs/srv/SetFanSpeed`]) Set fan speed percentage (`0..100`) or `-1` for automatic mode.

#### Actions

* None.

#### Parameters

* None.

### tts_interface_node
Runs text-to-speech synthesis and audio playback through an action; action result returns when speech playback finishes.

#### Subscribed Topics

* None.

#### Published Topics

* None.

#### Services

* None.

#### Actions

* **`speak`** ([`doly_msgs/action/Speak`]) Synthesize and play speech from input text; succeeds when playback completes.

#### Parameters

* **`tts_output_path`** (`string`) Path of generated speech audio file used for playback.

### touch_interface_node
Publishes touch press/release events; touch activity pattern events are intentionally ignored.

#### Subscribed Topics

* None.

#### Published Topics

* **`touch/event`** ([`doly_msgs/msg/TouchEvent`]) Touch side/state event matching SDK touch defines.

#### Services

* None.

#### Actions

* None.

#### Parameters

* None.

### battery_status_broadcaster_node
Reads battery telemetry from sysfs and publishes standardized battery state messages.

#### Subscribed Topics

* None.

#### Published Topics

* **`battery_status`** ([`sensor_msgs/msg/BatteryState`]) Battery voltage/current/percentage and cell voltages derived from hwmon data.

#### Services

* None.

#### Actions

* None.

#### Parameters

* **`current_path`** (`string`) Sysfs path to current input (nA).
* **`bus_voltage_path`** (`string`) Sysfs path to bus voltage input (mV).
* **`shunt_voltage_path`** (`string`) Sysfs path to shunt voltage input (mV).

### eye_interface_node
Controls eye animations through an action and supports eye style updates via service.

#### Subscribed Topics

* None.

#### Published Topics

* None.

#### Services

* **`set_eye_type`** ([`doly_msgs/srv/SetEyeType`]) Set iris shape and iris/background color presets.

#### Actions

* **`set_eye_animation`** ([`doly_msgs/action/EyeAnimation`]) Start a named eye animation and return when it completes or aborts.

#### Parameters

* None.

# trans_communication

## Overview
This package manages communication between obs(intball2) and ocs(ground).
Telecommands and telemetries are specified in [config.yml](config/config.yml).

## Dependencies

* ib2_msgs
* PyYAML

## Usage
To start node
``` bash
roslaunch trans_communication bringup.launch
```


## Nodes
### trans_communication
Manage communication.

#### Subscrived Topics
This node subscribes topics specified in `telemetry` in [config.yml](config/config.yml).

#### Published Topics
This node subscribes topics specified in `telecommand` in [config.yml](config/config.yml).


#### Parameters

* **~config_path** (string) :

    Path to yaml file of config.

* **~ocs_host** (string) :

    Host address of ocs.

* **~ocs_port** (int) :

    Port number of ocs to send telemetry.

* **~receive_port** (int) :

    Port number of obs to receive telecommand from ocs.

* **~lex_min_user_data_bytes** (int) :

    Minimum data size for sending telemetry to the LEHX.

* **~telemetry_rate** (int) :

    Publish rate of the telemetry.

* **~wait_after_send_error** (int) :

    Waiting time after detecting a telemetry transmission error.

* **~ros_wait_for_service_time** (int) :

    Time to wait for ROS service server startup.
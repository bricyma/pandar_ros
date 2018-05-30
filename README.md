## Dependency
```
1. ROS
2. sudo apt install libpcap-dev libyaml-cpp-dev
```


## Run
### Pandar40
```
roslaunch hesai_lidar p40.launch 
```

### Pandar40P
#### Online mode: 
input: UDP packets
output: ```/pandar_packets, /pandar_points```
```
roslaunch hesai_lidar p40p.launch mode:=0
```
#### Offline mode:
input: ```/pandar_packets ```
output: ```/pandar_points```
```
roslaunch hesai_lidar p40p.launch mode:=1
```

## Parameters:
```
	<arg name="mode" default="0"/> driver mode, 0: online, 1: offline
	<arg name="server_ip" default="192.168.20.51"/> pandora's ip
	<arg name="server_port"  default="9870"/>       pandora's camera port
	<arg name="lidar_recv_port"  default="8080"/>   lidar's port
	<arg name="gps_port"  default="10110"/>         gps's port
	<arg name="start_angle"  default="0"/>          lidar's start angle

  ......

	<param name="calibration_file" type="string" value="$(find hesai_lidar)/config/calibration.yml"/>  Calibration of Camera (Pandora Only, instrinsic and exstrinsic)
	<param name="lidar_correction_file"  type="string" value="$(find hesai_lidar)/config/correction.csv"/> Calibration of Lidar

```

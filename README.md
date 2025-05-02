# Robotteknologi-4.-semester

## Raspberry to Cube orange
For the Cube Orange Plus to communicate, some parameters need to be changed. In the GitHub repository, the file CubeFirmware.params contains working parameters for functional flight and Serial uORB communication.  
Install QGroundControl by following the guide: [QGroundControl Installation Guide](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html)  
Connect the computer to the drone using a micro USB cable or radio transmitter and go to:  
```QGroundControl -> Vehicle setup -> Change parameters -> Upper right corner -> Load parameters -> Choose paramters```

Next, the Raspberry Pi and Cube Orange need to be connected as follows:  
| **RPi4 Pin** | **PX4 Telem 2 Pin** | **Connection**         |
|-----------------------|---------------------|------------------------|
| 8                     | Pin 3               | RPi4 TX → PX4 RX       |
| 10                    | Pin 2               | RPi4 RX → PX4 TX       |
| 9                     | Pin 6               | RPi4 GND → PX4 GND     |

The Raspberry Pi must run Ubuntu 22.04 (required for ROS 2 Humble), and it is recommended to use the server version to reduce CPU usage.  

## Setup of software
# Setup serial port
First, check if Serial0 is used by any conflicting services:  
```bash
sudo nano /boot/firmware/cmdline.txt
```  
Delete any mentions of Serial0.  
Next, edit the configuration:  
```bash
sudo nano /boot/firmware/config.txt
```  
Add the line  
```bash
enable_uart=1
```  
Add your user to the serial port group:  
```bash
sudo usermod -a -G dialout $(whoami)
```  
and reboot:  
```bash
sudo reboot
```  

# Setup ROS2 Humble
Install ROS2 Humble by following this guide [ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

# Setup Micro XRCE-DDS Agent
Set up the Micro XRCE-DDS Agent. More information can be found here: [Setup Micro XRCE-DDS Agent & Client](https://docs.px4.io/main/en/ros2/user_guide.html#setup-micro-xrce-dds-agent-client)
```bash
git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```
To run the agent, use:  
```bash
MicroXRCEAgent serial --dev /dev/serial0 -b 921600
```  

# Setup PX4_msgs for reading and writing to topics
Currently, the ROS 2 topics from the Micro XRCE-DDS agent are not readable. This is due to message formatting. To resolve this, use px4_msgs as part of your src folder or source it in your environment: 
```bash
git clone -b release/1.15 https://github.com/PX4/px4_msgs.git
```
Build with colcon, then source:  
```bash
source install/setup.bash
```
Where the ROS2 topics then can be echoed.  
**Important**
It is important to use px4_msgs version 1.15. This version is intended for communication with a physical drone. The main branch is for simulation in Gazebo.
If updated to a newer version, ensure that messages like vehicleAttitudeSetpoint include the thrust variable—this is a key difference.  

# Clone and Build
Now that the dependencies are installed, the drone software can be built. Clone the workshop repo and build:  
```bash
git clone https://github.com/jens1906/Robotteknologi-4.-semester.git
cd Robotteknologi-4.-semester/ws_drone_control
colcon build
```
To run the code:  
```bash
source install/setup.bash
ros2 run drone_software offboard_control_node
```
The control panel should explain the implemented commands.  
  
Note: This code can be built and run using both ROS 2 Humble and ROS 2 Jazzy. This means it can run locally on the Raspberry Pi or on another computer on the same network. In the latter case, the Raspberry acts as a ROS 2 topic router. Some issues with this can occur and the most typical can be fixed by:

**Firewall access**  
Ensure your firewall does not block ROS 2 topics:  
```bash
 sudo ufw disable
 sudo ufw allow proto udp from any to 224.0.0.0/4 
 # Allow ROS 2 default Fast DDS ports
 sudo ufw allow 7400:7600/udp
 sudo ufw allow 7400:7600/tcp
 sudo ufw enable
```

**Restart ROS**  
Sometimes a restart of ROS is useful which is done by:  
```bash
ros2 daemon stop
ros2 daemon start
```

## Contributors
This project was developed by a group of students from Aalborg University, as part of the 4th semester in the Robot technologi bachalor.  
- André Vester Magnusson
- Daniel Holst Dreier
- Jens Søby Hansen
- Mads Majlund Thomsen
- Mayvand Basir Hotaki
- Thor Ivarsen Østergaard




**Used sources**  
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html  
https://docs.px4.io/main/en/ros2/user_guide.html#setup-micro-xrce-dds-agent-client  
https://docs.px4.io/main/en/flight_modes/offboard.html?utm_source  
https://ardupilot.org/copter/docs/common-thecubeorange-overview.html  
https://github.com/PX4/px4_msgs/tree/release/1.15  
https://github.com/AAU-Space-Robotics/drone-software/tree/main

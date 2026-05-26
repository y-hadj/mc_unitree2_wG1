# mc_unitree2
Interface between [Unitree robots](https://github.com/unitreerobotics/unitree_ros2/tree/master/robots) and [mc_rtc](https://jrl-umi3218.github.io/mc_rtc). Provides connectivity with [Go2](https://www.unitree.com/products/go2/) robots.

## 1. Required dependencies

 - [mc_rtc](https://jrl-umi3218.github.io/mc_rtc/)
 - [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2)

## 2. Install dependencies

### unitree_sdk2
 - Install the include files
```
$ git clone https://github.com/y-hadj/unitree_sdk2.git #fork with necessary headers
$ mkdir build
$ cmake ..
$ make ; make install
```
## 3. Install this project

### Build instructions

```
$ cd src
$ git clone https://github.com/mc_unitree2
$ mkdir build
$ cd build
$ cmake ..
$ make ; make install
```

## 4. Usage

### Configuration

There is an example in `etc/` which can be passed as an argument when running the controller to override the default values of this repository.  
The network interface is loopback ("lo") by default for simulation, but can be replace by your ethernet interface (check `ifconfig`) for deployment.

### Running the program

Now, turn on the robot and connect it to your host machine with an ethernet cable then configure the netwrok interface:
```
$ ifconfig #if command not found, install it with: sudo apt install net-tools
```
from there note G1 interface name and your host's IP adress. Then set your IP to the same subnet as G1's
```
$ sudo ip addr add <Host-IP-adress>/24 dev <G1-interface-name> 
```
and verify connectivity
```
$ ping 192.168.123.161  #G1's default IP
```
Now, launch with loopback to make sure the passed configuration file loads without connecting the robot
```
$ MCControlG1 --conf ~/superbuild/install/etc/mc_unitree/mc_rtc_example.yaml --network lo
```
then press enter to transit from each state shown in the terminal accordingly. If loopback is clean on mc-rtc-magnum with no task errors, send the commands to the robot
```
$ MCControlG1 --conf ~/superbuild/install/etc/mc_unitree/mc_rtc_example.yaml --network <G1-interface-name>
```

<ins>PS.</ins> You can also manually configure the network by setting the IPv4 protocol's adress to <Host-IP-adress> and netmask to 255.255.255.0


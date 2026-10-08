# mc_unitree2
Interface between [Unitree robots](https://github.com/unitreerobotics/unitree_ros2/tree/master/robots) and [mc_rtc](https://jrl-umi3218.github.io/mc_rtc). Provides connectivity with [Go2](https://www.unitree.com/products/go2/) robots.

There are two options to build:
- With Nix
- From source

## Nix

To get started with Nix, use

```
nix develop .#mc-rtc-superbuild-g1
```

This puts you in a shell with all dependencies built and installed, and ready to use in simulation / with the real robot.

## From source
### 1. Required dependencies

 - [mc_rtc](https://jrl-umi3218.github.io/mc_rtc/)
 - [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2)

### 2. Install dependencies

#### unitree_sdk2
 - Install the include files
```
$ git clone https://github.com/y-hadj/unitree_sdk2.git #fork with necessary headers
$ mkdir build
$ cmake ..
$ make ; make install
```
### 3. Install this project

#### Build instructions

```
$ cd src
$ git clone https://github.com/mc_unitree2
$ mkdir build
$ cd build
$ cmake ..
$ make ; make install
```

## Usage

#### Configuration

There is an example in `etc/` which can be passed as an argument when running the controller to override the default values of this repository.  
The network interface is loopback ("lo") by default for simulation, but can be replace by your ethernet interface (check `ifconfig`) for deployment.

#### Running the program

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

### Adding support for Brainco Revo2 Hands
Brainco Revo2 hands are connected to the G1 robot by default, this means we have to SSH into the robot's Jeston Orin and install brainco_hand_service which is a bridge between the Serial/CAN protocols of the Revo2 hands to DDS for easy unitree manipulation. 
```bash
ssh unitree@192.168.123.164	#pwd is 123
```
For security reasons, the jetson doesnt have access to the internet. Therefore, we have to mirror brainco_hand_service and its dependencies into it. Run this on your host machine on a separate terminal session
```bash
cd /tmp

#1) clone unitree_sdk2, a dependency of brainco_hand_service
git clone https://github.com/unitreerobotics/unitree_sdk2.git
tar czf unitree_sdk2.tar.gz unitree_sdk2
scp unitree_sdk2.tar.gz unitree@192.168.123.164:~/

#2) clone brainco_hand_service 
git clone --recursive https://github.com/unitreerobotics/brainco_hand_service.git
tar czf brainco_hand_service.tar.gz brainco_hand_service
scp brainco_hand_service.tar.gz unitree@192.168.123.164:~/
```
then in the terminal session where G1's jeston was ssh-ed in, mirror and build the packages
```bash
#1) mirror and build unitree_sdk2
cd ~
tar xzf unitree_sdk2.tar.gz
cd unitree_sdk2
mkdir build & cd build
cmake ..
sudo make install

#2) mirror and build brainco_hand_service
cd ~
tar xzf brainco_hand_service.tar.gz
cd brainco_hand_service
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
```
Now we can control the robot, start another ssh session into the G1's jetson on a separate terminal then
```bash
#on first jetson terminal
cd ~/brainco_hand_service/bin
sudo ./brainco_hand_server --network eth0

#on second jetson terminal
cd ~/brainco_hand_service/bin
sudo ./test_brainco_hand_server left	#can change arg to right for the right hand testing
```
and the hands should show a variety of gripping motions, which confirms they work. Now we can launch them on our controller by adding ``-DGENERATE_G1_REVO2_CONTROLLER=ON`` on ~/superbuild/mc-rtc-superbuild-G1-Manipulation/extensions/local.cmake then recompiling mc_unitree2 and running the generated executable MCControlG1Revo2.
```bash
#on the devcontainer
cd ~/superbuild/mc-rtc-superbuild-G1-Manipulation/build
cmake .. && cmake --build . --target mc_unitree2 
MCControlG1Revo2 --conf ~/superbuild/install/etc/mc_unitree/mc_rtc_example.yaml --network <G1-interface-name>
```


<ins>PS.</ins> If libfmt dev library is missing, download it 
```bash
#on your host machine
wget http://ports.ubuntu.com/pool/universe/f/fmtlib/libfmt-dev_8.1.1+ds1-2_arm64.deb	#depends on the sys architecture, here is ARM64
scp libfmt-dev_8.1.1+ds1-2_arm64.deb unitree@192.168.123.164:~/

#then on the G1 jetson
cd ~
sudo dpkg -i libfmt-dev_8.1.1+ds1-2_arm64.deb
cd ~/brainco_hand_service/build
make clean && make
```
and if u encounter this problem with other libraries, redo the same installation procedure.

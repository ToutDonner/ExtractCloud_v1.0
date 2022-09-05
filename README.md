# ExtractCloud_v1.0

第一步，确保系统中安装有[Livox-SDK](https://github.com/Livox-SDK/Livox-SDK)。

第二步，确保系统中可以编译[FAST-LIO](https://github.com/hku-mars/FAST_LIO)。

## Prerequisites
### 1 **Ubuntu** and **ROS**
**Ubuntu >= 16.04**

For **Ubuntu 18.04 or higher**, the **default** PCL and Eigen is enough for FAST-LIO to work normally.

ROS    >= Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 2. **PCL && Eigen**
PCL    >= 1.8,   Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).

Eigen  >= 3.3.4, Follow [Eigen Installation](http://eigen.tuxfamily.org/index.php?title=Main_Page).

### 3. **修改bash**

首先修改aaa.sh，将命令第一行修改至为cd /../../JLU_ectract_v1.0。以此电脑为例，命令为cd /home/crz/SLAM/JLU_ectract_v1.0，并给.sh赋予权限

然后修改FAST_LIO中的putoutCommand.cpp中第36行，将“”内的路径修改为本地aaa.sh路径

## Build
Clone the repository and catkin_make:

```
    cd ~/$A_ROS_DIR$/src
    git clone https://github.com/ToutDonner/ExtractCloud_v1.0.git
    catkin_make
    source devel/setup.bash
```

## Run
```
    cd ~/work_space/src
    source devel/setup.bash
    roslaunch extract_pc crz.launch
    rosrun fast_lio putoutCommand
 ```
 当需要提取点云时
 ```
    rostopic pub /command std_msgs/Bool true
 ```
 
 演示视频->[Viedo](https://www.bilibili.com/video/BV1oP4y1f7HL/)

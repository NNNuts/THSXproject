Sub topic  AgvControl/Float32MultiArray
data 里面九个数据
data[0] 工作模式 0：失能模式 1：速度模式
失能模式：轮毂电机松轴，data[1:4]无效
速度模式：单位m/s data[1:4] 是四个电机的速度
相对位置模式：单位m，相对于当前位置的变化量 data[1:4] 是四个电机的位置

data[5:8] 是四个步进电机位置，单位rad

Pub topic AgvData/Float32MultiArray
data 里面九个数据

data[0] 工作模式 0：失能模式 1：速度模式
失能模式：轮毂电机松轴，data[1:4]无效
速度模式：单位m/s data[1:4] 是四个电机的速度

data[5:8] 是四个步进电机位置，单位rad

接受指令和播报电机实际状态周期为100ms

按CTRL+SHIFT+B，然后回车编译文件

第一个终端 roscore
第二个终端 source ./devel/setup.bash
         rosrun HubMotor HubMotor
第三个终端 source ./devel/setup.bash
         rosrun HubMotor control

模式一 control_mode1 双阿克曼
模式二 control_mode2 斜移
模式三 control_mode3 平移自旋驻车
模式四 control_mode4 双阿克曼（多键） ps：必须root模式使用,即在第三个终端第一条指令sudo su
模式五 control_mode5 双阿克曼（左右松手即停）
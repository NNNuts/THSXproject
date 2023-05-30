topic  HubControl/Float32MultiArray
data 里面九个数据
data[0] 工作模式 0：失能模式 1：速度模式 2：相对位置模式
失能模式：轮毂电机松轴，data[1:4]无效
速度模式：单位m/s data[1:4] 是四个电机的速度
相对位置模式：单位m，相对于当前位置的变化量 data[1:4] 是四个电机的位置

data[5:8] 是四个步进电机位置，单位rad


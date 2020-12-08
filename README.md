# delta_ac
 Adaptive Control of Delta Robot Using xPC Target

## How to use
第一次控制实验，应当
1.按驱动器说明书连接电机和驱动器的主电
2.按下面“Wiring”接线，这样数据采集卡的输出电压就可以控制驱动器了
3.用网线连接主机和目标机
4.目标机中放入xpc启动光盘，开机
5.将主机ip修改为与目标机同一网段，使其可以进行TCP/IP通信
6.打开主机中的matlab，打开simulink中相应模型，点击右上角“build”按钮，等待编译完成，如果编译成功，目标机屏幕会有变化。
7.在matlab中输入xpcexplr命令，打开xpc控制界面，点击开始按钮，实验开始

## Naming convention of simulink modules
命名由从前到后被下划线分割的几个字段组成

第一个字段：基本类型
xpc意为：基于xpc组件的半实物仿真
traj意为：轨迹规划器的调试

第二个字段：控制对象
dlt意为：Delta机器人
test意为：测试

第三个字段：机器人执行的动作
adept意为：执行的是Adept Motion轨迹
hold意为：维持当前位置
homing意为：回原点（三个关节水平的位置）
p2p：执行的是点到点运动

6229：测试的是6229的功能
6601：测试的是6601的功能

第四个字段：控制算法
ctc意为：计算力矩法（computed torque control）
ctf意为：力矩前馈法（computed torque feedforward）
pid意为：PID控制
sa意为：自适应控制（self adaptive）

第五个字段：执行Adept Motion一次的时间
1:1s
7:700ms
204:204ms
4:400ms
25:250ms

第六个字段：执行次数
s意为：执行一次（single）
c意为：三个来回，共六次（cycle）

## Wiring
驱动器控制口（x4）的接线
驱动器1
青线----24v
橙黑----24v的GND
紫线----急停开关----24v的GND
橙线----A0接线盒


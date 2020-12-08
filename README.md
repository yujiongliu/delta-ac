# delta_ac
 Adaptive Control of Delta Robot Using xPC Target
 
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

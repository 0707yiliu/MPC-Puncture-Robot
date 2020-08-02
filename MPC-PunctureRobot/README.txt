Version: MATLAB 2015a

(运行含有bp神经网络拟合的控制器以及部分sin函数前，需要将全部文件和子文件载入路径)

穿刺平行机构机器人matlab运动学控制仿真：

（1）Kinematic Modle

机器人是平行并联机构，通过多关节控制一个link动作，用几何解法获得解耦后的运动学可行解
（the source file in MPC_Kalman.slx/robot.sfunction）
（the 2D solver file in needle2.slx）

（2）PID controller

并联四电机输入控制，可视为四输入单输出控制机构。
（the source file is needle3.slx）

（3）MPC controller （the solver file in MPC_Kalman.slx）



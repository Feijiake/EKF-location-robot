clear;
clc;


%% 初始参数设置
time = 0;
ContinueTime = 60;  %[s]
global dt;
dt = 0.1;                   % 时间间隔
Step = ceil((ContinueTime - time) / dt);
Tred = 100e-3;  %[m]               
a = [0.1, 0.1, 0.1, 0.1];  % [a1, a2, a3, a4]    
global Qt;
Qt = 0.001 ^2;                 % 陀螺仪观测误差方差

%% 初始位置定义
TruePosition = [0, 0, 0];   % 真实位置
OdoPosition = [0, 0, 0];    % 里程计位置
PreZt = 0;                  % 上次角速度观测
PreXt = [0, 0, 0];
PrePt = zeros(3, 3);
Robot_u = [1, 0.1]; %[Transration, Rotation]

%% 开始仿真
figure;
for i = 1 : Step
    time = time + dt;
    
    input = CalcU(Robot_u,Tred,dt);                % 获取当前输入
    Rt = CalcRt(a, input);                 % 控制输入协方差
    sigma = [Rt(1, 1), Rt(2, 2)];%模拟带误差的移动时的误差系数
    OdoPosition = CalcPositionWithError(OdoPosition, input,sigma);  % 加噪声的里程计
    TruePosition = CalcTruePosition(TruePosition, input);     % 真实位置更新
    ObsZt = GetIMU(PreZt, Robot_u(1, 2));     % 获取角速度观测（加噪声）

    [EstXt, PreXt, PrePt] = EKF_main(Robot_u, a, Qt, Tred, ObsZt,dt,sigma,PreXt,PrePt); % EKF估计

    % 可视化轨迹
    if rem(i, 10) == 0
        plot(TruePosition(1), TruePosition(2), '.b'); hold on;
        plot(OdoPosition(1), OdoPosition(2), '.k'); hold on;
        plot(EstXt(1), EstXt(2), '.r'); hold on;
        axis equal;
        legend('True', 'Odometry', 'EKF');
        drawnow
    end

    PreZt = ObsZt; % 保存角速度用于模拟陀螺仪积分误差
end

function theta = GetIMU(pre, u)  % 通过陀螺仪计算角度
    global Qt
    global dt
    
    theta = pre + (u * dt + normrnd(0, sqrt(Qt)));  % 正态分布噪声
    %disp(u);
    %disp(dt);
    %disp(theta);
   
end

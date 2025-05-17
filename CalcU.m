%差速机器人
%u机器人输入方程
%Tred 两轮之间的距离
%dt 采集时间
function out = CalcU(u,Tred,dt)
    
       
    velo.Tra = u(1);
    velo.Rot = u(2);
    
    Vr = (velo.Rot * Tred + 2 * velo.Tra) / 2;%差速机器人模型
    Vl = (-velo.Rot * Tred + 2 * velo.Tra) / 2;
    
    dSr = Vr * dt;
    dSl = Vl * dt;
    
    dS = (dSr + dSl) / 2;
    dTh = (dSr - dSl) / Tred;
    
    out = [dS, dTh];
end

%纯舵机打角机器人
%u机器人输入方程
%Tred 两轮之间的距离
%dt 采集时间

% function out = CalcU(u,Tred,dt)
% 
% 
%     v = u(1);       % 线速度
%     delta = u(2);   % 舵机转向角（单位：弧度）
% 
%     dS = v * dt;
%     dTh = (v / L) * tan(delta) * dt;
% 
%     out = [dS, dTh];
% end


%差速机器人
%u机器人输入方程
%Tred 两轮之间的距离
%dt 采集时间

% function out = CalcU(u,Tred,dt)
% 
% 
%     % u(1): 前轮线速度 v (m/s)
%     % u(2): 前轮转向角 delta (rad)
%     % u(3): 后轮差速角速度 omega_diff (rad/s)
% 
%     v_front = u(1);
%     delta = u(2);
%     omega_diff = u(3);
% 
%     % 计算后轮左右轮速度
%     % 这里假设总速度 v_front，加上差速导致左右轮速度不同
%     Vr = v_front + omega_diff * Tred / 2;
%     Vl = v_front - omega_diff * Tred / 2;
% 
%     % 计算前进距离增量，取左右轮平均速度
%     dS = (Vr + Vl) / 2 * dt;
% 
%     % 总角速度 = 前轮转向角产生的角速度 + 后轮差速产生的角速度
%     dTh = (v_front / L) * tan(delta) * dt + omega_diff * dt;
% 
%     out = [dS, dTh];
% end


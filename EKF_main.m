%Robot_u机器人状态矩阵
function [EstTraj, PreXt, PrePt] = EKF_main(Robot_u, a, Qt, Tred, ObsZt,dt,sigma,PreXt,PrePt)
   
    input = CalcU(Robot_u,Tred,dt);                % 获取当前输入
    Rt = CalcRt(a, input);                           % 控制噪声协方差
    At = CalcAt(PreXt, input);                       % 状态雅可比
    Wt = CalcWt(PreXt, input);                       % 控制输入雅可比

    EstXt = CalcPosition(PreXt, input);     % 状态预测（带噪声）
    EstXt = EstXt';                                  % 转置为列向量
    EstPt = At * PrePt * At' + Wt * Rt * Wt';        % 协方差预测

    Ht = [0, 0, 1];                                   % 观测模型（仅角度）
    St = Ht * EstPt * Ht' + Qt;                      % 观测协方差
    Kt = St \ (EstPt * Ht');                    % 卡尔曼增益

    EstXt = EstXt + Kt * (ObsZt - Ht * EstXt);       % 状态更新
    ExtPt = (eye(3) - Kt * Ht) * EstPt;              % 协方差更新

    PreXt = EstXt';                                  % 更新持久状态（行向量）
    PrePt = ExtPt;
    EstTraj = PreXt;                                 % 输出估计轨迹
end





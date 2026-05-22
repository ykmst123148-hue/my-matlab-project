%% 目標応答伝達関数 Hd の設計
clearvars;
close all;
clc;

%% パラメータ
Ts = 0.001;              % サンプリング周期 [s]
T_cycle = 2.0;           % 歩行周期 [s]
s = tf('s');
G = 1064/(s^2+56.9542*s+1043.1);
% 時間設定
t_LR = T_cycle * 0.12;                        % 沈み込み時間 
t_Stance_Total = T_cycle * 0.62;              % 荷重継続時間 
t_recover = T_cycle - t_LR - t_Stance_Total;  % 戻り動作の時間 

% 物理量
weight_torque = 0.431;    % 体重負荷 [Nm] 質量945g
target_sag_deg = 15.0;    % 目標沈み込み角度 [deg]
target_rad = target_sag_deg * pi / 180;

% 時刻設定
t_land = 1.0;                        % 着地時刻

t_takeoff = t_land + t_Stance_Total; % 離地時刻
t_end = t_takeoff + t_recover + 0.5; 
t = (0:Ts:t_end)';

%% 入力と目標軌道の生成
% 入力信号 u (矩形波)
u_weight = zeros(size(t));
idx_land = round(t_land/Ts) + 1;
idx_takeoff = round(t_takeoff/Ts) + 1;
if idx_takeoff > length(u_weight)
    idx_takeoff = length(u_weight);
end
u_weight(idx_land : idx_takeoff) = weight_torque;

% 参照軌道 y_ref
y_ref_minjerk = zeros(size(t));

% 沈み込み (MinJerk)
t_vec_LR = (0:Ts:t_LR)';
[pos_LR, ~] = generate_minjerk(0, target_rad, t_LR, t_vec_LR);
idx_LR_end = idx_land + length(pos_LR) - 1;
if idx_LR_end > length(y_ref_minjerk), idx_LR_end = length(y_ref_minjerk); end
y_ref_minjerk(idx_land : idx_LR_end) = pos_LR;

% ホールド
if idx_takeoff > length(y_ref_minjerk), idx_takeoff = length(y_ref_minjerk); end
y_ref_minjerk(idx_LR_end+1 : idx_takeoff) = target_rad;

% 戻り
t_vec_rec = (0:Ts:t_recover)';
[pos_rec, ~] = generate_minjerk(target_rad, 0, t_recover, t_vec_rec);
idx_rec_end = idx_takeoff + 1 + length(pos_rec) - 1;
if idx_takeoff + 1 <= length(y_ref_minjerk)
    if idx_rec_end > length(y_ref_minjerk)
        valid_len = length(y_ref_minjerk) - (idx_takeoff + 1) + 1;
        y_ref_minjerk(idx_takeoff+1 : end) = pos_rec(1:valid_len);
    else
        y_ref_minjerk(idx_takeoff+1 : idx_rec_end) = pos_rec;
    end
end

%% パラメータ最適化
fprintf('Hdの設計(最適化)を実行中...\n');
Kd = target_rad / weight_torque;
rho0 = [1.5, 20.0]; % 初期値 [ζ, ωn]
lb = [1.0001, 0.1]; 
ub = [10.0, 200.0];

opt = optimoptions('fmincon','Algorithm','sqp','Display','none');
f = @(x) J_design(x, Kd, y_ref_minjerk, u_weight, t);
tic;
[rho_opt, fval, exitflag, output] = fmincon(f, rho0, [], [], [], [], lb, ub, [], opt);
calc_time = toc;
zeta_opt = rho_opt(1);
omega_opt = rho_opt(2);
K_opt = Kd;

fprintf('--------------------------\n');
fprintf('最適化計算時間: %.4f 秒\n', calc_time);
fprintf('反復回数 (Iterations): %d 回\n', output.iterations);
fprintf('関数評価回数 (Func Count): %d 回\n', output.funcCount);
fprintf('--------------------------\n');
%% 結果表示
num_C = K_opt * omega_opt^2;
den_C = [1, 2*zeta_opt*omega_opt, omega_opt^2];
Cd_opt = tf(num_C, den_C);
Cd = Cd_opt*G;
y_sim = lsim(Cd, u_weight, t);
C_ini = K_opt*rho0(2)/(s^2+2*rho0(1)*rho0(2)*s+(rho0(2))^2);
y_ini = C_ini*G;
C = lsim(y_ini, u_weight, t);

fprintf('\n=== [Step 1] Hd 設計結果 (T_cycle = %.1fs) ===\n', T_cycle);
fprintf('theta = %.4f\n', zeta_opt);
fprintf('omega_n =  %.4f\n', omega_opt);
fprintf('K_d = %.6f\n', K_opt);
fprintf('--------------------------\n');

figure('Name', 'Step1: Hd Design');
plot(t, y_ref_minjerk*180/pi, '--k', 'LineWidth', 2.0); hold on;
plot(t, y_sim*180/pi, '-r', 'LineWidth', 2.0);
plot(t, C*180/pi, '-k', 'LineWidth', 2.0);
ylabel('angle [deg]'); xlabel('time [s]');title(['設計: MinJerk vs Hd応答 (T_{cycle}=', num2str(T_cycle), 's)']);
legend('目標軌道 ', '理想制御器','初期パラメータ');
grid on; 
xlim([0.8, t_end]); 
ylim([0, 20]);

%% 最小ジャーク
function err = J_design(rho, K_val, y_ref, u, t)
    z = rho(1); w = rho(2); k = K_val;
    try
        sys = tf(k*w^2, [1, 2*z*w, w^2]);
        y = lsim(sys, u, t);
        err = sum((y - y_ref).^2);
    catch
        err = inf;
    end
end

function [pos, vel] = generate_minjerk(p_start, p_end, T, t)
    tau = t / T; tau(tau<0)=0; tau(tau>1)=1;
    poly_pos = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;
    pos = p_start + (p_end - p_start) * poly_pos;
    vel = 0; % 速度の軌道も欲しい場合は右の式に書き換える　(p_end - p_start) * poly_vel / T 
end
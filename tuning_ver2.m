%% FRITによるコンプライアンスパラメータ (I, D, K) のチューニング
clearvars;
close all;
clc;

%% 設計
Ts = 0.001;
s = tf('s');

% 制御対象（サーボモータ） G(s)
num_g = 1064;
den_g = [1, 56.9542, 1043.1];
G_s = tf(num_g, den_g);
G_z = c2d(G_s, Ts, 'tustin'); % シミュレーション用

% Step1で得られた理想パラメータ (例)
zeta = 1.0001;
omega_n = 10.3409;
K_ideal_gain = 0.607423; 

% 理想制御器 Hd (ボード線図比較用)
num_Hd = K_ideal_gain * omega_n^2;
den_Hd = [1, 2*zeta*omega_n, omega_n^2];
Hd_s = tf(num_Hd, den_Hd);

% 目標とする全体システム Td = Hd * G (時間応答計算用)
Td_s = Hd_s * G_s;
Td_z = c2d(Td_s, Ts, 'tustin');

% 制御器構造 C(rho) = 1 / (I*s^2 + D*s + K)
Crho_s = @(rho) tf(1, [rho(1), rho(2), rho(3)]);

%% データ準備
disp('データを準備中...');
t_end = 8.0;
t = (0:Ts:t_end)';

% 入力信号
u0 = zeros(size(t));
idx_land = round(1.0/Ts)+1;
idx_takeoff = round(1.48/Ts)+1;
u0(idx_land : idx_takeoff) = 0.54;

% 目標軌道 y_target (理想制御器 Hd * G を通した応答)
y_target = lsim(Td_z, u0, t);

% 初期値 rho0 の設定 [I, D, K]
rho0 = [0.01, 0.5, 1.0]; 

%% FRIT最適化
% 評価関数 (最適化には G を含める必要があります)
f = @(rho) J(rho, u0, y_target, Ts, G_s);

% 計算オプション
opt = optimoptions('fmincon','Algorithm','sqp','Display','iter');

% 制約: I, D, K > 0
lb = [1e-6, 1e-4, 1e-4]; 
ub = [1.0,  100.0, 1000.0];

fprintf('\nFRIT最適化を開始...\n');
tic;
try
    [rho_opt, fval, exitflag, output] = fmincon(f, rho0, [], [], [], [], lb, ub, [], opt);
catch ME
    warning('エラー発生: %s', ME.message);
    rho_opt = rho0;
end
calc_time = toc;

fprintf('--------------------------\n');
fprintf('最適化計算時間: %.4f 秒\n', calc_time);
fprintf('反復回数: %d 回\n', output.iterations);
fprintf('--------------------------\n');

fprintf('\n=== [Step 2] チューニング結果 ===\n');
fprintf('I = %.6f\n', rho_opt(1));
fprintf('D = %.6f\n', rho_opt(2));
fprintf('K = %.6f\n', rho_opt(3));

%% 検証
% 最適化された制御器 C_opt
C_opt_s = Crho_s(rho_opt);
C_opt_z = c2d(C_opt_s, Ts, 'tustin');

% 時間応答確認用全体システム (C * G)
Sys_final_z = C_opt_z * G_z;
y_final = lsim(Sys_final_z, u0, t);

% --- 時間応答のプロット ---
figure('Name', 'Step2 Result: Time Response');
plot(t, y_target*180/pi, '--k', 'LineWidth', 2.0); hold on;
plot(t, y_final*180/pi, '-r', 'LineWidth', 2.0);
ylabel('Angle [deg]'); xlabel('Time [s]');
title('【結果】時間応答 (全体システム出力)');
legend('Target (Hd \times G)', 'Tuned (C \times G)');
grid on; xlim([0, 8.0]);

% --- ボード線図のプロット（ここを制御器のみに変更） ---
figure('Name', 'Step2 Result: Bode Plot (Controllers Only)');
opts = bodeoptions; 
opts.FreqUnits = 'Hz'; 
opts.Grid = 'on'; 
opts.PhaseVisible = 'off';

% Hd_s (理想制御器) と C_opt_s (最適化後制御器) を比較
bodeplot(Hd_s, 'k--', C_opt_s, 'r', opts);

title('ボード線図 (制御器単体の比較)');
legend('Ideal Controller (Hd)', 'Tuned Controller (C)');

%% サブ関数: コスト関数
function val = J(rho, u, y_target, Ts, G_s)
    try
        I = rho(1); D = rho(2); K = rho(3);
        if I < 1e-8 || D < 1e-8 || K < 1e-8; val = inf; return; end
        
        % 現在の制御器 C
        C_s = tf(1, [I, D, K]);
        
        % シミュレーションは「制御器 C × 制御対象 G」で行う
        Sys_s = C_s * G_s;
        Sys_z = c2d(Sys_s, Ts, 'tustin'); 
        
        y_sim = lsim(Sys_z, u);
        
        len = min(length(y_sim), length(y_target));
        e = y_sim(1:len) - y_target(1:len);
        val = sum(e.^2);
    catch
        val = inf;
    end
end
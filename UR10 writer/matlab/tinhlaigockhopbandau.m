%Diem goc cho dong hoc nguoc
xx_0 = -1.0024;
yy_0 = -0.2653;
zz_0 = 0.38823;
X_0 = [-yy_0; xx_0; zz_0]; % Vector vi tri E
% Gia tri gan dung cua cac goc khop ban dau
q1_0 = 1.6708;
q2_0 = 0.62;
q3_0 = 1.208;
q4_0 = -0.25;
q5_0 = -1.5708;
q6_0 = 0;

for n = 1: 1: 10^5
    Jnd_0 = computeJnd(q1_0, q2_0, q3_0, q4_0, q5_0, q6_0);
    [xE_0, yE_0, zE_0] = UR10_forwardKinematic(q1_0, q2_0, q3_0, q4_0, q5_0, q6_0);% tinh lai xx_0, yy_0 theo q_0
    XX_0 = [xE_0; yE_0; zE_0];
    delta_q_0 = Jnd_0*(X_0 - XX_0);% Tinh gia tri hieu chinh delta_q_0
    % Tinh lai cac gia tri q_0 hieu chinh
    q1_0 = q1_0 + delta_q_0(1, 1);
    q2_0 = q2_0 + delta_q_0(2, 1);
    q3_0 = q3_0 + delta_q_0(3, 1);
    q4_0 = q4_0 + delta_q_0(4, 1);
    q5_0 = q5_0 + delta_q_0(5, 1);
    q5_0 = q5_0 + delta_q_0(6, 1);
    % Khai bao do chinh xac can thiet va ta vong lap tinh toan
    ss = 10^(-10);
    if abs(delta_q_0(1, 1)) < ss
        if abs(delta_q_0(2, 1)) < ss
            if abs(delta_q_0(3, 1)) < ss
                if abs(delta_q_0(4, 1)) < ss
                    if abs(delta_q_0(5, 1)) < ss
                        if abs(delta_q_0(6, 1)) < ss
                            break
                        end
                    end
                end
            end
        end
    end
    n;
end
% Xac nhan cac gia tri q_0 chinh xac sau khi hieu chinh
q1 = q1_0;
q2 = q2_0;
q3 = q3_0;
q4 = q4_0;
q5 = q5_0;
q6 = q6_0;

q = [q1 q2 q3 q4 q5 q6]
t = 0:0.1:4;
%%
figure("Name","Đồ thị quỹ đạo các khớp từ A tới B");
for i = 1:6
    hold on;
    plot(t,q(i,:),"LineWidth",1.5);
end
ylim([-3 3]);
legend('Khớp 1','Khớp 2','Khớp 3','Khớp 4','Khớp 5','Khớp 6');
title("Đồ thị quỹ đạo các khớp từ A tới B")
xlabel('time(s)'); ylabel('rad');
%%
figure("Name","Đồ thị vận tốc các khớp từ A tới B");
for i = 1:6
    hold on;
    plot(t,qd(i,:),"LineWidth",1.5);
end
ylim([-1 1.5]);
legend('Khớp 1','Khớp 2','Khớp 3','Khớp 4','Khớp 5','Khớp 6');
title("Đồ thị vận tốc các khớp từ A tới B")
xlabel('time(s)'); ylabel('rad/s');
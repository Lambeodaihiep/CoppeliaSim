
% t = 0:16/162:16;
t = trajTimes;
%% Tròn
for i = 1:6
    figure("Name","Đồ thị vị trí khớp thứ " + string(i) + " khi gắp vật hình tròn");
    plot(t,q_circle(i,:),"LineWidth",2);
    hold on;
    plot(t,q_circle_real(i,2:end),"LineWidth",1.5);
    legend('Quỹ đạo mong muốn','Quỹ đạo thực tế');
    title('Đồ thị vị trí khớp thứ ' + string(i) + ' khi gắp vật hình tròn')
    xlabel('time(s)'); ylabel('rad');
end
%% Sao
for i = 1:6
    figure("Name","Đồ thị vị trí khớp thứ " + string(i) + " khi gắp vật hình ngôi sao");
    plot(t,q_star(i,:),"LineWidth",2);
    hold on;
    plot(t,q_star_real(i,2:end),"LineWidth",1.5);
    legend('Quỹ đạo mong muốn','Quỹ đạo thực tế');
    title('Đồ thị vị trí khớp thứ ' + string(i) + ' khi gắp vật hình ngôi sao')
    xlabel('time(s)'); ylabel('rad');
end
%% Tim
for i = 1:6
    figure("Name","Đồ thị vị trí khớp thứ " + string(i) + " khi gắp vật hình trái tim");
    plot(t,q_heart(i,:),"LineWidth",2);
    hold on;
    plot(t,q_heart_real(i,2:end),"LineWidth",1.5);
    legend('Quỹ đạo mong muốn','Quỹ đạo thực tế');
    title('Đồ thị vị trí khớp thứ ' + string(i) + ' khi gắp vật hình trái tim')
    xlabel('time(s)'); ylabel('rad');
end
%% Tam giác
for i = 1:6
    figure("Name","Đồ thị vị trí khớp thứ " + string(i) + " khi gắp vật hình tam giác");
    plot(t,q_triangle(i,:),"LineWidth",2);
    hold on;
    plot(t,q_triangle_real(i,2:end),"LineWidth",1.5);
    legend('Quỹ đạo mong muốn','Quỹ đạo thực tế');
    title('Đồ thị vị trí khớp thứ ' + string(i) + ' khi gắp vật hình tam giác')
    xlabel('time(s)'); ylabel('rad');
end
%% Vuông
for i = 1:6
    figure("Name","Đồ thị vị trí khớp thứ " + string(i) + " khi gắp vật hình vuông");
    plot(t,q_square(i,:),"LineWidth",2);
    hold on;
    plot(t,q_square_real(i,2:end),"LineWidth",1.5);
    legend('Quỹ đạo mong muốn','Quỹ đạo thực tế');
    title('Đồ thị vị trí khớp thứ ' + string(i) + ' khi gắp vật hình vuông')
    xlabel('time(s)'); ylabel('rad');
end
%% ALL
t_all = 0:80/814:80;
for i = 1:6
    figure("Name","Đồ thị vị trí khớp thứ " + string(i) + " của tất cả");
    plot(t_all,[q_circle(i,:) q_star(i,:) q_heart(i,:) q_triangle(i,:) q_square(i,:)],"LineWidth",2);
    hold on;
    plot(t_all,[q_circle_real(i,2:end) q_star_real(i,2:end) q_heart_real(i,2:end) q_triangle_real(i,2:end) q_square_real(i,2:end)],"LineWidth",1.5);
    legend('Quỹ đạo mong muốn','Quỹ đạo thực tế');
    title('Đồ thị vị trí khớp thứ ' + string(i) + ' của tất cả')
    xlabel('time(s)'); ylabel('rad');
end
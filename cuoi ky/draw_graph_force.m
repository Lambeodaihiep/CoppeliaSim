%% Tròn
t = 0:16/(length(q_force_circle)-2):16;
for i = 1:6
    figure("Name","Đồ thị momen lực khớp thứ " + string(i) + " khi gắp vật hình tròn");
    plot(t,q_force_circle(i,2:end),"LineWidth",1.5);
    title('Đồ thị momen lực khớp thứ ' + string(i) + ' khi gắp vật hình tròn')
    xlabel('time(s)'); ylabel('N/m');
end
%% Sao
t = 0:16/(length(q_force_star)-2):16;
for i = 1:6
    figure("Name","Đồ thị momen lực khớp thứ " + string(i) + " khi gắp vật hình ngôi sao");
    plot(t,q_force_star(i,2:end),"LineWidth",1.5);
    title('Đồ thị momen lực khớp thứ ' + string(i) + ' khi gắp vật hình ngôi sao')
    xlabel('time(s)'); ylabel('N/m');
end
%% Tim
t = 0:16/(length(q_force_heart)-2):16;
for i = 1:6
    figure("Name","Đồ thị momen lực khớp thứ " + string(i) + " khi gắp vật hình trái tim");
    plot(t,q_force_heart(i,2:end),"LineWidth",1.5);
    title('Đồ thị momen lực khớp thứ ' + string(i) + ' khi gắp vật hình trái tim')
    xlabel('time(s)'); ylabel('N/m');
end
%% Tam giác
t = 0:16/(length(q_force_triangle)-2):16;
for i = 1:6
    figure("Name","Đồ thị momen lực khớp thứ " + string(i) + " khi gắp vật hình tam giác");
    plot(t,q_force_triangle(i,2:end),"LineWidth",1.5);
    title('Đồ thị momen lực khớp thứ ' + string(i) + ' khi gắp vật hình trái tam giác')
    xlabel('time(s)'); ylabel('N/m');
end
%% Vuông
t = 0:16/(length(q_force_square)-2):16;
for i = 1:6
    figure("Name","Đồ thị momen lực khớp thứ " + string(i) + " khi gắp vật hình vuông");
    plot(t,q_force_square(i,2:end),"LineWidth",1.5);
    title('Đồ thị momen lực khớp thứ ' + string(i) + ' khi gắp vật hình trái vuông')
    xlabel('time(s)'); ylabel('N/m');
end
%% ALL
q_force_all = [q_force_circle(:,2:end) q_force_star(:,2:end) q_force_heart(:,2:end) q_force_triangle(:,2:end) q_force_square(:,2:end)];
t_all = 0:80/(length(q_force_all)-1):80;
for i = 1:6
    figure("Name","Đồ thị momen lực khớp thứ " + string(i) + " của tất cả");
    plot(t_all,q_force_all(i,:),"LineWidth",1.5);
    title('Đồ thị momen lực khớp thứ ' + string(i) + ' của tất cả');
    xlabel('time(s)'); ylabel('N/m');
end
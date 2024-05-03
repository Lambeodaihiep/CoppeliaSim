%% Get template
img_input = imread("star.jpg");
GRAY_input = rgb2gray(img_input);
threshold = graythresh(GRAY_input);
BW_input = imbinarize(GRAY_input, threshold);
st1 = regionprops(~BW_input, 'BoundingBox','Area', 'Extent');
imshow(img_input); hold on;

for k = 1 : length(st1)
    if (st1(k).Area < 10000 && st1(k).Area > 300)
        BB = st1(k).BoundingBox;
        y = round(BB(1)); x = round(BB(2)); h = round(BB(3)); w = round(BB(4));
        rectangle('Position', BB,'EdgeColor','r','LineWidth',2 )
        template = imcrop(~BW_input, BB);
    end
end
Size = size(template);
figure;
imshow(template); hold on;
%%
[B,L] = bwboundaries(template, 'noholes');
boundary = B{1};
plot(boundary(:,2), boundary(:,1), 'g', 'LineWidth', 2)
distance = {};
center = [Size(1)/2, Size(2)/2];
max = 0;
min = 1e4;
for i = 1:length(boundary)
    a = sqrt((center(1)-boundary(i,1))^2 + (center(2)-boundary(i,2))^2 );
    if(min > a) min = a; end
    if(max < a)
        max = a;
    end
end
template_ratio = min/max;
%% Lấy các hình dạng có trên hộp và so sánh với template
img = imread("box.jpg");
GRAY = rgb2gray(img);
threshold = graythresh(GRAY);
BW = imbinarize(GRAY, threshold);
% imshow(~BW); hold on;
crop = {};
st = regionprops(~BW, 'BoundingBox','Area','Extent');
for k = 1 : length(st)
    if (st(k).Area < 10000 && st(k).Area > 2000)
        BB = st(k).BoundingBox;
        y = round(BB(1)); x = round(BB(2)); h = round(BB(3)); w = round(BB(4));
%         rectangle('Position', BB,'EdgeColor','r','LineWidth',2 )
%         crop{1, end+1} = imresize(imcrop(~BW, BB), Size);
        crop{1, end+1} = imcrop(~BW, BB);
        crop{2, end} = BB; % Lưu lại tọa độ cẳt ảnh
%         figure;
%         imshow(crop{1,end}); hold on;
        % Tính tỷ lệ cạnh ngắn nhất và dài nhất
        Size = size(crop{1, end});
        [B,L] = bwboundaries(crop{1, end}, 'noholes');
        boundary = B{1};
%         plot(boundary(:,2), boundary(:,1), 'g', 'LineWidth', 2)
        center = [Size(1)/2, Size(2)/2];
        max = 0;
        min = 1e4;
        for i = 1:length(boundary)
            a = sqrt((center(1)-boundary(i,1))^2 + (center(2)-boundary(i,2))^2 );
            if(min > a) min = a; end
            if(max < a)
                max = a;
            end
        end
        crop{3, end} = min/max;
    end
end

min = 1e4;
for i = 1:length(crop)
    if (min > abs(template_ratio - crop{3,i}))
        min = abs(template_ratio - crop{3,i});
        res = i;
    end
end

figure;
imshow(crop{1,res});
%% Tìm góc xoay giữa 2 ảnh
max = 0;
for i = 0:120
    BW_rotated = BW_input;
    BW_rotated = imrotate(BW_rotated, i);
    st1 = regionprops(~BW_rotated, 'BoundingBox','Area', 'Extent');
    
    for k = 1 : length(st1)
        BB = st1(k).BoundingBox;
        a = BB(3) / BB(4);
        if (st1(k).Area < 10000 && st1(k).Area > 300 && a>0.75 && a<1.25)
            rectangle('Position', BB,'EdgeColor','r','LineWidth',2 )
            template = imcrop(~BW_rotated, BB);
        end
    end
%     imshow(template);
    Size = size(template);
    temp = imresize(crop{1,res}, Size);
    overlap = template & temp;
    if (max < sum(overlap(:) == 1))
        max = sum(overlap(:) == 1);
        angle = i;
    end
end
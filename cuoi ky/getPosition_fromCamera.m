%     sim=remApi('remoteApi');
%     sim.simxFinish(-1);
%     clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);
%     
%     if clientID < 0
%         disp("Fail to connect to Coppelia");
%         sim.delete();
%         return;
%     else
%         disp('Connected to remote API server');
%     end
    
    %% Get handles
    [~, cam_handle] = sim.simxGetObjectHandle(clientID, 'Vision_sensor', sim.simx_opmode_oneshot_wait);
    [~, base_handle] = sim.simxGetObjectHandle(clientID, 'UR10', sim.simx_opmode_oneshot_wait);
    [~, goalPose] = sim.simxGetObjectHandle(clientID, 'goalPose', sim.simx_opmode_oneshot_wait);
    
    %% Get fixed positions
    [~, cam2base] = sim.simxGetObjectPosition(clientID, cam_handle, base_handle, sim.simx_opmode_oneshot_wait);
    
    %% Camera configuration
    height_of_conveyor = 0.15805 + 0.04;
    field_of_view = 2 * tan(pi/6) * (0.5 - height_of_conveyor);
    %% Get image
    % while (true)
        [~, resolution, img] = sim.simxGetVisionSensorImage2(clientID, cam_handle, 0, sim.simx_opmode_oneshot_wait);
        
        GRAY = rgb2gray(img);
        threshold = graythresh(GRAY);
        BW = imbinarize(GRAY, 0.9);
        
        imshow(img); hold on;
        
        st = regionprops(~BW, 'BoundingBox','Area');
        for k = 1 : length(st)
            if (st(k).Area < 10000 && st(k).Area > 2000)
                BB = st(k).BoundingBox;
                y = round(BB(1)); x = round(BB(2)); h = round(BB(3)); w = round(BB(4));
                rectangle('Position', [y, x, h, w], 'EdgeColor','r','LineWidth',2 )
%                 crop = img(x:x+w, y:y+h);
    %             figure;
    %             imshow(crop);
            end
        end
        % Toạ độ tâm vật thể
        centerX = x + w/2;
        centerY = y + h/2;
        plot(centerY, centerX, 'ro', 'MarkerSize', 10);
        % Tọa độ tâm vật thể so với gốc
        size = double(resolution(1));
        real_posX = (size/2 - centerY) * field_of_view / size + cam2base(1);
        real_posY = (centerX - size/2) * field_of_view / size + cam2base(2);
        real_posZ = height_of_conveyor;
        [~] = sim.simxSetObjectPosition(clientID, goalPose, -1, [real_posX, real_posY, real_posZ+0.01], sim.simx_opmode_oneshot_wait);
        targetpoint = double([real_posX real_posY real_posZ-height_of_robot_base]);
        clf;
    % end


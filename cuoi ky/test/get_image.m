    sim=remApi('remoteApi');
    sim.simxFinish(-1);
    clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);
    
    if clientID < 0
        disp("Fail to connect to Coppelia");
        sim.delete();
        return;
    else
        disp('Connected to remote API server');
    end
    
    %% Get handles
    [~, cam_handle] = sim.simxGetObjectHandle(clientID, '/box/Vision_sensor', sim.simx_opmode_oneshot_wait);
    %% Get image
    while (true)
        [~, resolution, img] = sim.simxGetVisionSensorImage2(clientID, cam_handle, 0, sim.simx_opmode_oneshot_wait);
        imshow(img); hold on;
%         GRAY = rgb2gray(img);
%         threshold = graythresh(GRAY);
%         BW = imbinarize(GRAY, threshold);
%         st1 = regionprops(~BW, 'BoundingBox','Area');
%         
%         for k = 1 : length(st1)
%             if (st1(k).Area < 10000 && st1(k).Area > 300)
%                 BB = st1(k).BoundingBox;
%                 rectangle('Position', BB,'EdgeColor','r','LineWidth',2 )
%             end
%         end
        pause(0.2);
    end


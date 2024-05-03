function UR10_ = load_robot()

    rotationX = @(t) [1 0 0; 0 cosd(t) -sind(t); 0 sind(t) cosd(t)] ;
    rotationY = @(t) [cosd(t) 0 sind(t); 0 1 0; -sind(t) 0 cosd(t)] ;
    rotationZ = @(t) [cosd(t) -sind(t) 0; sind(t) cosd(t) 0; 0 0 1] ;
    
    UR10_ = loadrobot("universalUR10");
    % gripper = loadrobot("robotiq2F85");
    % UR10_.addSubtree('tool0', gripper, ReplaceBase=false);
    
    glueDispenserBody = rigidBody('gripper');
    tForm = [eye(3).*0.001,[-0.027 0 -0.0545]';[0 0 0 1]]; % scale units of stl from mm to m 
    addVisual(glueDispenserBody,"Mesh",'gripper.stl', tForm);
    glueDispenserBodyJoint = rigidBodyJoint('glueDispenserBodyJoint','fixed');
    glueDispenserBody.Joint = glueDispenserBodyJoint;
    transfForglueDispenserBody = rotm2tform(rotationZ(-90)*rotationY(90));
    setFixedTransform(glueDispenserBody.Joint, transfForglueDispenserBody)
    curEndEffectorBodyName = UR10_.BodyNames{9};
    addBody(UR10_,glueDispenserBody,curEndEffectorBodyName);
    
    transfForNewEndEffectorBody = rotm2tform(rotationZ(-90)*rotationX(-90));
%     transfForNewEndEffectorBody = rotm2tform(rotationZ(90)*rotationX(180));
    transfForNewEndEffectorBody(:,4) = [0.19000; 0; 0; 1];
    newEndEffectorBody = rigidBody('mid_point');
    setFixedTransform(newEndEffectorBody.Joint, transfForNewEndEffectorBody);
    glueDispenserBodyName = UR10_.BodyNames{9};
    addBody(UR10_,newEndEffectorBody,glueDispenserBodyName);
    UR10_.DataFormat = 'row';
    
    
    UR10_.show

    curDir = pwd;
    saveDir = fileparts(mfilename('fullpath'));
    cd(saveDir)
    save UR10_ UR10_
    cd(curDir)
%     axis([-1 1.5 -1 1 -1 1])
end
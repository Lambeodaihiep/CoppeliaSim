a = loadrobot('kinovaGen3');
a.DataFormat = 'row';

rotationX = @(t) [1 0 0; 0 cosd(t) -sind(t) ; 0 sind(t) cosd(t)] ;

glueDispenserBody = rigidBody('dispenser');
addVisual(glueDispenserBody,"Mesh",'cylinder.STL')
glueDispenserBodyJoint = rigidBodyJoint('glueDispenserBodyJoint','fixed');
glueDispenserBody.Joint = glueDispenserBodyJoint;
transfForglueDispenserBody = rotm2tform(rotationX(0));
setFixedTransform(glueDispenserBody.Joint, transfForglueDispenserBody)
curEndEffectorBodyName = a.BodyNames{8};
addBody(a,glueDispenserBody,curEndEffectorBodyName);

% transfForNewEndEffectorBody = rotm2tform(rotationX(180));
% transfForNewEndEffectorBody(:,4) = [0.04; -0.195; 0; 1];
% newEndEffectorBody = rigidBody('dispenserEdge');
% setFixedTransform(newEndEffectorBody.Joint, transfForNewEndEffectorBody);
% glueDispenserBodyName = a.BodyNames{9};
% addBody(a,newEndEffectorBody,glueDispenserBodyName);

% Close the previous figure window before running the script again
close(findobj('type','figure','name','Interactive Visualization'));

% Visualize the interactive rigid body tree at the home position ('q_home')
a = interactiveRigidBodyTree(a);
q_home = [0 0 0 0 0 0 0]'*pi/180;
rotate3d off;
view(145,25)
lightangle(20,-160)
axis([-1 1 -1 1 -0.5 1])
hold on
zlim([-0.5 1.5])
a.ShowMarker = false;
a.Configuration = q_home;



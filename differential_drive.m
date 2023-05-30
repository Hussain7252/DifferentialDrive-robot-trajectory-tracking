% Read the Image and Convert to Binary Occupancy Map 
image = imread("occupancy_map.png");
figure();
imshow(image);
MapX=size(image,2);
MapY=size(image,1);
resolution = 1; % 1 pixels = 1meter
%{Please be careful binaryOccupancyMap dimensions are flipped to that of the image%}
map = binaryOccupancyMap(MapX,MapY,resolution);
map1 = binaryOccupancyMap(MapX,MapY,resolution);
%Also be careful Image Origin is at the top left while binaryOccupancyMap origin is at the bottom left
% to set the binaryOccupancyMap correctly 
for x = 1:MapX
    for y = 1:MapY
        if image(y,x) == 255
            setOccupancy(map,[x,MapY-y+1],0); %Free
            setOccupancy(map1,[x,MapY-y+1],0);
        else
            setOccupancy(map,[x,MapY-y+1],1);%Occupied
            setOccupancy(map1,[x,MapY-y+1],1);
        end
    end
end
figure();
show(map);

% Method to check if the Map creation is correct or not
estMap = occupancyMatrix(map);
for x = 1: size(estMap,1)
    flag=0;
    for y= 1:size(estMap,2)
        if (image(x,y)==255 && estMap(x,y)==0)
            continue
        elseif (image(x,y)==0 && estMap(x,y)==1)
            continue
        else
            flag=1;
            break;
    if (flag==1)
        break;
    end
        end
    end
end

%Robot Behaviour
robot = differentialDriveKinematics("TrackWidth",8,"VehicleInputs","VehicleSpeedHeadingRate");

%Inflate the boundaries to avoid collision

inflate(map,robot.TrackWidth/2);


%Global Planner (Hybrid A*)
ss = stateSpaceSE2; 
validator= validatorOccupancyMap(ss);
validator.Map = map;
validator.ValidationDistance = 0.1;
ss.StateBounds = [map.XWorldLimits;map.YLocalLimits;[-pi pi]];
planner = plannerHybridAStar(validator,'MinTurningRadius',4);
start=[319 172 0];
goal =[90 489 pi/4];
route=plan(planner,start,goal);
figure();
show(planner)

%Local Planner (Pure Pursuit)
controller = controllerPurePursuit;
controller.Waypoints = route.States(:,1:2);
controller.DesiredLinearVelocity = 10;
controller.LookaheadDistance = 0.7;
controller.MaxAngularVelocity = 2;

%Costmap creation
costmap = vehicleCostmap(map1);

%Implementing
robotInitialLocation = route.States(1,1:2);
robotgoal = route.States(end,1:2);
goalRadius = 5;
distancetoGoal = norm(robotInitialLocation - robotgoal);
robotposition = (start)';
sampleTime=0.05;
vizRate=rateControl(1/sampleTime);
figure
frameSize = robot.TrackWidth/0.8;
while(distancetoGoal > goalRadius)
    %Compute local controller outputs
    [v,omega] = controller(robotposition(1:3));

    %Get Robots velocity using controller inputs
    vel = derivative(robot,robotposition,[v omega]);

    %Update the current pose
    robotposition = robotposition + vel*sampleTime;

    %Compute distance to the goal
    distancetoGoal = norm(robotposition(1:2) - robotgoal(:));

    %update plot
    hold off
    plot(costmap);
    hold all

    %plot path each instance so that it stays persistent while robot moves
    plot(route.States(:,1),route.States(:,2),"k--d")
   
    %plot the path of the robot as a set of transforms
    plotTrVec = [robotposition(1:2);0];
    plotRot = axang2quat([0 0 1 robotposition(3)]);
    plotTransforms(plotTrVec',plotRot,'MeshFilePath','groundvehicle.stl','Parent',gca,'View','2D',"FrameSize",frameSize);
    light;
    xlim([0 623])
    ylim([0 680])
    waitfor(vizRate);
end
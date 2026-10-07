%% Road scenario

nlanes=2;
rr=create_road_1([Xr Yr],nlanes);

scenario = drivingScenario;

% Road
roadCenters = [rr.lb{4}, zeros(length(rr.lb{2}),1)];
ls1=lanespec(nlanes,'Width',4);
r1=road(scenario,roadCenters,'Lanes',ls1);

% EGO vehicle
ego=vehicle(scenario,'ClassID',1);

% Obstacle
if rp==3
    obst1=actor(scenario,'Position',[Co;0],'Yaw',-10,'Length',1.5,'Width',1.5);
end

%% Bus definition for Scenario Reader

busInfo1=Simulink.Bus.createObject( ...
    struct('ActorID', uint32(1), ...
        'Position', zeros(1,3), ...
        'Velocity', zeros(1,3), ...
        'Roll', 0, ...
        'Pitch', 0, ...
        'Yaw', 0, ...
        'AngularVelocity', zeros(1,3)));
ActorPose=evalin('base', busInfo1.busName);
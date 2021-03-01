%uses peripherals2.py,
%jointcontroller_host_Loacl_Compliance_addRawPositionMode.py
%% initial setup
clear; clc;
warning off; %used to prevent unnecessary displays to screen because this takes a lot of time
twhole = tic; %used to see how long the entire script execution takes
addpath('/home/cats/Documents/oakesk/02_10_2021_rawPositionMode/schroed/longRawMove')
%% things that may be modified
numLoops = 1; %number of times to loop through the joints
jointDes = [1]; %joints to be tested
armPose = 4;
interPoseNum = 3;
arm = 'left';
fmaxdes = 10; %maximum frequency of interest (frequency at which data is no longer useful - mostly noise)
amp = 2; %amplitude of sinusoid in degrees
Ns = 10000; %number of sines
loadStatus = 'weight'; %noGrip, grip, weight


pause(10);
%%
qbias = [-0.85;18.1;-1.7;11.9;-0.15;6.9;-0.3]*pi/180;    

%% load poses joint angles
load('schroedPoses.mat') %all joint positions are in degrees
interPosesChoice = [zeros(7,1),[-55;-45;0;90;0;-45;0],[0;-35;zeros(5,1)]]*pi/180;
inds = 1:7;
if strcmp(arm,'right')
    qposes = [-1;1;-1;1;-1;1;-1].*qposes;
    qposesweight = [-1;1;-1;1;-1;1;-1].*qposesweight;
    interPosesChoice = [-1;1;-1;1;-1;1;-1].*interPosesChoice;
    inds = 8:14;
end
interPose = interPosesChoice(:,interPoseNum);

%% variables for generating the schroeder phase signal
usat = amp*pi/180;
T = 0.02; %sample time (same as the control loop)
fvec = (1/T)*(1:Ns/2)/Ns; %vector of corresponding frequencies (Hz)
%zero out the frequency components above the max frequency of interest
idx = find(fvec>=fmaxdes,1,'first');
relmag = [ones(1,idx),zeros(1,length(1:Ns/2)-idx)];
%generate the schroeder phase signal
[us,ts,mags,phs] = schroed(T,Ns,relmag,usat);
us = us-us(1); %this ensures the initial value is zero (so first commanded position is the initial position)
%us = flip(us);
%% additional setup
%disable peripherals (1 to disable, 0 to have enabled)
disCollAvoid = 1; %collision avoidance
disConSafe = 1; %contact safety
disBodyAvoid = 1; %body avoidance
disGravComp = 1; %gravity compensation

[robot_const,~] = defineBaxter();
%sound effect for when done
seff = load('gong.mat');

%connect to baxter
robot = RobotRaconteur.Connect('tcp://localhost:4545/BaxterJointServer/Baxter');
%connect to peripherals
peripherals = RobotRaconteur.Connect('tcp://localhost:46604/BaxterPeripheralServer/BaxterPeripherals');
%set baxter to raw position control mode
robot.setControlMode(uint8(3));
%disable/enable desired peripherals
peripherals.suppressBodyAvoidance(arm,uint8(disBodyAvoid));
peripherals.suppressCollisionAvoidance(arm,uint8(disCollAvoid));
peripherals.suppressContactSafety(arm,uint8(disConSafe));
peripherals.suppressGravityCompensation(arm,uint8(disGravComp));
baxGrip = RobotRaconteur.Connect('tcp://192.168.1.134:6006/GripperController/gripcon');

pause(5);
%determine joint upper and lower bounds to check that desired amplitude
%won't go beyond the joint limits
if strcmp(arm,'left')
    ub = robot_const(1).limit.upper_joint_limit;
    lb = robot_const(1).limit.lower_joint_limit;
elseif strcmp(arm,'right')
    ub = robot_const(2).limit.upper_joint_limit;
    lb = robot_const(2).limit.lower_joint_limit;
end


%% if with load, use buttons for grasping
tmp = peripherals.getNavigatorState(arm);
baxGrip.setPosition(uint8(0),uint8(100),uint8(1));
pause(5);
if strcmp(loadStatus,'weight')
    tmp = peripherals.getNavigatorState(arm);
    disp('waiting for ok button press to close gripper')
    while tmp.ok_button == 0
        tmp = peripherals.getNavigatorState(arm);
    end
    baxGrip.setPosition(uint8(255),uint8(1),uint8(1));
    disp('waiting for cancel button press to start movement')
    while tmp.cancel_button == 0
        tmp = peripherals.getNavigatorState(arm);
    end
end
pause(5);
tmp = peripherals.getNavigatorState(arm);

%% perform the data collection
for ii = numLoops %this specifies the number of times to loop through the data
    [~,~,~] = longMove(robot,interPose,100,0.02,arm,qbias);
    pause(5);
    
     pause(1);
        %make sure arm is at the desired initial pose
        if strcmp(loadStatus,'grip')
            qinit = qposes(:,armPose)*pi/180;
        elseif strcmp(loadStatus,'weight')
            qinit = qposesweight(:,armPose)*pi/180;
        else
            disp('wrong load status')
            return;
        end
        [~,~,~] = longMove(robot,qinit,100,0.02,arm,qbias);
        pause(5);
        Q0 = qinit;
        
    % loop through the desired joints
    for jointnum = jointDes
               
        %check to make sure signal won't violate joint constraints
        if Q0(jointnum)+amp*pi/180 > ub(jointnum)
            disp('amp too large');
            return;
        elseif Q0(jointnum)-amp*pi/180 < lb(jointnum)
            disp('amp too large');
            return;
        elseif Q0(jointnum)-amp*pi/180-qbias(jointnum) < lb(jointnum)
            disp('bias violates lower joint limit')
            return;
        elseif Q0(jointnum)+amp*pi/180-qbias(jointnum) > ub(jointnum)
            disp('bias violates upper joint limit')
            return;
        elseif Q0(jointnum)-amp*pi/180-qbias(jointnum) > ub(jointnum)
            disp('bias violates upper joint limit')
            return;
        elseif Q0(jointnum)+amp*pi/180-qbias(jointnum) < lb(jointnum)
            disp('bias violates lower joint limit')
            return;
        end
        
        pause(1)
        %get the starting joint positions for reference
        q0init = robot.joint_positions;
        
        %% set up other variables and figures
        t = zeros(1,20001);             %time vector
        q_desired=zeros(7,20000);       %desired joint positions
        q_baxter=zeros(14,20000);       %measured joint positions
        q_vel_baxter = zeros(14,20000); %measured joint velocities
        tor_baxter = zeros(14,20000);   %measured joint torques
        q_error = zeros(7,20000);       %computed joint error
        commQ = zeros(14,20000);
        commTor = zeros(14,20000);
        dt = 0.02;                      %control loop time
        q_c = zeros(7,20000);           %commanded joint position
        count = 0;                      %initialize count
        
        %%
        pause(4);
        disp('starting');
        pause(10);
        tmp = peripherals.getNavigatorState(arm);
        tstart = tic; %keep track of test length
        while t(count+1)<ts(end)
            tic; %keep track of control loop time
            %if ok button is pressed, stop execution
            if tmp.ok_button == 0
                count = count+1;
                %compute the desired joint positions
                q_temp = interp1(ts,us,t(count));
                q_desired(:,count) = Q0; %this means all other joints stay constant
                q_desired(jointnum,count) = q_desired(jointnum,count)+q_temp;
                
                % read data
                q_baxter(:,count) = robot.joint_positions;
                q_vel_baxter(:,count) = robot.joint_velocities;
                tor_baxter(:,count) = robot.joint_torques;
                commTor(:,count) = peripherals.commanded_torque;
                commQ(:,count) = peripherals.commanded_position;
                %compute joint error
                q_error(:,count) = q_desired(:,count)-q_baxter(inds,count);
                %compute command (add offset)
                q_c(:,count) = q_desired(:,count);
                %send command
                robot.setJointCommand(arm,q_c(:,count)-qbias);
                tmp = peripherals.getNavigatorState(arm);
                t(count+1) = toc(tstart);
            else
                
                break;
            end
            %set control loop to 20ms
            try
                java.lang.Thread.sleep((0.02-toc)*1000);
            catch ME
                pause(0.02-toc);
                %toc
            end
            
        end
        %% resend to initial pose
        disp(['Finished joint ',num2str(jointnum)])
        robot.setJointCommand(arm,Q0-qbias);
        pause(5);

        
        %% save data
        %trim off extra zeros
        idx = find(t>0,1,'last');
        t = t(2:idx);
        q_desired = q_desired(:,1:idx-1);
        q_baxter = q_baxter(:,1:idx-1);
        q_error = q_error(:,1:idx-1);
        q_vel_baxter = q_vel_baxter(:,1:idx-1);
        tor_baxter = tor_baxter(:,1:idx-1);
        q_c = q_c(:,1:idx-1);
        commTor = commTor(:,1:idx-1);
        commQ = commQ(:,1:idx-1);
        
        %save
        mkdir([pwd,'/',datestr(now,'dd-mmm-yyyy'),'/Arm',arm,'/Pose',num2str(armPose)])
        save([pwd,'/',datestr(now,'dd-mmm-yyyy'),'/Arm',arm,'/Pose',num2str(armPose),...
            '/baxter_schroederRawPosControl_arm',arm,'_jointNumber',num2str(jointnum),'_pose',num2str(armPose),'_',...
            'loadStatus',loadStatus,'_',datestr(now,'dd-mmm-yyyy-HH-MM'),'.mat'],...
            'q_desired','q_baxter','q_vel_baxter','t','tor_baxter','q_error','jointnum','Q0',...
            'T','Ns','relmag','ts','us','mags','phs','usat','fvec','q0init','fmaxdes',...
            'commTor','commQ','interPose','qbias','loadStatus','arm');
        
        pause(5)
    end
    sound(seff.y);
    
end

%play sound effect when done collecting data
toc(twhole)
robot.setControlMode(uint8(1));
%peripherals.suppressGravityCompensation('left',uint8(0));

%% open gripper if needed
if strcmp(loadStatus,'weight')
    disp('waiting for cancel button press to open gripper')
    while tmp.cancel_button == 0
        tmp = peripherals.getNavigatorState(arm);
    end
    baxGrip.setPosition(uint8(0),uint8(255),uint8(1));
end

rmpath('/home/cats/Documents/oakesk/02_10_2021_rawPositionMode/schroed/longRawMove/')


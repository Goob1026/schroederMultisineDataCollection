%uses peripherals2.py,
%jointcontroller_host_Loacl_Compliance_addRawPositionMode.py
%% initial setup
clear; clc;
warning off; %used to prevent unnecessary displays to screen because this takes a lot of time
twhole = tic; %used to see how long the entire script execution takes
pause(10);

%% things that may be modified
%path to the needed files
path1 = '/home/cats/Documents/oakesk/';
Fdes = logspace(-1,1,30);
%Fdes = 0.025;
Tdes = 1./Fdes;
loadStatus = 'weight'; %noGrip, grip, weight
jointnum = 6;
armPose = 6; 
interPoseNum = 1;
arm = 'left';



%% some params
N = 6;
amp = 2;
usat = amp*pi/180;


load('schroedPoses.mat')
interPosesChoice = [zeros(7,1),[-55;-45;0;90;0;-45;0],[0;-35;zeros(5,1)]]*pi/180;
interPose = interPosesChoice(:,interPoseNum);
if strcmp(arm,'right')
    interPose = [-1;1;-1;1;-1;1;-1].*interPose;
end
%% set up qbias
% if strcmp(loadStatus,'weight')
%     qbias = [0;9;0;5.5;0;6.9;0]*pi/180; %with weight
% elseif strcmp(loadStatus,'grip')
    qbias = [-0.85;18.1;-1.7;11.9;-0.15;6.9;-0.3]*pi/180;
    if strcmp(arm,'right')
        qbias = [-1;1;-1;1;-1;1;-1].*qbias;
    end
% elseif strcmp(loadStatus,'noGrip')
%     qbias = [-0.15;11;-1.15;6.85;0;1.3;-0.15]*pi/180;
% else
%     disp('Wrong Load Status')
%     return;
% end


%% set up
%disable peripherals (1 to disable, 0 to have enabled)
disCollAvoid = 1; %collision avoidance
disConSafe = 1; %contact safety
disBodyAvoid = 1; %body avoidance
disGravComp = 1; %gravity compensation
%sound effect for when done
seff = load('gong.mat');

%connect to baxter and send to start position then wait for box
robot = RobotRaconteur.Connect('tcp://localhost:4545/BaxterJointServer/Baxter');
peripherals = RobotRaconteur.Connect('tcp://localhost:46604/BaxterPeripheralServer/BaxterPeripherals');
robot.setControlMode(uint8(3));
peripherals.suppressBodyAvoidance(arm,uint8(disBodyAvoid));
peripherals.suppressCollisionAvoidance(arm,uint8(disCollAvoid));
peripherals.suppressContactSafety(arm,uint8(disConSafe));
peripherals.suppressGravityCompensation(arm,uint8(disGravComp));
baxGrip = RobotRaconteur.Connect('tcp://192.168.1.134:6006/GripperController/gripcon');

pause(5);

%% add path for large movements
addpath('/home/cats/Documents/oakesk/02_10_2021_rawPositionMode/schroed/longRawMove/')


%%
Q0 = qposes(:,armPose)*pi/180;
if strcmp(arm,'right')
    Q0 = [-1;1;-1;1;-1;1;-1].*Q0;
end
[~,~,~] = longMove(robot,interPose,100,0.02,arm,qbias);
    pause(5);
    
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
pause(5)
   
[~,~,~] = longMove(robot,Q0,100,0.02,arm,qbias);
pause(1);
[robot_const,~] = defineBaxter();
ub = robot_const(1).limit.upper_joint_limit;
lb = robot_const(1).limit.lower_joint_limit;

for kkk = 1:length(Tdes)
    
    T = Tdes(kkk);
    
    %fvec = (1/T)*(0:Ns/2-1)/Ns;
    f = 1/T;
    ts = 0:0.0001:T;
    ussingle = usat.*sin(2*pi*ts/T);
    tssingle = ts;
    us = [ussingle,repmat(ussingle(2:end),1,N-1)];
    ts = 0:0.0001:T*N;
    if length(ts)>length(us)
        ts = ts(1:length(us));
    end
    
    
    
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
    
    q0init = robot.joint_positions;
    
    %% set up other variables and figures
    t = zeros(1,20001);
    q_desired=zeros(7,20000);
    q_baxter=zeros(14,20000);
    q_vel_baxter = zeros(14,20000);
    tor_baxter = zeros(14,20000);
    q_error = zeros(7,20000);
    commQ = zeros(14,20000);
    commTor = zeros(14,20000);
    dt = 0.02;
    count = 0;
    q_c = zeros(7,20000);
    
    %%
    pause(4);
    
    disp('starting');
    pause(0.1);
    tstart = tic;
    tmp = peripherals.getNavigatorState('left');
    while t(count+1)<ts(end)%dataFile.t(end)
        tic;
        if tmp.ok_button == 0
            count = count+1;
            
            %q_temp = interp1(dataFile.t,dataFile.us,t(count));
            q_temp = interp1(ts,us,t(count));
            q_desired(:,count) = Q0;
            q_desired(jointnum,count) = q_desired(jointnum,count)+q_temp;
            
            %%%% send to baxter and read position
            q_baxter(:,count) = robot.joint_positions;
            q_vel_baxter(:,count) = robot.joint_velocities;
            tor_baxter(:,count) = robot.joint_torques;
            commTor(:,count) = peripherals.commanded_torque;
            commQ(:,count) = peripherals.commanded_position;
            inds = 1:7;
            if strcmp(arm,'right')
                inds = 8:14;
            end
            q_error(:,count) = q_desired(:,count)-q_baxter(inds,count);
            
            q_c(:,count) = q_desired(:,count)-qbias;
            
            robot.setJointCommand(arm,q_c(:,count));
            t(count+1) = toc(tstart);
        else
            
            break;
        end
        tmp = peripherals.getNavigatorState('left');
        try
            java.lang.Thread.sleep((0.02-toc)*1000);
        catch ME
            pause(0.02-toc);
           % toc
          %  count
        end
    end
    
    disp(num2str(count))
    %% 
    robot.setJointCommand(arm,Q0-qbias);
    
    disp(['Done joint ',num2str(jointnum),' run ',num2str(kkk),' of ',num2str(length(Tdes))])
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
    mkdir(['/home/cats/Documents/oakesk/02_10_2021_rawPositionMode/schroed/',datestr(now,'dd-mmm-yyyy'),'/SingleSines/loadStatus',loadStatus,'/arm',arm,'/Joint',num2str(jointnum),'/Pose',num2str(armPose),'/InterPose',num2str(interPoseNum)])
    save(['/home/cats/Documents/oakesk/02_10_2021_rawPositionMode/schroed/',datestr(now,'dd-mmm-yyyy'),'/SingleSines/loadStatus',loadStatus,'/arm',arm,'/Joint',num2str(jointnum),'/Pose',num2str(armPose),'/InterPose',num2str(interPoseNum),...
        '/baxter_singleFreqSine_jointNumber',num2str(jointnum),'_loadStatus',loadStatus,'_pose',num2str(armPose),'_',...
        'amp',num2str(amp),'_f',num2str(f),'_',datestr(now,'dd-mmm-yyyy-HH-MM-SS'),'.mat'],...
        'q_desired','q_baxter','q_vel_baxter','t','tor_baxter','q_error','jointnum','Q0',...
        'T','N','ts','us','usat','q0init','tssingle','ussingle','f','commTor','commQ','qbias','interPose','loadStatus','arm');
    
    pause(5);
end

robot.setControlMode(uint8(1));
%peripherals.suppressGravityCompensation('left',uint8(0));

sound(seff.y);
toc(twhole)

if strcmp(loadStatus,'weight')
    disp('waiting for cancel button press to open gripper')
    while tmp.cancel_button == 0
        tmp = peripherals.getNavigatorState(arm);
    end
    baxGrip.setPosition(uint8(0),uint8(255),uint8(1));
end

%figure;plot(t(2:end),q_user(jointnum,:)*180/pi,'k',t(2:end),q_baxter(jointnum,:)*180/pi,'LineWidth',2);title(num2str(jointnum))



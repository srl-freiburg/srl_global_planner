%% MOTION PRIMITIVE GENERATOR

% THIS SCRIPT GENERATES AND DISPLAYS MOTION PRIMITIVE BASE ON A UNICYCLE
% KINEMATIC MODEL.
% created by lp@srl 26.11.2013
clear all
% close all
clc


% Define the Velocities' ranges
% WI is the angular velocity range


% % %%% First set of motion primitives
Wmin=-3;
Wmax=3;
Wstep=0.6;
wI=Wmin:Wstep:Wmax;

% VI is the translational velocity range
Vmin=0.8;
Vmax=2;
Vstep=0.2;
vI=Vmin:Vstep:Vmax;


%% Second set of motion primitives
% Wmin=-1.8;
% Wmax=1.8;
% Wstep=0.3;
% wI=Wmin:Wstep:Wmax;
% 
% % VI is the translational velocity range
% Vmin=0.5;
% Vmax=3.0;
% Vstep=0.5;
% vI=Vmin:Vstep:Vmax;


% % % %%% Third set of motion primitives
% Wmin=-4.8;
% Wmax=4.8;
% Wstep=0.6;
% wI=Wmin:Wstep:Wmax;
% 
% % VI is the translational velocity range
% Vmin=0.5;
% Vmax=4.0;
% Vstep=0.5;
% vI=Vmin:Vstep:Vmax;

%%% 4th set of motion primitives

% vI=[0.8 2];
% wI=[-2.6 -1.3 0 1.3 2.6]



k=1;
for i=1:length(vI)
    for j=1:length(wI)
        
        vM(k)=vI(i);
        wM(k)=wI(j);
        k=k+1;
    end
end


% define the motion primitive array
motionPrimitiveCommandArray=[vM' wM'];
q0=[0;0;0];
% Make sure that the time of integration is small!! 
ti=0;
tf=0.3;
dt=0.01;
tSpan=ti:dt:tf;
Qexport=[];
Nexport=[];
Vexport=[];
Wexport=[];
[nr nc]=size(motionPrimitiveCommandArray);
figure(20),hold on
for i=1:nr
        v     = motionPrimitiveCommandArray(i,1);
        omega = motionPrimitiveCommandArray(i,2);
        [T Q] = ode45(@(t,q) unicycleKinematics( t, q, v ,omega ),tSpan,q0); % solve ODE

       figure(20), plot(Q(:,1),Q(:,2),'b','LineWidth',1)
       [n1r nc]=size(Q);
       Nexport=[Nexport;n1r];
       Qexport=[Qexport; Q(:,:)];
       Vexport=[Vexport;v];
       Wexport=[Wexport;omega];
       

end
axis equal,box on,
axisx=xlabel('x [m]')
set(axisx,'FontSize',16);
axisy=ylabel('y [m]')
set(axisy,'FontSize',16);
set(findall(figure(20),'type','text'),'fontSize',18,'fontWeight','light')
% set(gca,'XTick',[],'YTick',[]),box on



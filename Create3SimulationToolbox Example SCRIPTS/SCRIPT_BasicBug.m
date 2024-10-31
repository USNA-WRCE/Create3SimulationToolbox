%% SCRIPT_BasicBug
% Implement a basic bug algorithm using the Create3 Simulation.
%
%   M. Kutzer, 22Oct2024, USNA
clear all
close all
clc

makeVideo = false;

%% Initialize simulation
sim = Create3sim;
% Update room
sim.updateRoom('HP208');
% Update view
view(sim.axs,[-90,90]);
% Update limits
xlim(sim.axs,[-8.5, 3.2]);
ylim(sim.axs,[-5.5,11.7]);
% Update pose
sim.setPose(-2.5,-4.5,-pi/2);
% Enable collsion checking
sim.checkCollosions = true;

%% Define goal
X_des = [-7; 8];
plt_des = plot(sim.axs,X_des(1),X_des(2),'xm','MarkerSize',14,'LineWidth',4);

%% Implement bug algorithm
% Waypoint parameters
Ks = 0.5;   % Speed gain
Kp = 0.8;   % Turn rate gain
d_stop = 0.01;  % Stop condition

% Initialize plot
plt = plot(sim.axs,nan,nan,'m');

if makeVideo
    set(sim.fig,'WindowState','Maximized','Color',[1 1 1]);
    set(sim.axs,'Visible','off');
    vid = VideoWriter('BasicBug_208.mp4','MPEG-4');
    open(vid);
end

xn = [];
ye = [];
while true
    if ~sim.isCollided
        poseInfo = sim.getOdomPose('World');
        xn(end+1) = poseInfo(1);
        ye(end+1) = poseInfo(2);
        psi = poseInfo(6);

        dx = X_des(1) - xn(end);
        dy = X_des(2) - ye(end);

        psi_des = atan2(dy,dx);
        dPsi = wrapToPi(psi_des - psi);
        r_des = Kp*dPsi;

        d = sqrt( dx^2 + dy^2 );

        if d < d_stop
            sim.setVelCmd(0,0);
            break
        end

        u_des = Ks*d;

        sim.setVelCmd(u_des,r_des);
    else
        if makeVideo
            frm = getframe(sim.axs);
            writeVideo(vid,frm);
        end

        sim.setVelCmd(-1,0);
        pause(0.05);
        sim.setVelCmd(0,-1);
        pause(0.20);

        xn(end+1) = poseInfo(1);
        ye(end+1) = poseInfo(2);
    end

    set(plt,'XData',xn,'YData',ye);
    drawnow;

    if makeVideo
        frm = getframe(sim.axs);
        writeVideo(vid,frm);
    end

    pause(0.05);
end

if makeVideo
    close(vid);
end
%% 
% ================================================================
%  STEWART PLATFORM — 3D Inverse Kinematics Simulation
%  File   : stewart_platform_ik.m
%  MATLAB : R2016b or later  (uses local functions at end of script)
%
%  How it works:
%    Given a desired top-plate POSE [x, y, z, roll, pitch, yaw],
%    the inverse kinematics computes each leg length as:
%
%      P_world_i = T + R * p_i        <- top joint in world frame
%      L_i       = norm(P_world_i - b_i)   <- leg length
%
%    where:
%      T   = translation of top plate center (3×1)
%      R   = rotation matrix from RPY angles
%      p_i = top joint pos. in LOCAL frame
%      b_i = base joint pos. in WORLD frame (fixed)
% ================================================================
clear; clc; close all;


%% ===== 1. PLATFORM PARAMETERS ==================================

R_base   = 1.00;    % base plate radius        (m)
R_top    = 0.65;    % top  plate radius        (m)
z_home   = 1.00;    % nominal / home height    (m)
gamma    = 12;      % half-angle of joint pairs (deg)

% ----- Base joint angles: 3 pairs centred at 0°, 120°, 240° ----
b_ang = deg2rad([ -gamma,      +gamma, ...
                   120-gamma,  120+gamma, ...
                   240-gamma,  240+gamma ]);

% ----- Top  joint angles: 3 pairs centred at 60°, 180°, 300° ---
t_ang = deg2rad([  60-gamma,   60+gamma, ...
                  180-gamma,  180+gamma, ...
                  300-gamma,  300+gamma ]);

% ----- Joint position matrices  (3×6, each column = one joint) -
%   B  : base joints in WORLD frame      (fixed, z = 0 plane)
%   P0 : top  joints in PLATFORM frame   (fixed in local coords)
B  = [ R_base*cos(b_ang);  R_base*sin(b_ang);  zeros(1,6) ];
P0 = [ R_top *cos(t_ang);  R_top *sin(t_ang);  zeros(1,6) ];


%% ===== 2. DEFINE TRAJECTORY ====================================
% Pose = [x  y  z  roll  pitch  yaw]  in metres / radians
% Edit this section to try your own motion.

N = 400;
t = linspace(0, 2*pi, N);

traj = [ 0.25*cos(t)',                 ...   % x   – circular sweep
         0.25*sin(t)',                 ...   % y
         (z_home + 0.15*sin(2*t))',   ...   % z   – heave oscillation
         deg2rad(12*sin(t))',          ...   % roll
         deg2rad(12*cos(t))',          ...   % pitch
         zeros(N,1)                    ];   % yaw  (keep zero)


%% ===== 3. FIGURE & AXES SETUP ==================================

fig = figure('Name','Stewart Platform — IK Simulation', ...
             'Color','k', 'Position',[60 60 1050 780]);

ax  = axes('Color','k', ...
           'GridColor',[0.20 0.20 0.20], ...
           'XColor',[0.65 0.65 0.65], ...
           'YColor',[0.65 0.65 0.65], ...
           'ZColor',[0.65 0.65 0.65]);
hold on;  grid on;  axis equal;
view(52, 22);                           % camera azimuth / elevation

xlabel('X (m)', 'Color','w');
ylabel('Y (m)', 'Color','w');
zlabel('Z (m)', 'Color','w');
title('Stewart Platform — Inverse Kinematics Simulation', ...
      'Color','w', 'FontSize',15, 'FontWeight','bold');

xlim([-1.65  1.65]);
ylim([-1.65  1.65]);
zlim([0.00   1.95]);


% ----- Static base plate (drawn once, never updated) ------------
b_ord = convhull(B(1,:)', B(2,:)');    % convex hull vertex order
b_ord = b_ord(1:end-1);               % remove duplicate closing vertex

patch('XData', B(1,b_ord), ...
      'YData', B(2,b_ord), ...
      'ZData', B(3,b_ord), ...
      'FaceColor',[0.10 0.22 0.58], 'FaceAlpha',0.85, ...
      'EdgeColor',[0.28 0.54 1.00], 'LineWidth',1.8);

plot3(B(1,:), B(2,:), B(3,:), 'o', ...
      'MarkerSize',10, ...
      'MarkerFaceColor',[0.35 0.65 1.00], ...
      'MarkerEdgeColor','none');

% Label base joints
for i = 1:6
    text(B(1,i)*1.12, B(2,i)*1.12, 0.04, sprintf('b%d',i), ...
         'Color',[0.5 0.8 1.0], 'FontSize',7.5);
end


% ----- Six legs (one coloured line per leg) ---------------------
cLegs = [ 1.00 0.38 0.08;   % L1 – orange
          1.00 0.72 0.08;   % L2 – amber
          0.72 0.95 0.08;   % L3 – lime
          0.08 0.90 0.50;   % L4 – mint
          0.08 0.62 1.00;   % L5 – sky blue
          0.82 0.12 0.95 ]; % L6 – violet

hLeg = gobjects(1,6);
for i = 1:6
    hLeg(i) = plot3([0 0],[0 0],[0 0], '-', ...
                    'Color', cLegs(i,:), 'LineWidth', 2.8);
end


% ----- Top plate (dynamic patch) --------------------------------
hTop = patch('XData', zeros(6,1), ...
             'YData', zeros(6,1), ...
             'ZData', zeros(6,1), ...
             'FaceColor',[0.12 0.68 0.38], 'FaceAlpha',0.80, ...
             'EdgeColor',[0.28 1.00 0.52], 'LineWidth',1.8);


% ----- Top joint markers ----------------------------------------
hTopJ = plot3(zeros(1,6), zeros(1,6), zeros(1,6), 'o', ...
              'MarkerSize',10, ...
              'MarkerFaceColor',[1.00 0.92 0.18], ...
              'MarkerEdgeColor','none');


% ----- HUD / readout text (top-left corner) ---------------------
hTxt = text(-1.60, -1.60, 1.86, '', ...
            'Color','w', 'FontSize',8.5, 'FontName','Courier New', ...
            'VerticalAlignment','top', 'HorizontalAlignment','left');

% ----- Legend ---------------------------------------------------
for i = 1:6
    text(1.25, -1.55 + (i-1)*0.18, 0.05, sprintf('L%d',i), ...
         'Color', cLegs(i,:), 'FontSize',9, 'FontWeight','bold');
end


%% ===== 4. ANIMATION LOOP =======================================

fprintf('Running simulation...  Close the figure window to stop.\n\n');

for k = 1:N

    if ~isvalid(fig), break; end

    pose = traj(k,:);

    % ---- Decompose pose ----------------------------------------
    T = pose(1:3)';                           % translation  (3×1)
    R = rpy2rot(pose(4), pose(5), pose(6));   % rotation matrix

    % ===========================================================
    %   INVERSE KINEMATICS  (core — 3 lines per leg)
    %
    %   Pw(:,i)  =  T  +  R * P0(:,i)
    %                 ^--- rotate local joint pos into world frame
    %                         then translate by platform centre
    %
    %   L(i)  =  norm( Pw(:,i) - B(:,i) )
    %                  ^--- vector from base anchor to top anchor
    % ===========================================================
    Pw = T + R * P0;          % top joints in world frame  (3×6)

    L  = zeros(1,6);
    for i = 1:6
        L(i) = norm( Pw(:,i) - B(:,i) );
    end

    % ---- Update leg lines --------------------------------------
    for i = 1:6
        set(hLeg(i), ...
            'XData', [B(1,i)  Pw(1,i)], ...
            'YData', [B(2,i)  Pw(2,i)], ...
            'ZData', [B(3,i)  Pw(3,i)]);
    end

    % ---- Update top plate patch --------------------------------
    % convhull gives correct winding order even when plate is tilted
    ch  = convhull(Pw(1,:)', Pw(2,:)');
    ch  = ch(1:end-1);
    set(hTop, ...
        'XData', Pw(1,ch)', ...
        'YData', Pw(2,ch)', ...
        'ZData', Pw(3,ch)');

    % ---- Update top joint markers ------------------------------
    set(hTopJ, ...
        'XData', Pw(1,:), ...
        'YData', Pw(2,:), ...
        'ZData', Pw(3,:));

    % ---- HUD readout -------------------------------------------
    s = sprintf( ...
        'Frame %3d / %d\n  x=%+.3f m  y=%+.3f m  z=%.3f m\n  roll=%+.1f°    pitch=%+.1f°\n\nLeg lengths:\n', ...
        k, N, pose(1), pose(2), pose(3), ...
        rad2deg(pose(4)), rad2deg(pose(5)));

    for i = 1:6
        s = [s  sprintf('  L%d = %.4f m', i, L(i))]; %#ok<AGROW>
        if i == 3
            s = [s  newline]; %#ok<AGROW>
        elseif i < 6
            s = [s  newline]; %#ok<AGROW>
        end
    end

    set(hTxt, 'String', s);

    drawnow limitrate;   % cap at ~20 fps; drop frames if too slow
end

fprintf('Simulation complete.\n');


%% ===== LOCAL FUNCTIONS  (requires MATLAB R2016b+) ==============

function R = rpy2rot(roll, pitch, yaw)
%RPLY2ROT  Rotation matrix from ZYX Euler angles (roll–pitch–yaw)
%
%  Convention:  R = Rz(yaw) * Ry(pitch) * Rx(roll)
%
%  roll  – rotation about X (φ)
%  pitch – rotation about Y (θ)
%  yaw   – rotation about Z (ψ)

    cr = cos(roll);  sr = sin(roll);
    cp = cos(pitch); sp = sin(pitch);
    cy = cos(yaw);   sy = sin(yaw);

    Rx = [1   0    0  ;   0   cr  -sr;   0  sr  cr];
    Ry = [cp  0   sp  ;   0    1    0;  -sp   0  cp];
    Rz = [cy -sy   0  ;  sy   cy    0;   0    0   1];

    R  = Rz * Ry * Rx;
end

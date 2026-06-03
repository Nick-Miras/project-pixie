% ================================================================
%  STEWART PLATFORM — Robot Head Expressions Simulation
%  File   : robot_head_expressions.m
%  Requires: MATLAB R2016b+
%
%  Plays 7 robot head emotions in a continuous loop:
%    NEUTRAL → NOD → SHAKE (no) → CURIOUS → SAD → THINKING → EXCITED
%
%  IK per leg i:
%    P_world_i = T + R * p_local_i     (top joint, world frame)
%    L_i       = norm(P_world_i - b_i) (required leg length)
%
%  Angle sign convention  (ZYX Euler — R = Rz·Ry·Rx):
%    pitch > 0  →  head looks DOWN   (front tilts toward floor)
%    pitch < 0  →  head looks UP     (front tilts toward ceiling)
%    roll  > 0  →  head tilts LEFT
%    yaw   > 0  →  head turns LEFT
% ================================================================
clear; clc; close all;


%% ─── 1. PLATFORM GEOMETRY ──────────────────────────────────────

R_base = 1.00;   R_top = 0.65;   z0 = 0.90;   gamma = 12;

b_ang = deg2rad([ -gamma,+gamma, 120-gamma,120+gamma, 240-gamma,240+gamma ]);
t_ang = deg2rad([ 60-gamma,60+gamma, 180-gamma,180+gamma, 300-gamma,300+gamma ]);

B  = [ R_base*cos(b_ang); R_base*sin(b_ang); zeros(1,6) ];   % base joints (world)
P0 = [ R_top*cos(t_ang);  R_top*sin(t_ang);  zeros(1,6) ];   % top  joints (local)


%% ─── 2. ROBOT HEAD VISUAL GEOMETRY (local frame) ───────────────
% +X = "nose / forward"   +Z = "top of head"

head_r   = 0.28;
head_off = [ 0; 0; head_r + 0.10 ];           % head centre above top plate

eye_L    = head_off + head_r * [  0.70;  0.35;  0.28 ];  % left  eye
eye_R    = head_off + head_r * [  0.70; -0.35;  0.28 ];  % right eye
nose_pt  = head_off + head_r * [  1.00;  0.00; -0.05 ];  % nose dot

% Sphere mesh template (relative to head centre, local frame)
[ssx, ssy, ssz] = sphere(18);
sgrid      = size(ssx);
sphere_tpl = [ ssx(:)'; ssy(:)'; ssz(:)' ] * head_r;   % 3×N


%% ─── 3. EMOTION TRAJECTORIES ───────────────────────────────────
% Each emotion = struct with .name  .color  .traj (N×6 pose matrix)

FPS = 30;
dur = @(s) round(FPS * s);          % seconds → number of frames
mkE = @(n,c,t) struct('name',n,'color',c,'traj',t);
emos = {};

% ── NEUTRAL ──────────────────────────────────────────────────────
N = dur(1.5);
tr = repmat([0, 0, z0, 0, 0, 0], N, 1);
emos{end+1} = mkE('NEUTRAL', [0.80 0.80 0.80], tr);

% ── NOD  (pitch oscillates +/- : forward nod then slight lean back) ──
N = dur(3.0);   tv = linspace(0, 3*2*pi, N)';
tr = zeros(N,6);
tr(:,3) = z0 - 0.04 * max(sin(tv), 0);    % z dips slightly on downstroke
tr(:,5) = deg2rad(22) * sin(tv);           % +pitch = nod forward/down
emos{end+1} = mkE('NOD', [0.20 0.95 0.30], tr);

% ── SHAKE / NO  (yaw left-right) ─────────────────────────────────
N = dur(2.5);   tv = linspace(0, 3*2*pi, N)';
tr = zeros(N,6);
tr(:,3) = z0;
tr(:,6) = deg2rad(20) * sin(tv);
emos{end+1} = mkE('SHAKE  (NO)', [1.00 0.28 0.28], tr);

% ── CURIOUS  (roll tilt, slight upward look, slight yaw) ─────────
N = dur(4.0);   tv = linspace(0, 2*2*pi, N)';
tr = zeros(N,6);
tr(:,3) = z0 + 0.05;
tr(:,4) = deg2rad(22) * sin(tv);                    % roll: head tilts side to side
tr(:,5) = deg2rad(-8) + deg2rad(5)*cos(tv);         % slight upward look
tr(:,6) = deg2rad(10) * sin(tv + pi/3);             % slight yaw offset
emos{end+1} = mkE('CURIOUS  ?', [1.00 0.82 0.10], tr);

% ── SAD  (head droops down, look at floor, slow weary sway) ──────
N = dur(4.5);   tv = linspace(0, 2*pi, N)';
ramp = ssmooth(tv / (1.5*pi));                       % gradual onset ramp
tr = zeros(N,6);
tr(:,3) = z0 - 0.12 * ramp;                         % head sinks
tr(:,5) = deg2rad(25)  * ramp;                      % +pitch = looking down
tr(:,4) = deg2rad(4)   * sin(tv * 0.6);             % slow, weary side sway
emos{end+1} = mkE('SAD', [0.40 0.60 1.00], tr);

% ── THINKING  (look up, slow contemplative yaw to one side) ──────
N = dur(4.0);   tv = linspace(0, 2*pi, N)';
tr = zeros(N,6);
tr(:,3) = z0 + 0.06 + 0.02*sin(tv*0.7);
tr(:,5) = deg2rad(-16) + deg2rad(4)*sin(tv*0.9);    % -pitch = looking up
tr(:,6) = deg2rad(15)  * sin(tv * 0.4);             % slow yaw to one side
tr(:,4) = deg2rad(6)   * sin(tv);                   % gentle roll
emos{end+1} = mkE('THINKING ...', [0.80 0.35 1.00], tr);

% ── EXCITED / HAPPY  (fast bounce + quick rolls) ─────────────────
N = dur(3.0);   tv = linspace(0, 4*2*pi, N)';
tr = zeros(N,6);
tr(:,3) = z0 + 0.10 * abs(sin(tv));                 % energetic z bounce
tr(:,4) = deg2rad(14) * sin(2*tv);                  % quick roll
tr(:,5) = deg2rad(10) * cos(2*tv);                  % quick pitch
emos{end+1} = mkE('EXCITED  !', [1.00 0.55 0.10], tr);


%% ─── 4. CONCATENATE WITH SMOOTH TRANSITIONS ────────────────────
% Emotions play in sequence. Smooth blend of TRANS_N frames between each.

TRANS = 20;
tc = {};  ic = {};

for ke = 1:length(emos)
    % Blend from end of previous emotion to start of this one
    if ~isempty(tc)
        last  = tc{end}(end,:);
        first = emos{ke}.traj(1,:);
        bl = zeros(TRANS, 6);
        for f = 1:TRANS
            a = ssmooth(f/TRANS);
            bl(f,:) = (1-a)*last + a*first;
        end
        tc{end+1} = bl;                          %#ok<SAGROW>
        ic{end+1} = ke * ones(TRANS, 1);         %#ok<SAGROW>
    end
    tc{end+1} = emos{ke}.traj;                              %#ok<SAGROW>
    ic{end+1} = ke * ones(size(emos{ke}.traj,1), 1);        %#ok<SAGROW>
end

% Close the loop: blend back to NEUTRAL so looping is seamless
last  = tc{end}(end,:);
first = emos{1}.traj(1,:);
bl = zeros(TRANS,6);
for f = 1:TRANS
    a = ssmooth(f/TRANS);
    bl(f,:) = (1-a)*last + a*first;
end
tc{end+1} = bl;
ic{end+1} = ones(TRANS,1);   % emotion 1 = NEUTRAL

full_traj = vertcat(tc{:});
full_eidx = vertcat(ic{:});
N_tot     = size(full_traj, 1);


%% ─── 5. FIGURE SETUP ───────────────────────────────────────────

fig = figure('Name','Robot Head — Stewart Platform IK', ...
             'Color','k', 'Position',[50 50 1100 820]);
ax  = axes('Color','k', 'GridColor',[0.18 0.18 0.18], ...
           'XColor',[0.55 0.55 0.55], 'YColor',[0.55 0.55 0.55], ...
           'ZColor',[0.55 0.55 0.55]);
hold on; grid on; axis equal; view(52, 22);
xlabel('X (m)','Color','w');
ylabel('Y (m)','Color','w');
zlabel('Z (m)','Color','w');
title('Stewart Platform — Robot Head Expressions', ...
      'Color','w', 'FontSize',13, 'FontWeight','bold');
xlim([-1.65  1.65]);
ylim([-1.65  1.65]);
zlim([0      2.15]);

% ── Static base plate ────────────────────────────────────────────
b_ord = convhull(B(1,:)', B(2,:)');
b_ord = b_ord(1:end-1);
patch('XData',B(1,b_ord),'YData',B(2,b_ord),'ZData',B(3,b_ord), ...
      'FaceColor',[0.08 0.18 0.50],'FaceAlpha',0.90, ...
      'EdgeColor',[0.22 0.48 1.00],'LineWidth',1.5);
plot3(B(1,:),B(2,:),B(3,:),'o','MarkerSize',9, ...
      'MarkerFaceColor',[0.32 0.60 1.00],'MarkerEdgeColor','none');

% ── Six actuated legs (one colour each) ──────────────────────────
cLegs = [ 1.00 0.38 0.08;   % L1  orange
          1.00 0.72 0.08;   % L2  amber
          0.72 0.95 0.08;   % L3  lime
          0.08 0.90 0.50;   % L4  mint
          0.08 0.62 1.00;   % L5  sky
          0.82 0.12 0.95 ]; % L6  violet
hLeg = gobjects(1,6);
for i = 1:6
    hLeg(i) = plot3([0 0],[0 0],[0 0],'-','Color',cLegs(i,:),'LineWidth',2.5);
end

% ── Moving top plate ─────────────────────────────────────────────
% Top joints are already in CCW angular order, so no convhull needed in loop
hTop  = patch('XData',zeros(6,1),'YData',zeros(6,1),'ZData',zeros(6,1), ...
              'FaceColor',[0.10 0.58 0.32],'FaceAlpha',0.72, ...
              'EdgeColor',[0.22 0.95 0.48],'LineWidth',1.5);
hTopJ = plot3(zeros(1,6),zeros(1,6),zeros(1,6),'o','MarkerSize',8, ...
              'MarkerFaceColor',[1.00 0.90 0.15],'MarkerEdgeColor','none');

% ── Robot head sphere ────────────────────────────────────────────
hHead = surf(ax, zeros(sgrid), zeros(sgrid), zeros(sgrid), ...
             'FaceColor',[0.55 0.60 0.65], 'FaceAlpha',0.88, ...
             'EdgeAlpha',0.10, 'EdgeColor',[0.40 0.40 0.40], 'LineWidth',0.3);

% ── Eyes (white dots) ────────────────────────────────────────────
hEyes = plot3([0 0],[0 0],[0 0],'o','MarkerSize',11, ...
              'MarkerFaceColor',[0.90 0.98 1.00],'MarkerEdgeColor','none');

% ── Nose (orange dot, marks the "front" direction) ───────────────
hNose = plot3(0,0,0,'o','MarkerSize',9, ...
              'MarkerFaceColor',[1.00 0.52 0.12],'MarkerEdgeColor','none');

% ── Emotion label (large, centred at top) ────────────────────────
hLbl = text(0, 0, 2.10, 'NEUTRAL', ...
            'Color','w','FontSize',26,'FontWeight','bold', ...
            'HorizontalAlignment','center','VerticalAlignment','top');

% ── Left info panel: pose angles ─────────────────────────────────
hInfo = text(-1.60,-1.60,2.07,'','Color',[0.70 0.70 0.70],'FontSize',8.5, ...
             'FontName','Courier New','VerticalAlignment','top');

% ── Right info panel: leg lengths ────────────────────────────────
hLegTxt = text(1.10,-1.60,2.07,'','Color',[0.70 0.70 0.70],'FontSize',8.5, ...
               'FontName','Courier New','VerticalAlignment','top');


%% ─── 6. ANIMATION (loops continuously until window is closed) ──

fprintf('Running robot head expression loop...\n');
fprintf('Close the figure window to stop.\n\n');
prev_e = 0;

while isvalid(fig)
    for k = 1:N_tot
        if ~isvalid(fig), break; end

        pose = full_traj(k,:);
        ei   = full_eidx(k);
        T    = pose(1:3)';
        R    = rpy2rot(pose(4), pose(5), pose(6));

        % ========================================================
        %   INVERSE KINEMATICS
        %   For each leg i:
        %     P_world = T + R * P0_i    (top joint, world frame)
        %     L_i     = norm(P_world - B_i)
        % ========================================================
        Pw = T + R * P0;          % 3×6
        L  = arrayfun(@(i) norm(Pw(:,i) - B(:,i)), 1:6);

        % ── Update legs ─────────────────────────────────────────
        for i = 1:6
            set(hLeg(i), ...
                'XData',[B(1,i) Pw(1,i)], ...
                'YData',[B(2,i) Pw(2,i)], ...
                'ZData',[B(3,i) Pw(3,i)]);
        end

        % ── Update top plate ────────────────────────────────────
        set(hTop,  'XData',Pw(1,:)','YData',Pw(2,:)','ZData',Pw(3,:)');
        set(hTopJ, 'XData',Pw(1,:), 'YData',Pw(2,:), 'ZData',Pw(3,:));

        % ── Update head sphere ───────────────────────────────────
        hc = T + R * head_off;                      % head centre, world
        sw = hc + R * sphere_tpl;                   % transformed mesh (3×N)
        set(hHead, ...
            'XData', reshape(sw(1,:), sgrid), ...
            'YData', reshape(sw(2,:), sgrid), ...
            'ZData', reshape(sw(3,:), sgrid));

        % ── Update eyes & nose ──────────────────────────────────
        eL = T + R * eye_L;
        eR = T + R * eye_R;
        ns = T + R * nose_pt;
        set(hEyes, 'XData',[eL(1) eR(1)], 'YData',[eL(2) eR(2)], 'ZData',[eL(3) eR(3)]);
        set(hNose, 'XData',ns(1),          'YData',ns(2),          'ZData',ns(3));

        % ── Update emotion label ─────────────────────────────────
        if ei ~= prev_e
            set(hLbl, 'String', emos{ei}.name, 'Color', emos{ei}.color);
            prev_e = ei;
        end

        % ── Update info panels ───────────────────────────────────
        set(hInfo, 'String', sprintf( ...
            'Pose\n x=%+.2f  y=%+.2f  z=%.2f\n roll  =%+.0f°\n pitch =%+.0f°\n yaw   =%+.0f°', ...
            pose(1),pose(2),pose(3), ...
            rad2deg(pose(4)), rad2deg(pose(5)), rad2deg(pose(6))));

        set(hLegTxt, 'String', sprintf( ...
            'Legs (m)\n L1 = %.3f\n L2 = %.3f\n L3 = %.3f\n L4 = %.3f\n L5 = %.3f\n L6 = %.3f', ...
            L(1),L(2),L(3),L(4),L(5),L(6)));

        drawnow limitrate;
    end
end

fprintf('Window closed — simulation ended.\n');


%% ─── LOCAL FUNCTIONS ────────────────────────────────────────────

function R = rpy2rot(roll, pitch, yaw)
%RPLY2ROT  Rotation matrix from ZYX Euler angles
%  R = Rz(yaw) * Ry(pitch) * Rx(roll)
    cr=cos(roll); sr=sin(roll);
    cp=cos(pitch); sp=sin(pitch);
    cy=cos(yaw);   sy=sin(yaw);
    Rx = [1   0    0;   0  cr  -sr;   0  sr  cr];
    Ry = [cp  0   sp;   0   1    0;  -sp   0  cp];
    Rz = [cy -sy   0;  sy  cy    0;   0    0   1];
    R  = Rz * Ry * Rx;
end

function y = ssmooth(x)
%SSMOOTH  Smoothstep: maps [0,1] → [0,1] with zero derivative at both ends
    x = max(0, min(1, x));
    y = x.^2 .* (3 - 2.*x);
end

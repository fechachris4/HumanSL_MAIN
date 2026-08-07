% show_frames.m — the coordinate frames of the Stage 2 pipeline, drawn in
% base_link: the frame goal.yaml, the controller prints, and pipe targets
% still use. This is a base_link-only diagram and deliberately stays that
% way; for the mounted pair and the mount frame use show_dual_arm_frames.
%
% base_link is NOT the mount frame. Since 2026-08-05 the model has a real
% `mount` root at the midpoint of the two arm bases, and a pipe target may
% declare its frame ("WORLD x y z" or "BASE x y z", no prefix meaning
% base_link) — see docs/decisions/custom-urdf.md. Everything drawn here is
% base_link, so a WORLD target will NOT appear where these axes suggest.
% Run:  show_frames
function show_frames()
    figure('Name', 'HumanSL frames (right arm, base_link)'); hold on; grid on;

    % base_link: the working frame. Origin = right-arm base. z up.
    draw_triad(eye(3), [0 0 0], 0.25, 'base\_link (goals live HERE)');

    % Planner's DH base frame (Kinova Table 94): 180 deg about x wrt
    % base_link — y and z sign-flip. Drawn slightly offset so both triads
    % are visible; physically they share the origin.
    Rx180 = [1 0 0; 0 -1 0; 0 0 -1];
    draw_triad(Rx180, [0.02 0.02 0.02], 0.15, 'DH base (planner internal)');

    % Reference points, metres, base_link — measured 2026-08-05. These are
    % recorded MEASUREMENTS, not targets, so they stay as literals; they are
    % labelled with the date so a stale one is obvious.
    pts = [ 0.0    -0.0246  1.3073;   % tool at zero configuration
            0.0826 -0.0195  0.7236];  % measured startup pose, 2026-08-05 am
    labels = {'tool @ zero config (2026-08-05)', 'startup pose (2026-08-05 am)'};
    for i = 1:size(pts,1)
        plot3(pts(i,1), pts(i,2), pts(i,3), 'ko', 'MarkerFaceColor', 'y');
        text(pts(i,1)+0.02, pts(i,2), pts(i,3), labels{i});
    end

    % The goal is READ from goal.yaml, never hardcoded here: a copy in this
    % script goes stale the moment the file changes, and a stale goal drawn
    % as if current is worse than no goal at all. The file declares its own
    % frame, so convert to base_link when it is not already base_link.
    scriptDir = fileparts(mfilename('fullpath'));
    repo = fileparts(fileparts(fileparts(scriptDir)));
    goalFile = fullfile(repo, 'Christian_control', 'planner_bridge', ...
                        'config', 'goal.yaml');
    mountFile = fullfile(scriptDir, '..', 'config', 'dual_arm_mounting.yaml');
    [goal, goalFrame] = read_goal(goalFile, mountFile);
    if ~isempty(goal)
        plot3(goal(1), goal(2), goal(3), 'rp', 'MarkerSize', 14, 'MarkerFaceColor', 'r');
        text(goal(1)+0.02, goal(2), goal(3), ...
             sprintf('goal.yaml (declared %s)', goalFrame));
        fprintf('goal.yaml declares frame %s; drawn here in base_link as [%.4f %.4f %.4f]\n', ...
                goalFrame, goal(1), goal(2), goal(3));
    end

    % Reach envelope, to eyeball feasibility: ~0.9 m sphere about base_link.
    [sx, sy, sz] = sphere(24);
    surf(0.9*sx, 0.9*sy, 0.9*sz, 'FaceAlpha', 0.05, ...
         'EdgeAlpha', 0.1, 'FaceColor', [0.3 0.5 0.9]);

    xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
    axis equal; view(135, 20);
    title('base\_link: goals, prints, and unprefixed pipe targets live here');
end

% Reads `goal:` and its `frame:` from goal.yaml, returning the point in
% base_link. Mirrors the bridge: an omitted frame means the compiled
% config::kReferenceFrame, which is mount.
function [goal_base, frameName] = read_goal(goalFile, mountFile)
    goal_base = []; frameName = 'mount';
    if ~isfile(goalFile), return, end
    txt = string(fileread(goalFile));
    lines = splitlines(txt);
    nums = [];
    for k = 1:numel(lines)
        line = extractBefore(lines(k) + "#", "#");     % strip comments
        if contains(line, "frame:")
            frameName = char(strtrim(extractAfter(line, "frame:")));
        elseif contains(line, "goal:")
            nums = sscanf(char(extractAfter(line, "goal:")), '[%f, %f, %f]');
        end
    end
    if numel(nums) ~= 3, return, end
    goal = nums(:);
    switch frameName
        case 'right_base'
            goal_base = goal;
        case {'mount', 'left_base'}
            % T_mount_rightbase from the mounting file — the same source of
            % truth the URDF is built from, so this cannot drift from it.
            sep = read_scalar(mountFile, 'base_separation_m');
            tilt = read_scalar(mountFile, 'mount_tilt_rad');
            Rr = rot_x(tilt);  tr = [0; -sep/2; 0];
            if strcmp(frameName, 'left_base')
                Rl = rot_x(-tilt); tl = [0; sep/2; 0];
                goal = Rl * goal + tl;                 % left_base -> mount
            end
            goal_base = Rr' * (goal - tr);             % mount -> right_base
        otherwise
            error('unknown frame ''%s'' in %s', frameName, goalFile);
    end
end

function R = rot_x(a)
    R = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];
end

function value = read_scalar(path, key)
    lines = splitlines(string(fileread(path)));
    for k = 1:numel(lines)
        line = extractBefore(lines(k) + "#", "#");
        parts = split(line, ":");
        if numel(parts) >= 2 && strtrim(parts(1)) == key
            value = str2double(strtrim(parts(2)));
            return
        end
    end
    error('no key ''%s'' in %s', key, path);
end

function draw_triad(R, o, len, name)
    c = {'r', 'g', 'b'};                 % x red, y green, z blue
    ax = {'x', 'y', 'z'};
    for k = 1:3
        v = R(:,k) * len;
        quiver3(o(1), o(2), o(3), v(1), v(2), v(3), 0, c{k}, 'LineWidth', 2);
        text(o(1)+v(1), o(2)+v(2), o(3)+v(3), ax{k});
    end
    text(o(1), o(2), o(3)-0.05, name, 'FontWeight', 'bold');
end

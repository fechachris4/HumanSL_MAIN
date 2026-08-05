% show_frames.m — the coordinate frames of the Stage 2 pipeline, drawn in
% base_link: the frame goal.yaml, the controller prints, and pipe targets
% still use. This is a base_link-only diagram and deliberately stays that
% way; for the mounted pair and the world frame use show_dual_arm_frames.
%
% base_link is NOT the world frame. Since 2026-08-05 the model has a real
% `world` root at the midpoint of the two arm bases, and a pipe target may
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

    % Reference points, metres, base_link — measured 2026-08-05.
    pts = [ 0.0    -0.0246  1.3073;   % tool at zero configuration
            0.0826 -0.0195  0.7236];  % measured startup pose, morning run
    labels = {'tool @ zero config', 'startup pose (this morning)'};
    for i = 1:size(pts,1)
        plot3(pts(i,1), pts(i,2), pts(i,3), 'ko', 'MarkerFaceColor', 'y');
        text(pts(i,1)+0.02, pts(i,2), pts(i,3), labels{i});
    end

    % Your current goal.yaml goal — edit to match the file.
    goal = [0.5 0.0 0.5];
    plot3(goal(1), goal(2), goal(3), 'rp', 'MarkerSize', 14, 'MarkerFaceColor', 'r');
    text(goal(1)+0.02, goal(2), goal(3), 'goal.yaml');

    % Reach envelope, to eyeball feasibility: ~0.9 m sphere about base_link.
    [sx, sy, sz] = sphere(24);
    surf(0.9*sx, 0.9*sy, 0.9*sz, 'FaceAlpha', 0.05, ...
         'EdgeAlpha', 0.1, 'FaceColor', [0.3 0.5 0.9]);

    xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
    axis equal; view(135, 20);
    title('base\_link: goals, prints, and unprefixed pipe targets live here');
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

% show_dual_arm_frames.m — ONE figure: the dual-mounted Gen3 model in its
% posed configuration, with world/base_link/leftbase_link/tool frames,
% the world-origin point, the goal.yaml target, and the tool-to-goal
% reach line overlaid. Fully interactive (orbit/pan/zoom).
%
% Edit repo and GOAL_BASE below to match your machine and current
% Christian_control/planner_bridge/config/goal.yaml, then run:
%   show_dual_arm_frames

repo = "/Users/christian/CLionProjects/HumanSL_MAIN";
urdfDir = fullfile(repo, "Christian_control", "basic_control", "config");
packageRoot = fullfile(repo, "third_party", "ros_kortex", "kortex_description");

% Keep this in sync with Christian_control/planner_bridge/config/goal.yaml
% ("goal: [x, y, z]") — expressed in the right arm's base_link, metres.
GOAL_BASE = [0.5; 0.0; 0.5];

dualText = replace(fileread(fullfile(urdfDir, "GEN3_dual_mounted.urdf")), ...
    "package://kortex_description", packageRoot);
dual = importrobot(dualText, "urdf", DataFormat="struct");

dualPose = homeConfiguration(dual);
rightArm = [0 0 0 0 0 0 0];
leftArm  = [0 0 0 0 0 0 0];
angles = [rightArm leftArm];
for index = 1:numel(angles)
    dualPose(index).JointPosition = angles(index);
end

figure("Color", "w", "Position", [100 100 1000 800]);
show(dual, dualPose, Visuals="on", Collisions="off", Frames="off");
hold on; axis equal
view(45, 18); camva(6)
title("Dual-mounted Gen3 — world vs right base\_link (goals live in base\_link)")

% Transforms out of the model itself, so nothing is hand-copied.
% dual.BaseName is whatever importrobot picked as the tree's root — the
% URDF's "world" link — so ask for its own transform rather than assuming
% it sits at eye(4).
T_world = getTransform(dual, dualPose, dual.BaseName);         % world frame origin
T_rbase = getTransform(dual, dualPose, "base_link");           % right arm base
T_lbase = getTransform(dual, dualPose, "leftbase_link");       % left arm base
T_tool  = getTransform(dual, dualPose, "ConfiguredTool_Link"); % goal frame's tool point

% World frame origin: red dot.
plot3(T_world(1,4), T_world(2,4), T_world(3,4), "o", ...
      MarkerSize=20, MarkerFaceColor="r", MarkerEdgeColor="k", LineWidth=1.5);
drawTriad(T_world, 0.30, "world (URDF root)");
drawTriad(T_rbase, 0.22, "base\_link (goals live HERE)");
drawTriad(T_lbase, 0.22, "leftbase\_link");
drawTriad(T_tool,  0.12, "ConfiguredTool\_Link");

% The TRUE midpoint between the two arms' actual base_link frames is NOT
% the same point as world. rot_linktras / leftrot_linktras apply the
% 0.16 m lateral offset in EACH SIDE'S OWN tilted frame (post the
% +-1.2085 rad roll), so the offset projects partly onto z; the two
% z-components add instead of cancelling while x/y still cancel by
% symmetry. Marked separately (magenta square) so the gap is visible
% rather than assumed — earlier claims that world's origin IS the
% midpoint were only true for the pre-offset base_linktras/
% leftbase_linktras frames, not for base_link/leftbase_link.
midpoint = (T_rbase(1:3,4) + T_lbase(1:3,4)) / 2;
fprintf("base_link origin     = (%.4f, %.4f, %.4f)\n", T_rbase(1:3,4));
fprintf("leftbase_link origin = (%.4f, %.4f, %.4f)\n", T_lbase(1:3,4));
fprintf("TRUE midpoint of base_link/leftbase_link = (%.4f, %.4f, %.4f)\n", midpoint);
fprintf("  vs. world origin                        = (%.4f, %.4f, %.4f)\n", T_world(1:3,4));
fprintf("  gap = %.4f m along z (x/y cancel by symmetry, z does not)\n", ...
    midpoint(3) - T_world(3,4));

plot3(midpoint(1), midpoint(2), midpoint(3), "s", ...
      MarkerSize=18, MarkerFaceColor="m", MarkerEdgeColor="k", LineWidth=1.5);
text(midpoint(1), midpoint(2), midpoint(3)-0.06, ...
     "TRUE midpoint of base\_link/leftbase\_link (not world)", ...
     "FontWeight", "bold", "Color", "m");

% world is not just a label: two fixed joints per arm carry it to
% base_link (GEN3_dual_mounted.urdf) —
%   right_base_rotation: rpy [1.2085 0 0] xyz [0 0 0]      world -> base_linktras
%   rot_linktras:        rpy [0 0 0]      xyz [0 -0.16 0]  base_linktras -> base_link
%   (left arm mirrors with rpy [-1.2085 0 0] and xyz [0 0.16 0])
% 1.2085 rad = 69.24 deg: the physical mount tilt of each arm off the
% shared world frame.
text(0, 0, -0.10, sprintf(['world root: right mount tilts %.1f%c about x, ' ...
    'then +%.2f m offset in y\nleft mount tilts %.1f%c, then %.2f m offset ' ...
    '-- goals/prints/pipe targets are in base\\_link, not world'], ...
    rad2deg(1.2085), char(176), 0.16, -rad2deg(1.2085), char(176), -0.16), ...
    "FontSize", 8, "Color", [0.3 0.3 0.3]);

% goal.yaml is expressed in base_link -> lift it into world to plot it.
goal_world = T_rbase * [GOAL_BASE; 1];
plot3(goal_world(1), goal_world(2), goal_world(3), "rp", ...
      MarkerSize=16, MarkerFaceColor="r");
text(goal_world(1)+0.03, goal_world(2), goal_world(3), "goal.yaml");

% Line from the tool to the goal: the move the planner is asked for.
toolP = T_tool(1:3, 4);
plot3([toolP(1) goal_world(1)], [toolP(2) goal_world(2)], ...
      [toolP(3) goal_world(3)], "r--", LineWidth=1.5);

% ~0.9 m reach shell about the right base, in world coordinates.
[sx, sy, sz] = sphere(28);
c = T_rbase(1:3, 4);
surf(0.9*sx + c(1), 0.9*sy + c(2), 0.9*sz + c(3), ...
     FaceAlpha=0.04, EdgeAlpha=0.08, FaceColor=[0.3 0.5 0.9]);

% Free-look camera: rotate3d is the default drag interaction; switch to
% pan or zoom any time via the axes toolbar (top-right of the plot), the
% Tools menu, or from the command line with `pan on` / `zoom on`.
rotate3d on
axtoolbar(gca, {"rotate", "pan", "zoomin", "zoomout", "restoreview"});

function drawTriad(T, len, name)
    o = T(1:3, 4); R = T(1:3, 1:3);
    colors = ["r" "g" "b"];
    for k = 1:3
        v = R(:,k) * len;
        quiver3(o(1), o(2), o(3), v(1), v(2), v(3), 0, colors(k), ...
                LineWidth=2.5, MaxHeadSize=0.6);
    end
    text(o(1), o(2), o(3)-0.06, name, FontWeight="bold");
end

function show_dual_arm_frames(rightDeg, leftDeg, goal, goalFrame)
% show_dual_arm_frames — ONE figure: the dual-mounted Gen3 model posed in the
% WORLD frame, with the world/base_link/leftbase_link/tool frames, the two
% arms' reach shells, and a goal point drawn in whichever frame you declare.
% Fully interactive (orbit/pan/zoom).
%
% Since 2026-08-05 `world` sits at the MIDPOINT of the two base_link origins
% (Christian_control/docs/decisions/custom-urdf.md). This script checks that
% rather than assuming it: it recomputes the midpoint from the posed model and
% reports the residual, so if the URDF mounting block is ever edited the
% figure says so instead of quietly drawing the wrong thing.
%
% Usage (every argument optional):
%   show_dual_arm_frames                            % both arms at q = 0
%   show_dual_arm_frames(rightDeg)                  % 1x7 degrees, right arm
%   show_dual_arm_frames(rightDeg, leftDeg)         % both arms
%   show_dual_arm_frames(rightDeg, leftDeg, [x y z], "WORLD")
%   show_dual_arm_frames(rightDeg, leftDeg, [x y z], "BASE")   % right base_link
%
% CROSS-CHECK: the numbers printed below come from MATLAB's own URDF parser,
% independently of the C++/Pinocchio path. They should agree to ~1e-6 m with
%   cd Christian_control/basic_control && ./build/print_dual_arm_fk
% Run both and compare. A disagreement means the two parsers read the same
% URDF differently, which is worth knowing before trusting either.
%
% Requires the Robotics System Toolbox (importrobot).

    if nargin < 1 || isempty(rightDeg), rightDeg = zeros(1, 7); end
    if nargin < 2 || isempty(leftDeg),  leftDeg  = zeros(1, 7); end
    if nargin < 3, goal = []; end
    if nargin < 4 || isempty(goalFrame), goalFrame = "BASE"; end
    goalFrame = upper(string(goalFrame));
    if ~ismember(goalFrame, ["WORLD", "BASE"])
        error('goalFrame must be "WORLD" or "BASE"');
    end
    if numel(rightDeg) ~= 7 || numel(leftDeg) ~= 7
        error("rightDeg and leftDeg must each hold 7 joint angles in degrees");
    end

    % ---------------------------------------------------------------
    % Locate the repo from this script's own path, so the file needs no
    % editing to move between machines.
    % ---------------------------------------------------------------
    % Walk up with fileparts rather than fullfile(..,"..",".."): it yields a
    % clean absolute path with no embedded "..", which matters because this
    % path is substituted into the URDF's mesh filenames below. Works
    % unchanged on macOS, Linux and Windows — fullfile picks the separator.
    %   scripts -> basic_control -> Christian_control -> repo root
    scriptDir = fileparts(mfilename("fullpath"));
    repo = fileparts(fileparts(fileparts(scriptDir)));
    urdfDir = fullfile(repo, "Christian_control", "basic_control", "config");
    packageRoot = fullfile(repo, "third_party", "ros_kortex", "kortex_description");
    urdfPath = fullfile(urdfDir, "GEN3_dual_mounted.urdf");
    if ~isfile(urdfPath)
        error(['cannot find %s\n' ...
               'This script locates the repo from its own path, so it must ' ...
               'stay in <repo>/Christian_control/basic_control/scripts/.'], ...
              urdfPath);
    end
    if ~isfolder(packageRoot)
        error(['cannot find the vendored meshes at %s\n' ...
               'They are tracked in git; a partial checkout will not draw ' ...
               'the arms.'], packageRoot);
    end

    % The mounting numbers' source of truth, read for the annotation so the
    % figure cannot drift from the file the way the old hardcoded text did.
    mountYaml = fullfile(urdfDir, "dual_arm_mounting.yaml");
    separation = readYamlScalar(mountYaml, "base_separation_m");
    tilt = readYamlScalar(mountYaml, "mount_tilt_rad");

    dualText = replace(fileread(urdfPath), "package://kortex_description", packageRoot);
    dual = importrobot(dualText, "urdf", DataFormat="struct");

    % ---------------------------------------------------------------
    % Pose the model BY JOINT NAME. Never index the configuration array by
    % position: the two arms' joints are not guaranteed to be contiguous or
    % ordered, which is the same reason the C++ side resolves them through
    % getJointId/idx_q rather than assuming a layout.
    % ---------------------------------------------------------------
    dualPose = homeConfiguration(dual);
    rightNames = "Actuator"     + string(1:7);
    leftNames  = "leftActuator" + string(1:7);
    dualPose = setJointsByName(dualPose, rightNames, deg2rad(rightDeg));
    dualPose = setJointsByName(dualPose, leftNames,  deg2rad(leftDeg));

    T_world = getTransform(dual, dualPose, dual.BaseName);
    T_rbase = getTransform(dual, dualPose, "base_link");
    T_lbase = getTransform(dual, dualPose, "leftbase_link");
    % NOTE: the two tool frames are NOT the same point on the arm. The right
    % chain ends at ConfiguredTool_Link (the tool physically mounted on the
    % right flange); the left has no such tool and ends at its bare flange.
    % Do not read the two tool positions as a symmetry check.
    T_rtool = getTransform(dual, dualPose, "ConfiguredTool_Link");
    T_ltool = getTransform(dual, dualPose, "leftEndEffector_Link");

    % ---------------------------------------------------------------
    % Figure
    % ---------------------------------------------------------------
    figure("Color", "w", "Position", [100 100 1000 800]);
    show(dual, dualPose, Visuals="on", Collisions="off", Frames="off");
    hold on; axis equal
    view(45, 18); camva(6)
    xlabel("x [m]"); ylabel("y [m]"); zlabel("z [m]");
    title(sprintf(['Dual-mounted Gen3 in WORLD - bases %.4f m apart, ' ...
                   'tilted %c%.2f%c about x'], separation, char(177), ...
                  rad2deg(tilt), char(176)));

    plot3(T_world(1,4), T_world(2,4), T_world(3,4), "o", ...
          MarkerSize=20, MarkerFaceColor="r", MarkerEdgeColor="k", LineWidth=1.5);
    drawTriad(T_world, 0.30, "world (URDF root, = base midpoint)");
    drawTriad(T_rbase, 0.22, "base\_link (right)");
    drawTriad(T_lbase, 0.22, "leftbase\_link");
    drawTriad(T_rtool, 0.12, "ConfiguredTool\_Link");
    drawTriad(T_ltool, 0.12, "leftEndEffector\_Link");

    % ---------------------------------------------------------------
    % Check, do not assert: is world really the midpoint?
    % ---------------------------------------------------------------
    midpoint = (T_rbase(1:3,4) + T_lbase(1:3,4)) / 2;
    residual = norm(midpoint - T_world(1:3,4));
    if residual < 1e-9
        fprintf(['world IS the midpoint of base_link/leftbase_link ' ...
                 '(residual %.2e m)\n'], residual);
    else
        % Loud, and drawn, because every world-frame number depends on it.
        fprintf(2, ['WARNING: world is NOT the midpoint - off by %.6f m. ' ...
                    'The URDF mounting block no longer matches ' ...
                    'dual_arm_mounting.yaml.\n'], residual);
        plot3(midpoint(1), midpoint(2), midpoint(3), "s", ...
              MarkerSize=18, MarkerFaceColor="m", MarkerEdgeColor="k", LineWidth=1.5);
        text(midpoint(1), midpoint(2), midpoint(3)-0.06, ...
             "TRUE midpoint (world has drifted from it)", ...
             FontWeight="bold", Color="m");
    end

    % ---------------------------------------------------------------
    % The table to diff against ./build/print_dual_arm_fk
    % ---------------------------------------------------------------
    fprintf("\nmodel: %s\n", urdfPath);
    printMount("right base_link in world", T_rbase);
    printMount("left  base_link in world", T_lbase);
    fprintf("\nFK at the requested configuration (angles given in degrees):\n");
    printArm("right", "ConfiguredTool_Link",  "base_link",     T_rtool, T_rbase);
    printArm("left ", "leftEndEffector_Link", "leftbase_link", T_ltool, T_lbase);
    fprintf(['\n(left figures are OPEN-LOOP: the left arm has no connection ' ...
             'and no feedback)\n']);

    % ---------------------------------------------------------------
    % Goal point, in whichever frame it was declared
    % ---------------------------------------------------------------
    if ~isempty(goal)
        goal = goal(:);
        if goalFrame == "BASE"
            goalWorld = T_rbase * [goal; 1];
            goalWorld = goalWorld(1:3);
            fprintf("goal (base_link) [%.4f %.4f %.4f] -> world [%.4f %.4f %.4f]\n", ...
                    goal, goalWorld);
        else
            goalWorld = goal;
            goalBase = T_rbase \ [goal; 1];
            fprintf("goal (world) [%.4f %.4f %.4f] -> base_link [%.4f %.4f %.4f]\n", ...
                    goal, goalBase(1:3));
        end
        plot3(goalWorld(1), goalWorld(2), goalWorld(3), "rp", ...
              MarkerSize=16, MarkerFaceColor="r");
        text(goalWorld(1)+0.03, goalWorld(2), goalWorld(3), ...
             sprintf("goal (%s)", goalFrame));
        toolP = T_rtool(1:3, 4);
        plot3([toolP(1) goalWorld(1)], [toolP(2) goalWorld(2)], ...
              [toolP(3) goalWorld(3)], "r--", LineWidth=1.5);
    end

    % ~0.9 m reach shell about each base, so feasibility is eyeballable.
    drawReachShell(T_rbase(1:3,4), 0.9, [0.3 0.5 0.9]);
    drawReachShell(T_lbase(1:3,4), 0.9, [0.9 0.5 0.3]);

    rotate3d on
    axtoolbar(gca, {"rotate", "pan", "zoomin", "zoomout", "restoreview"});
end

% =====================================================================

function config = setJointsByName(config, names, radians)
    have = string({config.JointName});
    for k = 1:numel(names)
        index = find(have == names(k), 1);
        if isempty(index)
            error("the model has no joint named '%s'", names(k));
        end
        config(index).JointPosition = radians(k);
    end
end

function printMount(label, T)
    % Roll about x, recovered from a rotation expected to BE a roll.
    roll = atan2(T(3,2), T(3,3));
    fprintf("%s: p [% .6f % .6f % .6f]  roll_x % .6f rad\n", ...
            label, T(1,4), T(2,4), T(3,4), roll);
end

function printArm(label, toolFrame, baseFrame, T_tool, T_base)
    T_inBase = T_base \ T_tool;
    fprintf("  %s  tool frame %s\n", label, toolFrame);
    fprintf("    world      p %s   rpy %s\n", ...
            fmtXyz(T_tool(1:3,4)), fmtRpy(T_tool(1:3,1:3)));
    fprintf("    %-10s p %s   rpy %s\n", baseFrame, ...
            fmtXyz(T_inBase(1:3,4)), fmtRpy(T_inBase(1:3,1:3)));
end

function s = fmtXyz(p)
    s = sprintf('%11.6f %11.6f %11.6f', p(1), p(2), p(3));
end

function s = fmtRpy(R)
    % R = Rz*Ry*Rx, printed roll pitch yaw — the same convention as
    % RotationFromRpy and the controller's orientation line.
    pitch = asin(-R(3,1));
    roll  = atan2(R(3,2), R(3,3));
    yaw   = atan2(R(2,1), R(1,1));
    s = sprintf('%11.6f %11.6f %11.6f', roll, pitch, yaw);
end

function value = readYamlScalar(path, key)
    % Minimal reader for the flat `key: value` mounting file. Errors rather
    % than defaulting, so a renamed key is visible instead of silently
    % drawing stale geometry.
    if ~isfile(path)
        error("cannot find %s", path);
    end
    lines = splitlines(string(fileread(path)));
    for k = 1:numel(lines)
        line = extractBefore(lines(k) + "#", "#");   % strip comments
        parts = split(line, ":");
        if numel(parts) >= 2 && strtrim(parts(1)) == key
            value = str2double(strtrim(parts(2)));
            if isnan(value)
                error("key '%s' in %s is not a number", key, path);
            end
            return
        end
    end
    error("no key '%s' in %s", key, path);
end

function drawReachShell(centre, radius, colour)
    [sx, sy, sz] = sphere(28);
    surf(radius*sx + centre(1), radius*sy + centre(2), radius*sz + centre(3), ...
         FaceAlpha=0.04, EdgeAlpha=0.08, FaceColor=colour);
end

function drawTriad(T, len, name)
    o = T(1:3, 4); R = T(1:3, 1:3);
    colours = ["r" "g" "b"];
    for k = 1:3
        v = R(:,k) * len;
        quiver3(o(1), o(2), o(3), v(1), v(2), v(3), 0, colours(k), ...
                LineWidth=2.5, MaxHeadSize=0.6);
    end
    text(o(1), o(2), o(3)-0.06, name, FontWeight="bold");
end

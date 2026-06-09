/*
 * basic_control.cpp
 * --------------------------------------------------------------------------
 * Minimal, standalone point-to-point control for a SINGLE Kinova Gen3 7-DoF
 * arm, using ONLY the Kinova Kortex high-level Base API.
 *
 * It deliberately does NOT use GTSAM / GPMP2 / Pinocchio / Vicon — none of the
 * HumanSL planning stack is needed just to move the arm from A to B. It reuses
 * only the bundled Kortex API in ../third_party/kortex_api.
 *
 * High-level servoing (SINGLE_LEVEL_SERVOING) keeps the arm under Kinova's own
 * supervised controller: joint/Cartesian limits and self-collision protection
 * stay active, and motions run at a capped speed.
 *
 * Commands:
 *   state                         print current joint angles + tool pose (read only)
 *   home                          run the robot's stored "Home" action
 *   joints j1 j2 j3 j4 j5 j6 j7    move to a joint configuration (degrees)
 *   pose x y z tx ty tz           move tool to a Cartesian pose (m, degrees)
 *   gripper v                     set gripper (0.0 = open ... 1.0 = closed)
 *
 * Options (anywhere on the line):
 *   --ip ADDR        arm IP (default 192.168.1.10)
 *   --speed DEG/S    max joint speed for `joints`/`home` (default 20)
 *   --lin M/S        max tool speed for `pose` (default 0.10)
 *
 * Build:  see basic_control/CMakeLists.txt
 * --------------------------------------------------------------------------
 */

#include <iostream>
#include <string>
#include <vector>
#include <future>
#include <chrono>
#include <cstdlib>

#include <BaseClientRpc.h>
#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientTcp.h>

namespace k_api = Kinova::Api;

#define PORT 10000
#define ACTUATOR_COUNT 7

// ---------------------------------------------------------------------------
// Action-notification helper: resolve a promise when the action ends/aborts.
// ---------------------------------------------------------------------------
static std::function<void(k_api::Base::ActionNotification)>
make_action_listener(std::promise<k_api::Base::ActionEvent>& done)
{
    return [&done](k_api::Base::ActionNotification n) {
        const auto e = n.action_event();
        if (e == k_api::Base::ActionEvent::ACTION_END ||
            e == k_api::Base::ActionEvent::ACTION_ABORT) {
            done.set_value(e);
        }
    };
}

// Subscribe, fire `trigger`, block until the arm reports the motion finished.
static bool run_action(k_api::Base::BaseClient* base,
                       const std::function<void()>& trigger,
                       int timeout_s = 60)
{
    std::promise<k_api::Base::ActionEvent> done;
    auto fut = done.get_future();
    auto handle = base->OnNotificationActionTopic(
        make_action_listener(done), k_api::Common::NotificationOptions());

    trigger();

    const auto status = fut.wait_for(std::chrono::seconds(timeout_s));
    base->Unsubscribe(handle);

    if (status != std::future_status::ready) {
        std::cerr << "  ! timed out after " << timeout_s << "s\n";
        return false;
    }
    const bool ok = fut.get() == k_api::Base::ActionEvent::ACTION_END;
    std::cout << (ok ? "  motion complete.\n" : "  ! motion ABORTED by robot.\n");
    return ok;
}

// ---------------------------------------------------------------------------
static void print_state(k_api::Base::BaseClient* base)
{
    auto a = base->GetMeasuredJointAngles();
    std::cout << "Joint angles (deg): ";
    for (int i = 0; i < a.joint_angles_size(); ++i)
        std::cout << a.joint_angles(i).value() << (i + 1 < a.joint_angles_size() ? ", " : "\n");

    auto p = base->GetMeasuredCartesianPose();
    std::cout << "Tool pose: x=" << p.x() << " y=" << p.y() << " z=" << p.z()
              << " m | thetaX=" << p.theta_x() << " thetaY=" << p.theta_y()
              << " thetaZ=" << p.theta_z() << " deg\n";
}

static bool move_to_joints(k_api::Base::BaseClient* base,
                           const std::vector<double>& deg, double speed_deg_s)
{
    if (deg.size() != ACTUATOR_COUNT) {
        std::cerr << "joints: need exactly " << ACTUATOR_COUNT << " angles\n";
        return false;
    }
    k_api::Base::Action action;
    action.set_name("basic_control joints");
    auto* reach = action.mutable_reach_joint_angles();
    auto* angles = reach->mutable_joint_angles();
    for (int i = 0; i < ACTUATOR_COUNT; ++i) {
        auto* ja = angles->add_joint_angles();
        ja->set_joint_identifier(i);
        ja->set_value(static_cast<float>(deg[i]));
    }
    // Cap the per-joint speed so the first runs are gentle.
    auto* c = reach->mutable_constraint();
    c->set_type(k_api::Base::JointTrajectoryConstraintType::JOINT_CONSTRAINT_SPEED);
    c->set_value(static_cast<float>(speed_deg_s));

    std::cout << "Moving to joint target (max " << speed_deg_s << " deg/s)...\n";
    return run_action(base, [&] { base->ExecuteAction(action); });
}

static bool move_to_pose(k_api::Base::BaseClient* base,
                         double x, double y, double z,
                         double tx, double ty, double tz, double lin_m_s)
{
    k_api::Base::Action action;
    action.set_name("basic_control pose");
    auto* reach = action.mutable_reach_pose();
    auto* pose = reach->mutable_target_pose();
    pose->set_x(static_cast<float>(x));
    pose->set_y(static_cast<float>(y));
    pose->set_z(static_cast<float>(z));
    pose->set_theta_x(static_cast<float>(tx));
    pose->set_theta_y(static_cast<float>(ty));
    pose->set_theta_z(static_cast<float>(tz));
    // Cap tool speed for gentle Cartesian motion.
    auto* speed = reach->mutable_constraint()->mutable_speed();
    speed->set_translation(static_cast<float>(lin_m_s)); // m/s
    speed->set_orientation(30.0f);                       // deg/s

    std::cout << "Moving tool to pose (max " << lin_m_s << " m/s)...\n";
    return run_action(base, [&] { base->ExecuteAction(action); });
}

static bool move_home(k_api::Base::BaseClient* base)
{
    k_api::Base::RequestedActionType req;
    req.set_action_type(k_api::Base::ActionType::REACH_JOINT_ANGLES);
    auto list = base->ReadAllActions(req);
    for (int i = 0; i < list.action_list_size(); ++i) {
        const auto& act = list.action_list(i);
        if (act.name() == "Home") {
            std::cout << "Running stored 'Home' action...\n";
            return run_action(base, [&] { base->ExecuteActionFromReference(act.handle()); });
        }
    }
    std::cerr << "No stored 'Home' action found on this arm.\n";
    return false;
}

static bool set_gripper(k_api::Base::BaseClient* base, double value)
{
    if (value < 0.0) value = 0.0;
    if (value > 1.0) value = 1.0;
    k_api::Base::GripperCommand cmd;
    cmd.set_mode(k_api::Base::GripperMode::GRIPPER_POSITION);
    auto* finger = cmd.mutable_gripper()->add_finger();
    finger->set_finger_identifier(1);
    finger->set_value(static_cast<float>(value));
    std::cout << "Setting gripper to " << value << " (0=open, 1=closed)...\n";
    base->SendGripperCommand(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    return true;
}

// ---------------------------------------------------------------------------
static void usage()
{
    std::cout <<
      "Usage: basic_control <command> [args] [--ip ADDR] [--speed DEG/S] [--lin M/S]\n"
      "  state\n"
      "  home\n"
      "  joints j1 j2 j3 j4 j5 j6 j7   (degrees)\n"
      "  pose x y z tx ty tz           (metres, degrees)\n"
      "  gripper v                     (0.0 open .. 1.0 closed)\n";
}

int main(int argc, char** argv)
{
    std::string ip = "192.168.1.10";
    double speed = 20.0;   // deg/s for joint moves
    double lin   = 0.10;   // m/s for Cartesian moves

    // Split options from positional args.
    std::vector<std::string> pos;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--ip" && i + 1 < argc)         ip = argv[++i];
        else if (a == "--speed" && i + 1 < argc) speed = std::atof(argv[++i]);
        else if (a == "--lin" && i + 1 < argc)   lin = std::atof(argv[++i]);
        else if (a == "-h" || a == "--help")     { usage(); return 0; }
        else                                     pos.push_back(a);
    }
    const std::string cmd = pos.empty() ? "state" : pos[0];

    // --- connect (TCP, high-level only) ---
    auto err = [](k_api::KError e){ std::cout << "API error: " << e.toString() << "\n"; };
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, err);
    transport->connect(ip, PORT);

    auto sinfo = k_api::Session::CreateSessionInfo();
    sinfo.set_username("admin");
    sinfo.set_password("admin");
    sinfo.set_session_inactivity_timeout(60000);
    sinfo.set_connection_inactivity_timeout(2000);
    auto session = new k_api::SessionManager(router);
    session->CreateSession(sinfo);

    auto base = new k_api::Base::BaseClient(router);
    std::cout << "Connected to arm at " << ip << ".\n";

    int rc = 0;
    try {
        base->ClearFaults();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Ensure supervised high-level control (limits + collision protection on).
        k_api::Base::ServoingModeInformation servoing;
        servoing.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
        base->SetServoingMode(servoing);

        std::cout << "--- current state ---\n";
        print_state(base);

        bool ok = true;
        if (cmd == "state") {
            // already printed
        } else if (cmd == "home") {
            ok = move_home(base);
        } else if (cmd == "joints") {
            std::vector<double> q;
            for (size_t i = 1; i < pos.size(); ++i) q.push_back(std::atof(pos[i].c_str()));
            ok = move_to_joints(base, q, speed);
        } else if (cmd == "pose") {
            if (pos.size() != 7) { std::cerr << "pose: need x y z tx ty tz\n"; ok = false; }
            else ok = move_to_pose(base, std::atof(pos[1].c_str()), std::atof(pos[2].c_str()),
                                    std::atof(pos[3].c_str()), std::atof(pos[4].c_str()),
                                    std::atof(pos[5].c_str()), std::atof(pos[6].c_str()), lin);
        } else if (cmd == "gripper") {
            if (pos.size() != 2) { std::cerr << "gripper: need one value 0..1\n"; ok = false; }
            else ok = set_gripper(base, std::atof(pos[1].c_str()));
        } else {
            usage(); ok = false;
        }

        if (cmd != "state") {
            std::cout << "--- state after ---\n";
            print_state(base);
        }
        rc = ok ? 0 : 1;
    } catch (k_api::KDetailedException& e) {
        std::cerr << "Kortex exception: " << e.what() << "\n";
        rc = 2;
    } catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
        rc = 2;
    }

    // --- cleanup ---
    session->CloseSession();
    router->SetActivationStatus(false);
    transport->disconnect();
    delete base;
    delete session;
    delete router;
    delete transport;
    return rc;
}

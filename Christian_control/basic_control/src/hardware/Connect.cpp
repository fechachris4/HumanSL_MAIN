//
// Connect: the two Kortex sessions to the arm (see Connect.h).
//

#include "hardware/Connect.h"

#include "app/Config.h"

#include <iostream>
#include <stdexcept>

#include <TransportClientTcp.h>
#include <TransportClientUdp.h>

namespace
{

    // The Gen3 base always listens on these two ports (fixed by the robot's
    // firmware, not a choice of ours — so not in Config.h).
    constexpr unsigned kTcpPort = 10000; // configuration + high-level commands
    constexpr unsigned kUdpPort = 10001; // 1 kHz low-level cyclic streaming

} // namespace

Connect::Channel::Channel(std::unique_ptr<k_api::ITransportClient> transport_in,
                          const std::string& ip_address, unsigned port, const char* label)
    : transport(std::move(transport_in))
{
    // connect() reports failure by return value, not by throwing — check it,
    // or an unreachable arm only fails later, inside CreateSession, with a
    // much less helpful error.
    if (!transport->connect(ip_address, port))
        throw std::runtime_error(std::string("could not reach the arm at ") + ip_address + ":" +
                                 std::to_string(port) + " (" + label + " channel)");

    router = std::make_unique<k_api::RouterClient>(transport.get(), [](k_api::KError err) {
        std::cout << "Kortex API error: " << err.toString() << std::endl;
    });

    auto session_info = k_api::Session::CreateSessionInfo();
    session_info.set_username(config::kSessionUsername);
    session_info.set_password(config::kSessionPassword);
    session_info.set_session_inactivity_timeout(config::kSessionInactivityTimeoutMs);
    session_info.set_connection_inactivity_timeout(config::kConnectionInactivityTimeoutMs);

    session = std::make_unique<k_api::SessionManager>(router.get());
    session->CreateSession(session_info);
}

Connect::Channel::~Channel()
{
    // CloseSession talks to the robot, so it can throw if the link is
    // already dead — and a throwing destructor would abort the program.
    try {
        session->CloseSession();
    } catch (...) {
        std::cerr << "warning: failed to close a Kortex session cleanly\n";
    }
    router->SetActivationStatus(false);
    transport->disconnect();
}

Connect::Connect(const std::string& ip_address)
    : tcp_(std::make_unique<k_api::TransportClientTcp>(), ip_address, kTcpPort, "TCP"),
      udp_(std::make_unique<k_api::TransportClientUdp>(), ip_address, kUdpPort, "UDP"),
      base_(std::make_unique<k_api::Base::BaseClient>(tcp_.router.get())),
      actuator_config_(std::make_unique<k_api::ActuatorConfig::ActuatorConfigClient>(
          tcp_.router.get())),
      base_cyclic_(std::make_unique<k_api::BaseCyclic::BaseCyclicClient>(udp_.router.get()))
{
    std::cout << "Connected to arm at " << ip_address << " (TCP + real-time UDP).\n";
}

bool Connect::EnsurePositionControlModes(std::ostream& out)
{
    auto desired = k_api::ActuatorConfig::ControlModeInformation();
    desired.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
    bool changed = false;
    for (std::uint32_t id = 1; id <= 7; ++id)
    {
        const auto before = actuator_config_->GetControlMode(id);
        if (before.control_mode() != k_api::ActuatorConfig::ControlMode::POSITION)
        {
            out << "joint " << id << " control mode was "
                << k_api::ActuatorConfig::ControlMode_Name(before.control_mode())
                << "; setting POSITION\n";
            actuator_config_->SetControlMode(desired, id);
            changed = true;
        }
        const auto verified = actuator_config_->GetControlMode(id);
        if (verified.control_mode() != k_api::ActuatorConfig::ControlMode::POSITION)
        {
            out << "robot NOT ready: joint " << id
                << " did not enter POSITION control mode\n";
            return false;
        }
    }
    out << "actuator control-mode gate: PASS (all joints POSITION"
        << (changed ? ", corrections verified" : "") << ")\n";
    return true;
}

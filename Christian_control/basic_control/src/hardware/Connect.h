//
// Connect: the two Kortex sessions to the arm, opened on construction and
// closed on destruction (RAII), on every exit path including exceptions.
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_CONNECT_H
#define HUMANSL_MASTERS_PROJECT_2025_CONNECT_H

#include <memory>
#include <ostream>
#include <string>

#include <RouterClient.h>
#include <ITransportClient.h>
#include <SessionManager.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>

namespace k_api = Kinova::Api;

// Opens two sessions to the arm on construction, cleans up on destruction:
//  - TCP (port 10000): configuration + high-level commands  -> base()
//  - UDP (port 10001): 1 kHz low-level cyclic streaming     -> base_cyclic()
//
// Throws std::runtime_error if the arm cannot be reached. If opening the
// second channel fails, the first is closed properly before the exception
// leaves the constructor (C++ destroys fully-constructed members on the way
// out), so no session is ever left open on the robot.
class Connect
{
public:
    explicit Connect(const std::string& ip_address);

    k_api::Base::BaseClient* base()
    {
        return base_.get();
    }
    k_api::BaseCyclic::BaseCyclicClient* base_cyclic()
    {
        return base_cyclic_.get();
    }

    // Read, set if necessary, and re-read all seven actuator control modes.
    // Must pass before the low-level takeover begins.
    bool EnsurePositionControlModes(std::ostream& out);

private:
    // One connected transport + router + logged-in session. Members are
    // declared socket -> router -> session, so destruction (reverse order)
    // logs out first and closes the socket last.
    struct Channel {
        // Takes ownership of an unconnected transport (TCP or UDP — that is
        // the only difference between the two channels), connects it, and
        // logs in. `label` names the channel in error messages.
        Channel(std::unique_ptr<k_api::ITransportClient> transport_in,
                const std::string& ip_address, unsigned port, const char* label);
        ~Channel();

        std::unique_ptr<k_api::ITransportClient> transport;
        std::unique_ptr<k_api::RouterClient> router;
        std::unique_ptr<k_api::SessionManager> session;
    };

    Channel tcp_; // port 10000: configuration + high-level commands
    Channel udp_; // port 10001: real-time cyclic channel

    std::unique_ptr<k_api::Base::BaseClient> base_;
    std::unique_ptr<k_api::ActuatorConfig::ActuatorConfigClient> actuator_config_;
    std::unique_ptr<k_api::BaseCyclic::BaseCyclicClient> base_cyclic_;
};

#endif // HUMANSL_MASTERS_PROJECT_2025_CONNECT_H

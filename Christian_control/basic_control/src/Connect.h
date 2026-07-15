//
// Created by christian on 7/10/26.
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_CONNECT_H
#define HUMANSL_MASTERS_PROJECT_2025_CONNECT_H

#include <string>

#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>
#include <SessionManager.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>

namespace k_api = Kinova::Api;

// Opens two sessions to the arm on construction, cleans up on destruction:
//  - TCP (port 10000): configuration + high-level commands  -> base()
//  - UDP (port 10001): 1 kHz low-level cyclic streaming     -> base_cyclic()
class Connect
{
public:
    explicit Connect(const std::string& ip_address);
    ~Connect();

    k_api::Base::BaseClient* base() { return base_; }
    k_api::BaseCyclic::BaseCyclicClient* base_cyclic() { return base_cyclic_; }

    // The TCP router, for building extra service clients (e.g. ControlConfig)
    // on the same session.
    k_api::RouterClient* router() { return router_; }

private:
    // TCP: high-level / configuration
    k_api::TransportClientTcp* transport_;
    k_api::RouterClient* router_;
    k_api::SessionManager* session_;
    k_api::Base::BaseClient* base_;

    // UDP: real-time cyclic channel
    k_api::TransportClientUdp* transport_rt_;
    k_api::RouterClient* router_rt_;
    k_api::SessionManager* session_rt_;
    k_api::BaseCyclic::BaseCyclicClient* base_cyclic_;
};

#endif //HUMANSL_MASTERS_PROJECT_2025_CONNECT_H

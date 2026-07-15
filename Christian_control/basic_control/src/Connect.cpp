//
// Created by christian on 7/10/26.
//

#include "Connect.h"

#include <iostream>

Connect::Connect(const std::string& ip_address)
{
    auto error_callback = [](k_api::KError err) {
        std::cout << "Kortex API error: " << err.toString() << std::endl;
    };

    // Both sessions log in with the same credentials.
    auto session_info = k_api::Session::CreateSessionInfo();
    session_info.set_username("admin");
    session_info.set_password("admin");
    session_info.set_session_inactivity_timeout(60000);
    session_info.set_connection_inactivity_timeout(2000);

    // --- TCP channel (port 10000): configuration + high-level commands ---
    transport_ = new k_api::TransportClientTcp();
    router_ = new k_api::RouterClient(transport_, error_callback);
    transport_->connect(ip_address, 10000);

    session_ = new k_api::SessionManager(router_);
    session_->CreateSession(session_info);

    base_ = new k_api::Base::BaseClient(router_);

    // --- UDP channel (port 10001): 1 kHz low-level cyclic streaming ---
    transport_rt_ = new k_api::TransportClientUdp();
    router_rt_ = new k_api::RouterClient(transport_rt_, error_callback);
    transport_rt_->connect(ip_address, 10001);

    session_rt_ = new k_api::SessionManager(router_rt_);
    session_rt_->CreateSession(session_info);

    base_cyclic_ = new k_api::BaseCyclic::BaseCyclicClient(router_rt_);

    std::cout << "Connected to arm at " << ip_address
              << " (TCP + real-time UDP).\n";
}

Connect::~Connect()
{
    session_rt_->CloseSession();
    router_rt_->SetActivationStatus(false);
    transport_rt_->disconnect();

    session_->CloseSession();
    router_->SetActivationStatus(false);
    transport_->disconnect();

    delete base_cyclic_;
    delete session_rt_;
    delete router_rt_;
    delete transport_rt_;

    delete base_;
    delete session_;
    delete router_;
    delete transport_;
}

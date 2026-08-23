#include <future>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "Hardware.h"

namespace
{

    int failures = 0;

    void Check(bool condition, const char* description)
    {
        if (!condition) {
            std::cout << "FAIL: " << description << "\n";
            ++failures;
        }
    }

    enum class RefreshFailure {
        kRuntimeError,
        kDetailedNonTimeout,
    };

    class TimeoutRouter final : public k_api::IRouterClient
    {
    public:
        void FailRefreshes(RefreshFailure failure)
        {
            fail_refreshes_ = true;
            failure_ = failure;
        }

        void reset() override {}
        void registerBridgingCallback(std::function<void(k_api::Frame&)>) override {}
        void registerNotificationCallback(std::uint32_t,
                                          std::function<k_api::Error(k_api::Frame&)>) override
        {
        }
        void registerErrorCallback(std::function<void(k_api::KError)>) override {}
        void registerHitCallback(std::function<void(k_api::FrameTypes)>) override {}

        std::future<k_api::Frame> send(const std::string& payload, std::uint32_t, std::uint32_t,
                                       std::uint32_t,
                                       const k_api::RouterClientSendOptions&) override
        {
            std::promise<k_api::Frame> reply;
            if (fail_refreshes_) {
                k_api::BaseCyclic::Command command;
                Check(command.ParseFromString(payload), "router received cyclic command");
                refresh_commands.push_back(command);
                if (failure_ == RefreshFailure::kDetailedNonTimeout) {
                    reply.set_exception(
                        std::make_exception_ptr(k_api::KDetailedException(k_api::KError(
                            k_api::ERROR_INTERNAL, k_api::METHOD_FAILED, "test cyclic failure"))));
                    return reply.get_future();
                }
                reply.set_exception(
                    std::make_exception_ptr(std::runtime_error("test cyclic timeout")));
                return reply.get_future();
            }

            k_api::BaseCyclic::Feedback feedback;
            std::string feedback_payload;
            Check(feedback.SerializeToString(&feedback_payload), "router serializes seed feedback");
            k_api::Frame frame;
            frame.set_payload(feedback_payload);
            reply.set_value(std::move(frame));
            return reply.get_future();
        }

        k_api::Error sendWithCallback(const std::string&, std::uint32_t, std::uint32_t,
                                      std::uint32_t, k_api::MessageCallback) override
        {
            return {};
        }
        k_api::Error sendMsgFrame(const k_api::Frame&) override
        {
            return {};
        }
        std::uint16_t getConnectionId() override
        {
            return 0;
        }
        void SetActivationStatus(bool) override {}
        k_api::ITransportClient* getTransport() override
        {
            return nullptr;
        }

        std::vector<k_api::BaseCyclic::Command> refresh_commands;

    private:
        bool fail_refreshes_ = false;
        RefreshFailure failure_ = RefreshFailure::kRuntimeError;
    };

    void CheckRuntimeErrorRetriesOnceThenPropagates()
    {
        TimeoutRouter router;
        k_api::BaseCyclic::BaseCyclicClient client(&router);
        CyclicSession session(&client);
        session.Seed();
        router.FailRefreshes(RefreshFailure::kRuntimeError);

        const JointVector setpoints_deg{{10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0}};

        bool propagated = false;
        try {
            session.Send(setpoints_deg);
        } catch (const std::runtime_error&) {
            propagated = true;
        }

        Check(propagated, "second runtime error propagates");
        Check(router.refresh_commands.size() == 2,
              "runtime error retries the unchanged cyclic command exactly once");
        if (router.refresh_commands.size() == 2) {
            Check(router.refresh_commands[0].SerializeAsString() ==
                      router.refresh_commands[1].SerializeAsString(),
                  "retry preserves the complete cyclic command");
        }
    }

    void CheckDetailedNonTimeoutPropagatesWithoutRetry()
    {
        TimeoutRouter router;
        k_api::BaseCyclic::BaseCyclicClient client(&router);
        CyclicSession session(&client);
        session.Seed();
        router.FailRefreshes(RefreshFailure::kDetailedNonTimeout);

        const JointVector setpoints_deg{{10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0}};

        bool propagated = false;
        try {
            session.Send(setpoints_deg);
        } catch (const k_api::KDetailedException&) {
            propagated = true;
        }

        Check(propagated, "non-timeout detailed error propagates");
        Check(router.refresh_commands.size() == 1,
              "non-timeout detailed error does not retry the cyclic command");
    }

} // namespace

int main()
{
    CheckRuntimeErrorRetriesOnceThenPropagates();
    CheckDetailedNonTimeoutPropagatesWithoutRetry();
    return failures == 0 ? 0 : 1;
}

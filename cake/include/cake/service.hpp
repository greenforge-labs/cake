#pragma once

#include <functional>
#include <memory>
#include <type_traits>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "session.hpp"

namespace cake {

template <typename ServiceT, typename SessionType> class Service {
    static_assert(std::is_base_of_v<Session, SessionType>, "SessionType must derive from cake::Session");

  public:
    explicit Service(
        std::shared_ptr<SessionType> sn, const std::string &service_name, const rclcpp::QoS &qos = rclcpp::ServicesQoS()
    ) {
        set_request_handler([service_name](auto sn, auto /*req*/, auto /*res*/) {
            RCLCPP_WARN(
                sn->node.get_logger(),
                "Service '%s' received request but no handler configured. Call set_request_handler().",
                service_name.c_str()
            );
        });

        std::weak_ptr<SessionType> weak_sn = sn;
        service_ = sn->node.template create_service<ServiceT>(
            service_name,
            [weak_sn, this, service_name](
                const std::shared_ptr<typename ServiceT::Request> request,
                std::shared_ptr<typename ServiceT::Response> response
            ) {
                auto sn = weak_sn.lock();
                if (!sn)
                    return;
                if (sn->node.get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                    RCLCPP_WARN(sn->node.get_logger(), "Service '%s': rejected, node not active", service_name.c_str());
                    return;
                }
                request_handler_(sn, request, response);
            },
            qos
        );
    }

    void set_request_handler(
        std::function<
            void(std::shared_ptr<SessionType>, const std::shared_ptr<typename ServiceT::Request>, std::shared_ptr<typename ServiceT::Response>)>
            handler
    ) {
        request_handler_ = handler;
    }

  private:
    typename rclcpp::Service<ServiceT>::SharedPtr service_;
    std::function<
        void(std::shared_ptr<SessionType>, const std::shared_ptr<typename ServiceT::Request>, std::shared_ptr<typename ServiceT::Response>)>
        request_handler_;
};

template <typename ServiceT, typename SessionType>
std::shared_ptr<Service<ServiceT, SessionType>> create_service(
    std::shared_ptr<SessionType> sn, const std::string &service_name, const rclcpp::QoS &qos = rclcpp::ServicesQoS()
) {
    return std::make_shared<Service<ServiceT, SessionType>>(sn, service_name, qos);
}

} // namespace cake

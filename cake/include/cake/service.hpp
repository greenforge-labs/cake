#pragma once

#include <functional>
#include <memory>
#include <type_traits>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "context.hpp"

namespace cake {

template <typename ServiceT, typename ContextType> class Service {
    static_assert(std::is_base_of_v<Context, ContextType>, "ContextType must derive from cake::Context");

  public:
    explicit Service(
        std::shared_ptr<ContextType> context,
        const std::string &service_name,
        const rclcpp::QoS &qos = rclcpp::ServicesQoS()
    ) {
        set_request_handler([service_name](auto ctx, auto /*req*/, auto /*res*/) {
            RCLCPP_WARN(
                ctx->node->get_logger(),
                "Service '%s' received request but no handler configured. Call set_request_handler().",
                service_name.c_str()
            );
        });

        std::weak_ptr<ContextType> weak_ctx = context;
        service_ = context->node->template create_service<ServiceT>(
            service_name,
            [weak_ctx, this, service_name](
                const std::shared_ptr<typename ServiceT::Request> request,
                std::shared_ptr<typename ServiceT::Response> response
            ) {
                auto ctx = weak_ctx.lock();
                if (!ctx)
                    return;
                if (ctx->node->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                    RCLCPP_WARN(
                        ctx->node->get_logger(), "Service '%s': rejected, node not active", service_name.c_str()
                    );
                    return;
                }
                request_handler_(ctx, request, response);
            },
            qos
        );
    }

    void set_request_handler(
        std::function<
            void(std::shared_ptr<ContextType>, const std::shared_ptr<typename ServiceT::Request>, std::shared_ptr<typename ServiceT::Response>)>
            handler
    ) {
        request_handler_ = handler;
    }

  private:
    typename rclcpp::Service<ServiceT>::SharedPtr service_;
    std::function<
        void(std::shared_ptr<ContextType>, const std::shared_ptr<typename ServiceT::Request>, std::shared_ptr<typename ServiceT::Response>)>
        request_handler_;
};

template <typename ServiceT, typename ContextType>
std::shared_ptr<Service<ServiceT, ContextType>> create_service(
    std::shared_ptr<ContextType> context,
    const std::string &service_name,
    const rclcpp::QoS &qos = rclcpp::ServicesQoS()
) {
    return std::make_shared<Service<ServiceT, ContextType>>(context, service_name, qos);
}

} // namespace cake

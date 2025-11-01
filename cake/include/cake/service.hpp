#pragma once

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace cake {

template <typename ServiceT, typename ContextType> class Service {
  public:
    explicit Service(
        std::shared_ptr<ContextType> context,
        const std::string &service_name,
        const rclcpp::QoS &qos = rclcpp::ServicesQoS()
    ) {
        service_ = context->node->template create_service<ServiceT>(
            service_name,
            [context, this](
                const std::shared_ptr<typename ServiceT::Request> request,
                std::shared_ptr<typename ServiceT::Response> response
            ) { request_handler_(context, request, response); },
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
        request_handler_ = [](auto /*ctx*/, auto /*req*/, auto /*res*/) {};
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

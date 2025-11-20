from rclpy.qos import QoSProfile, qos_profile_services_default
from rclpy.service import Service as RclpyService

from .context import Context

from typing import Any, Callable, Generic, TypeVar, cast

ServiceT = TypeVar("ServiceT")
RequestT = TypeVar("RequestT")
ResponseT = TypeVar("ResponseT")


def get_no_handler_warning_logger(service_name: str) -> Callable[[Context, Any, ResponseT], ResponseT]:
    def inner(ctx: Context, request: Any, response: ResponseT) -> ResponseT:
        ctx.node.get_logger().warning(
            f"Service '{service_name}' received request but no handler configured. Call set_request_handler()."
        )
        return response

    return inner


class Service(Generic[ServiceT, RequestT, ResponseT]):
    _service: RclpyService | None = None
    _request_handler: Callable[[Context, RequestT, ResponseT], ResponseT]
    _service_type: type[ServiceT]
    _service_name: str

    def _initialise(
        self,
        context: Context,
        srv_type: type[ServiceT],
        service_name: str,
        qos: QoSProfile = qos_profile_services_default,
    ) -> None:
        self._service_type = srv_type
        self._service_name = service_name
        self._request_handler = get_no_handler_warning_logger(service_name)

        self._service = context.node.create_service(
            srv_type=srv_type,
            srv_name=service_name,
            callback=lambda request, response: self._request_handler(
                context, cast(RequestT, request), cast(ResponseT, response)
            ),
            qos_profile=qos,
        )

    def set_request_handler(self, handler: Callable[[Context, RequestT, ResponseT], ResponseT]) -> None:
        if self._service is None:
            raise RuntimeError("Can't set request handler. Service has not been initialised! This is an error in cake.")
        self._request_handler = handler


__all__ = ["Service"]

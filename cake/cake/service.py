from rclpy.lifecycle import State
from rclpy.qos import QoSProfile, qos_profile_services_default
from rclpy.service import Service as RclpyService

from .session import Session

from typing import Any, Callable, Generic, TypeVar, cast

ServiceT = TypeVar("ServiceT")
RequestT = TypeVar("RequestT")
ResponseT = TypeVar("ResponseT")


def get_no_handler_warning_logger(service_name: str) -> Callable[[Session, Any, ResponseT], ResponseT]:
    def inner(sn: Session, request: Any, response: ResponseT) -> ResponseT:
        sn.node.get_logger().warning(
            f"Service '{service_name}' received request but no handler configured. Call set_request_handler()."
        )
        return response

    return inner


class Service(Generic[ServiceT, RequestT, ResponseT]):
    _service: RclpyService | None = None
    _request_handler: Callable[[Session, RequestT, ResponseT], ResponseT]
    _service_type: type[ServiceT]
    _service_name: str

    def _initialise(
        self,
        session: Session,
        srv_type: type[ServiceT],
        service_name: str,
        qos: QoSProfile = qos_profile_services_default,
    ) -> None:
        self._service_type = srv_type
        self._service_name = service_name
        self._request_handler = get_no_handler_warning_logger(service_name)

        def guarded_handler(request, response):
            if session.node.current_state != State.PRIMARY_STATE_ACTIVE:
                session.node.get_logger().warning(
                    f"Service '{service_name}' received request while not active, ignoring."
                )
                return response
            return self._request_handler(session, cast(RequestT, request), cast(ResponseT, response))

        self._service = session.node.create_service(
            srv_type=srv_type,
            srv_name=service_name,
            callback=guarded_handler,
            qos_profile=qos,
        )

    def _destroy(self, node) -> None:
        if self._service is not None:
            node.destroy_service(self._service)
            self._service = None

    def set_request_handler(self, handler: Callable[[Session, RequestT, ResponseT], ResponseT]) -> None:
        if self._service is None:
            raise RuntimeError("Can't set request handler. Service has not been initialised! This is an error in cake.")
        self._request_handler = handler


__all__ = ["Service"]

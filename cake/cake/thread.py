import threading

from .context import Context

from typing import Callable, TypeVar

ContextT = TypeVar("ContextT", bound=Context)


def create_thread(context: ContextT, func: Callable[[ContextT], None], daemon: bool = True):
    thread = threading.Thread(target=func, args=(context,), daemon=daemon)
    thread.start()
    context.threads.append(thread)


__all__ = ["create_thread"]

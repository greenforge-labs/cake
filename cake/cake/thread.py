import threading

from .context import Context

from typing import Callable, Any


def create_thread(context: Context, func: Callable[[Any], None]):
    thread = threading.Thread(target=func, args=(context,))
    thread.start()
    context.threads.append(thread)


__all__ = ["create_thread"]

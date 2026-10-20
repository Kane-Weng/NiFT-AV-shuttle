"""The exception classes of python-socketio 5.x (socketio/exceptions.py), same names and bases."""


class SocketIOError(Exception):
    pass


class ConnectionError(SocketIOError):  # noqa: A001 (python-socketio's own name)
    pass


class ConnectionRefusedError(ConnectionError):  # noqa: A001
    """Raised by a server connect handler to refuse a client (python-socketio's signature)."""

    def __init__(self, *args):
        super().__init__(*args)
        if len(args) == 0:
            self.error_args = {'message': 'Connection rejected by server'}
        elif len(args) == 1:
            self.error_args = {'message': str(args[0])}
        else:
            self.error_args = {'message': str(args[0])}
            self.error_args['data'] = args[1] if len(args) == 2 else args[1:]


class TimeoutError(SocketIOError):  # noqa: A001
    pass


class BadNamespaceError(SocketIOError):
    pass


class DisconnectedError(SocketIOError):
    pass

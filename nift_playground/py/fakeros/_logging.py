"""
rclpy's logger, on the world's log sink.

File: fakeros/_logging.py

Description:
  As rclpy's RcutilsLogger in Humble.
  - debug / info / warning / warn / error / fatal return True when the message was logged.
  - The level is checked first. Then every call site, which is (function, file, line, bytecode
    offset) of the caller, per logger object, keeps its severity, name, filters and filter
    parameters. Changing one of them at the same site raises ValueError, as in rclpy.
  - Filters run in rclpy's order (throttle_duration_sec, skip_first, once) and the first that
    says no stops the others.
  - A logger's level comes from set_level, else from rclpy.logging.set_logger_level for its name,
    else from world.log_level (INFO by default).
  Two deliberate differences from Humble.
  - Throttling runs on the world (sim) clock. Humble uses the wall clock by default, which would
    make the logs differ between runs. The first call at a site always logs, as with the wall
    clock.
  - world.reset() forgets every filter state, so a module-level logger (can_interface.logger)
    behaves after a reset as in a new process.
  A logged line goes to world.log_sink(level, name, text) with level 10..50, else to stderr in the
  ROS console format.
"""
import enum
import sys


class LoggingSeverity(enum.IntEnum):
    UNSET = 0
    DEBUG = 10
    INFO = 20
    WARN = 30
    ERROR = 40
    FATAL = 50


_LEVELS = {}            # logger name -> severity set with set_logger_level


def _world():
    from ._world import world
    return world


def _caller():
    """The first frame outside fakeros' logging code, as rclpy's CallerId."""
    f = sys._getframe(1)
    while f is not None and f.f_globals.get('__name__') == __name__:
        f = f.f_back
    if f is None:
        return ('', '', 0, 0)
    return (f.f_code.co_name, f.f_code.co_filename, f.f_lineno, f.f_lasti)


# filter -> its parameters and defaults, in the order rclpy applies the filters
_FILTERS = {'throttle': {'throttle_duration_sec': None}, 'skip_first': {'skip_first': None},
            'once': {'once': None}}
_ALL_PARAMS = ('throttle_duration_sec', 'throttle_time_source_type', 'skip_first', 'once')


class RcutilsLogger:
    def __init__(self, name=''):
        self.name = name
        self.contexts = {}
        self._epoch = None
        self._level = LoggingSeverity.UNSET

    def get_child(self, name):
        if not name:
            raise ValueError('Child logger name must not be empty.')
        return RcutilsLogger(f'{self.name}.{name}' if self.name else name)

    def set_level(self, level):
        self._level = LoggingSeverity(level)

    def get_effective_level(self):
        if self._level != LoggingSeverity.UNSET:
            return self._level
        lv = _LEVELS.get(self.name)
        if lv:
            return LoggingSeverity(lv)
        return LoggingSeverity(_world().log_level)

    def is_enabled_for(self, severity):
        return severity >= self.get_effective_level()

    def log(self, message, severity, **kwargs):
        if not self.is_enabled_for(severity):          # filters only run for enabled severities
            return False
        severity = LoggingSeverity(severity)
        name = kwargs.pop('name', self.name)
        for k in kwargs:
            if k not in _ALL_PARAMS:
                raise TypeError(f'parameter "{k}" is not one of the recognized logging options '
                                f'"{list(_ALL_PARAMS)}"')
        detected = [f for f, params in _FILTERS.items() if any(kwargs.get(p) for p in params)]
        params = {p: kwargs.get(p, default) for f in detected for p, default in _FILTERS[f].items()}
        w = _world()
        if self._epoch != w.epoch:                      # a world reset forgets the call sites
            self.contexts = {}
            self._epoch = w.epoch
        site = _caller()
        ctx = self.contexts.get(site)
        if ctx is None:
            ctx = {'name': name, 'severity': severity, 'filters': detected, 'params': params,
                   'throttle_last': None, 'skipped': False, 'logged_once': False}
            self.contexts[site] = ctx
        else:
            if severity != ctx['severity']:
                raise ValueError('Logger severity cannot be changed between calls.')
            if name != ctx['name']:
                raise ValueError('Logger name cannot be changed between calls.')
            if detected != ctx['filters']:
                raise ValueError('Requested logging filters cannot be changed between calls.')
            if params != ctx['params']:
                raise ValueError('Logging filter parameters cannot be changed between calls.')
        for f in detected:                              # throttle, skip_first, once (rclpy's order)
            if f == 'throttle':
                now = w.now_ns
                last = ctx['throttle_last']
                if last is not None and now < last + params['throttle_duration_sec'] * 1e9:
                    return False
                ctx['throttle_last'] = now
            elif f == 'skip_first':
                if not ctx['skipped']:
                    ctx['skipped'] = True
                    return False
            elif f == 'once':
                if ctx['logged_once']:
                    return False
                ctx['logged_once'] = True
        w.stats['log_' + severity.name.lower()] += 1
        text = str(message)
        if w.log_sink is not None:
            w.log_sink(int(severity), name, text)
        else:
            print(f'[{severity.name}] [{w.now_ns / 1e9:.9f}] [{name}]: {text}', file=sys.stderr)
        return True

    def debug(self, message, **kwargs):
        return self.log(message, LoggingSeverity.DEBUG, **kwargs)

    def info(self, message, **kwargs):
        return self.log(message, LoggingSeverity.INFO, **kwargs)

    def warning(self, message, **kwargs):
        return self.log(message, LoggingSeverity.WARN, **kwargs)

    def warn(self, message, **kwargs):
        return self.log(message, LoggingSeverity.WARN, **kwargs)

    def error(self, message, **kwargs):
        return self.log(message, LoggingSeverity.ERROR, **kwargs)

    def fatal(self, message, **kwargs):
        return self.log(message, LoggingSeverity.FATAL, **kwargs)


Logger = RcutilsLogger


def get_logger(name):
    if not name:
        raise ValueError('Logger name must not be empty.')
    return RcutilsLogger(name)


def set_logger_level(name, level):
    _LEVELS[name] = int(LoggingSeverity(level))


def get_logger_effective_level(name):
    return RcutilsLogger(name).get_effective_level()


def get_logging_severity_from_string(log_severity):
    return LoggingSeverity[log_severity.upper()]


def initialize():
    pass


def shutdown():
    pass


def clear_config():
    _LEVELS.clear()

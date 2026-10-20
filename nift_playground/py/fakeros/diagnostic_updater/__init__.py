"""
diagnostic_updater stand-in, with the Humble package's behaviour.

File: diagnostic_updater/__init__.py

Description:
  Checked against Humble's diagnostic_updater (Python) in nift_pub_deps:humble.
  - Updater(node, period=1.0) declares `diagnostic_updater.period` (a params file may override it)
    and `diagnostic_updater.use_fqn` (False), and runs update() on a node timer of that period.
  - add(name, fn) or add(task) publishes a "Node starting up" OK status for the task at once.
  - update() runs each task on a fresh DiagnosticStatusWrapper (level ERROR, message
    'No message was set', the hardware ID) and publishes what the task returns, named
    "<node name>: <task name>" (the launch name). With no hardware ID set and every status OK it
    warns once, as Humble does.
  - DiagnosticStatusWrapper.add assigns the value as given (a non-string fails the message's type
    check, as on ROS).
"""
import fakeros
fakeros.install()
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue  # noqa: E402

OK = DiagnosticStatus.OK
WARN = DiagnosticStatus.WARN
ERROR = DiagnosticStatus.ERROR
STALE = DiagnosticStatus.STALE


class DiagnosticStatusWrapper(DiagnosticStatus):
    def __init__(self, *args, **kwds):
        DiagnosticStatus.__init__(self, *args, **kwds)

    def summary(self, *args):
        if len(args) == 1:
            self.level = args[0].level
            self.message = args[0].message
        elif len(args) == 2:
            self.level = args[0]
            self.message = str(args[1])

    def clearSummary(self):
        self.summary(DiagnosticStatus.OK, '')

    def mergeSummary(self, *args):
        if len(args) == 1:
            lvl, msg = args[0].level, args[0].message
        else:
            lvl, msg = args[0], args[1]
        if (lvl > DiagnosticStatus.OK) == (self.level > DiagnosticStatus.OK):
            if len(self.message) > 0:
                self.message += '; '
            self.message += msg
        elif lvl > self.level:
            self.message = msg
        if lvl > self.level:
            self.level = lvl

    def add(self, key, val):
        kv = KeyValue()
        kv.key = key
        kv.value = val
        self.values.append(kv)


class DiagnosticTask:
    def __init__(self, name):
        self.name = name

    def getName(self):
        return self.name

    def run(self, stat):
        return stat


class FunctionDiagnosticTask(DiagnosticTask):
    def __init__(self, name, fn):
        DiagnosticTask.__init__(self, name)
        self.fn = fn

    def run(self, stat):
        return self.fn(stat)


class CompositeDiagnosticTask(DiagnosticTask):
    def __init__(self, name):
        DiagnosticTask.__init__(self, name)
        self.tasks = []

    def run(self, stat):
        combined = DiagnosticStatusWrapper()
        original = DiagnosticStatusWrapper()
        original.summary(stat)
        for task in self.tasks:
            stat.summary(original)
            stat = task.run(stat)
            combined.mergeSummary(stat)
        stat.summary(combined)
        return stat

    def addTask(self, t):
        self.tasks.append(t)


class DiagnosticTaskVector:
    class DiagnosticTaskInternal:
        def __init__(self, name, fn):
            self.name = name
            self.fn = fn

        def run(self, stat):
            stat.name = self.name
            return self.fn(stat)

    def __init__(self):
        self.tasks = []

    def addedTaskCallback(self, task):
        pass

    def add(self, *args):
        if len(args) == 1:
            task = DiagnosticTaskVector.DiagnosticTaskInternal(args[0].getName(), args[0].run)
        else:
            task = DiagnosticTaskVector.DiagnosticTaskInternal(args[0], args[1])
        self.tasks.append(task)
        self.addedTaskCallback(task)

    def removeByName(self, name):
        for i, t in enumerate(self.tasks):
            if t.name == name:
                self.tasks.pop(i)
                return True
        return False


class Updater(DiagnosticTaskVector):
    def __init__(self, node, period=1.0, starting_up_status=DiagnosticStatus.OK):
        DiagnosticTaskVector.__init__(self)
        self.node = node
        self.publisher = node.create_publisher(DiagnosticArray, '/diagnostics', 1)
        self.period_parameter = 'diagnostic_updater.period'
        self.__period = node.declare_parameter(self.period_parameter, period).value
        self.timer = node.create_timer(self.__period, self.update)
        self.__starting_up_status = starting_up_status
        self.verbose = False
        self.hwid = ''
        self.warn_nohwid_done = False
        self.use_fqn_parameter = 'diagnostic_updater.use_fqn'
        if node.has_parameter(self.use_fqn_parameter):
            self.__use_fqn = node.get_parameter(self.use_fqn_parameter).value
        else:
            self.__use_fqn = node.declare_parameter(self.use_fqn_parameter, False).value
        if self.__use_fqn:
            self.node_name = '/'.join([node.get_namespace(), node.get_name()])
        else:
            self.node_name = node.get_name()

    def update(self):
        warn_nohwid = len(self.hwid) == 0
        status_vec = []
        for task in self.tasks:
            status = DiagnosticStatusWrapper()
            status.level = DiagnosticStatus.ERROR
            status.name = task.name
            status.message = 'No message was set'
            status.hardware_id = self.hwid
            status = task.run(status)
            status_vec.append(status)
            if status.level != b'\x00':
                warn_nohwid = False
            if self.verbose and status.level != b'\x00':
                self.node.get_logger().warning(
                    'Non-zero diagnostic status. Name: %s, status %s: %s'
                    % (status.name, str(status.level), status.message))
        if warn_nohwid and not self.warn_nohwid_done:
            self.node.get_logger().warning(
                'diagnostic_updater: No HW_ID was set. This is probably a bug. Please report it. '
                'For devices that do not have a HW_ID, set this value to none. This warning only '
                'occurs once all diagnostics are OK so it is okay to wait until the device is open '
                'before calling setHardwareID.')
            self.warn_nohwid_done = True
        self.publish(status_vec)

    @property
    def period(self):
        return self.__period

    @period.setter
    def period(self, period):
        self.__period = period
        self.node.destroy_timer(self.timer)
        self.timer = self.node.create_timer(self.__period, self.update)

    def force_update(self):
        self.update()

    def broadcast(self, lvl, msg):
        status_vec = []
        for task in self.tasks:
            status = DiagnosticStatusWrapper()
            status.name = task.name
            status.summary(lvl, msg)
            status_vec.append(status)
        self.publish(status_vec)

    def setHardwareID(self, hwid):
        self.hwid = hwid

    def publish(self, msg):
        if type(msg) is not list:
            msg = [msg]
        da = DiagnosticArray()
        da.header.stamp = self.node.get_clock().now().to_msg()
        for stat in msg:
            stat.name = self.node_name + ': ' + stat.name
            db = DiagnosticStatus()
            db.name = stat.name
            db.message = stat.message
            db.hardware_id = stat.hardware_id
            db.values = stat.values
            db.level = stat.level
            da.status.append(db)
        self.publisher.publish(da)

    def addedTaskCallback(self, task):
        if self.__starting_up_status is None:
            return
        stat = DiagnosticStatusWrapper()
        stat.name = task.name
        stat.summary(self.__starting_up_status, 'Node starting up')
        self.publish(stat)

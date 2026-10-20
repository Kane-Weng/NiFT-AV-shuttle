"""rclpy.exceptions, with rclpy's messages."""


class NotInitializedException(Exception):
    def __init__(self, *args):
        Exception.__init__(self, 'rclpy.init() has not been called', *args)


class InvalidHandle(Exception):
    pass


class NoTypeSupportImportedException(Exception):
    pass


class NameValidationException(Exception):
    def __init__(self, name_type, name, error_msg, invalid_index, *args):
        msg = 'Invalid {name_type}: {error_msg}:\n  \'{name}\'\n   {indent}^'.format(
            name_type=name_type, name=name, error_msg=error_msg, indent=' ' * invalid_index)
        super().__init__(msg, *args)


class InvalidNamespaceException(NameValidationException):
    def __init__(self, name, error_msg, invalid_index, *args):
        super().__init__('namespace', name, error_msg, invalid_index, *args)


class InvalidNodeNameException(NameValidationException):
    def __init__(self, name, error_msg, invalid_index, *args):
        super().__init__('node name', name, error_msg, invalid_index, *args)


class InvalidTopicNameException(NameValidationException):
    def __init__(self, name, error_msg, invalid_index, *args):
        super().__init__('topic name', name, error_msg, invalid_index, *args)


class InvalidServiceNameException(NameValidationException):
    def __init__(self, name, error_msg, invalid_index, *args):
        super().__init__('service name', name, error_msg, invalid_index, *args)


class ParameterException(Exception):
    pass


class ParameterNotDeclaredException(ParameterException):
    def __init__(self, parameters, *args):
        Exception.__init__(self, 'Invalid access to undeclared parameter(s)', parameters, *args)


class ParameterAlreadyDeclaredException(ParameterException):
    def __init__(self, parameters, *args):
        Exception.__init__(self, 'Parameter(s) already declared', parameters, *args)


class InvalidParameterException(ParameterException):
    def __init__(self, parameter, *args):
        Exception.__init__(self, 'Invalid parameter name', parameter, *args)


class InvalidParameterTypeException(ParameterException):
    def __init__(self, desired_parameter, expected_type, *args):
        from ._params import Parameter
        Exception.__init__(
            self,
            f"Trying to set parameter '{desired_parameter._name}' to '{desired_parameter._value}'"
            f" of type '{Parameter.Type.from_parameter_value(desired_parameter._value).name}'"
            f", expecting type '{expected_type}'",
            *args)
        self._actual_type = desired_parameter.type_
        self._param_name = desired_parameter.name


class InvalidParameterValueException(ParameterException):
    def __init__(self, parameter, value, reason, *args):
        Exception.__init__(self, 'Invalid parameter value ({value}) for parameter. Reason: {reason}'
                           .format(value=value, reason=reason), parameter, *args)


class ParameterImmutableException(ParameterException):
    def __init__(self, parameter, *args):
        Exception.__init__(self, 'Attempted to modify read-only parameter', parameter, *args)


class ParameterUninitializedException(ParameterException):
    def __init__(self, parameter_name, *args):
        Exception.__init__(self, f"The parameter '{parameter_name}' is not initialized", *args)


class ROSInterruptException(Exception):
    def __init__(self, *args):
        Exception.__init__(self, 'rclpy.shutdown() has been called', *args)

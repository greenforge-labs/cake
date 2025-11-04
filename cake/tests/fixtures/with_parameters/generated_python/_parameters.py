# flake8: noqa

# auto-generated DO NOT EDIT

import copy

from generate_parameter_library_py.python_validators import ParameterValidators
import rclpy
from rclpy.clock import Clock
from rclpy.exceptions import InvalidParameterValueException
import rclpy.parameter
from rclpy.time import Time

from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor, SetParametersResult


class parameters:
    class Params:
        # for detecting if the parameter struct has been updated
        stamp_ = Time()

        robot_name = "robot1"
        update_rate = 10.0

    class ParamListener:
        def __init__(self, node, prefix=""):
            self.prefix_ = prefix
            self.params_ = parameters.Params()
            self.node_ = node
            self.logger_ = rclpy.logging.get_logger("parameters." + prefix)

            self.declare_params()

            self.node_.add_on_set_parameters_callback(self.update)
            self.user_callback = None
            self.clock_ = Clock()

        def get_params(self):
            tmp = self.params_.stamp_
            self.params_.stamp_ = None
            paramCopy = copy.deepcopy(self.params_)
            paramCopy.stamp_ = tmp
            self.params_.stamp_ = tmp
            return paramCopy

        def is_old(self, other_param):
            return self.params_.stamp_ != other_param.stamp_

        def unpack_parameter_dict(self, namespace: str, parameter_dict: dict):
            """
            Flatten a parameter dictionary recursively.

            :param namespace: The namespace to prepend to the parameter names.
            :param parameter_dict: A dictionary of parameters keyed by the parameter names
            :return: A list of rclpy Parameter objects
            """
            parameters = []
            for param_name, param_value in parameter_dict.items():
                full_param_name = namespace + param_name
                # Unroll nested parameters
                if isinstance(param_value, dict):
                    nested_params = self.unpack_parameter_dict(
                        namespace=full_param_name + rclpy.parameter.PARAMETER_SEPARATOR_STRING,
                        parameter_dict=param_value,
                    )
                    parameters.extend(nested_params)
                else:
                    parameters.append(rclpy.parameter.Parameter(full_param_name, value=param_value))
            return parameters

        def set_params_from_dict(self, param_dict):
            params_to_set = self.unpack_parameter_dict("", param_dict)
            self.update(params_to_set)

        def set_user_callback(self, callback):
            self.user_callback = callback

        def clear_user_callback(self):
            self.user_callback = None

        def refresh_dynamic_parameters(self):
            updated_params = self.get_params()
            # TODO remove any destroyed dynamic parameters

            # declare any new dynamic parameters

        def update(self, parameters):
            updated_params = self.get_params()

            for param in parameters:
                if param.name == self.prefix_ + "robot_name":
                    updated_params.robot_name = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "update_rate":
                    updated_params.update_rate = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

            updated_params.stamp_ = self.clock_.now()
            self.update_internal_params(updated_params)
            if self.user_callback:
                self.user_callback(self.get_params())
            return SetParametersResult(successful=True)

        def update_internal_params(self, updated_params):
            self.params_ = updated_params

        def declare_params(self):
            updated_params = self.get_params()
            # declare all parameters and give default values to non-required ones
            if not self.node_.has_parameter(self.prefix_ + "robot_name"):
                descriptor = ParameterDescriptor(description="Name of the robot", read_only=False)
                parameter = updated_params.robot_name
                self.node_.declare_parameter(self.prefix_ + "robot_name", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "update_rate"):
                descriptor = ParameterDescriptor(description="Update rate in Hz", read_only=False)
                parameter = updated_params.update_rate
                self.node_.declare_parameter(self.prefix_ + "update_rate", parameter, descriptor)

            # TODO: need validation
            # get parameters and fill struct fields
            param = self.node_.get_parameter(self.prefix_ + "robot_name")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.robot_name = param.value
            param = self.node_.get_parameter(self.prefix_ + "update_rate")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.update_rate = param.value

            self.update_internal_params(updated_params)

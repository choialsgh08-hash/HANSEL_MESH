"""Small ROS parameter-service boundary for the RQT configuration model."""
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.parameter import Parameter


class _DecodedFuture:
    def __init__(self, future, decode, read_client=None):
        self.future, self.decode, self.read_client = future, decode, read_client

    def done(self): return self.future.done()
    def result(self): return self.decode(self.future.result())

    def discard_read(self):
        if self.read_client is not None:
            self.read_client.remove_pending_request(self.future)
            self.future.cancel()


class ParameterEndpoint:
    def __init__(self, node, unit):
        self.node = node
        prefix = f'/hansel/{unit}/unit_controller'
        self._get = node.create_client(GetParameters, prefix + '/get_parameters')
        self._set = node.create_client(SetParametersAtomically, prefix + '/set_parameters_atomically')

    def ready(self):
        return self._get.service_is_ready() and self._set.service_is_ready()

    def read(self, names):
        request = GetParameters.Request()
        request.names = list(names)
        def decode(response):
            if response is None or len(response.values) != len(names):
                raise ValueError('Incomplete get_parameters response')
            if any(value.type != ParameterType.PARAMETER_DOUBLE for value in response.values):
                raise ValueError('Expected declared DOUBLE parameters from unit_controller')
            return {name:value.double_value for name,value in zip(names,response.values)}
        return _DecodedFuture(self._get.call_async(request), decode, self._get)

    def write(self, values):
        request = SetParametersAtomically.Request()
        request.parameters = [Parameter(name, Parameter.Type.DOUBLE, float(value)).to_parameter_msg()
                              for name,value in values.items()]
        def decode(response):
            if response is None or not response.result.successful:
                raise RuntimeError(response.result.reason if response is not None else 'No set response')
        return _DecodedFuture(self._set.call_async(request), decode)

    def close(self):
        self.node.destroy_client(self._get)
        self.node.destroy_client(self._set)

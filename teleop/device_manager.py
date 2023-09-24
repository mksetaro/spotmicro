import logging
from teleop import input_controller


class BaseDeviceManager(object):
    def __init__(self, name, input_config, keys_list) -> None:
        self.name = name
        self.input_controller = input_controller.InputDeviceController(
            input_config=input_config, device_manager=self
        )
        for key in keys_list:
            self.input_controller.register_handler(key)
        self.input_controller.open()

    def info(self):
        self.input_controller.info()

    def new_event(self, payload):
        logging.info(
            "Device {name} received event {payload}".format(
                name=self.name, payload=payload
            )
        )
        self.process_payload(payload)

    def process_payload(self, payload):
        raise TypeError(
            "Cannot Use BaseDeviceManager, inherit from it and override this method"
        )

    def run(self):
        self.input_controller.run()

    def is_running(self):
        self.input_controller.is_running()

    def shutdown(self):
        self.input_controller.shutdown()

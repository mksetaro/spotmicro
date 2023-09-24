import logging
import asyncio
from teleop import device_manager


# _TEST_CONFIG = {"event_path": "/dev/input/event6"}
_TEST_CONFIG = {"event_path": "/dev/input/event3"}

logging.basicConfig(
    format="%(asctime)s |%(levelname)s| %(message)s", level=logging.DEBUG
)


class MouseDeviceManager(device_manager.BaseDeviceManager):
    def __init__(self, name, input_config, keys_list) -> None:
        super().__init__(name, input_config, keys_list)

    def process_payload(self, payload):
        logging.info("Processed Button Event {}".format(payload))
        if payload["value"]["type"][0] == "BTN_LEFT":
            self.shutdown()


def main():
    try:
        mouse = MouseDeviceManager(
            "mouse_test_controller",
            _TEST_CONFIG,
            ["BTN_LEFT", "REL_X", "REL_Y", "BTN_MIDDLE"],
        )
        mouse.run()

    except KeyboardInterrupt:
        pass
    finally:
        logging.info("Shutting down InputController")
        mouse.shutdown()


if __name__ == "__main__":
    main()

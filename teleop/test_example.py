import logging
from teleop.input_controller import InputDeviceController


_TEST_CONFIG = {"event_path": "/dev/input/event3"}

logging.basicConfig(
    format="%(asctime)s |%(levelname)s| %(message)s", level=logging.DEBUG
)


def main():
    try:
        mouse = InputDeviceController(_TEST_CONFIG)
        mouse.open()
        mouse.run()
    except KeyboardInterrupt:
        pass
    finally:
        logging.info("Shutting down InputController")
        mouse.shutdown()


if __name__ == "__main__":
    main()

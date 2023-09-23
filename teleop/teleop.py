import evdev
import logging
import asyncio

from evdev import ecodes

# read specific events
# notify event
EV_PATH_KEY = "event_path"

_TEST_CONFIG = {EV_PATH_KEY: "/dev/input/event3"}


class InputDeviceController:
    def __init__(self, input_config) -> None:
        self.event_path = input_config[EV_PATH_KEY]
        self.events_whitelist = [ecodes.REL_Y, ecodes.REL_X, ecodes.BTN_LEFT]
        self.device = None
        self.event_loop = None

    def info(self):
        if self.is_open():
            logging.info(
                "{path} | {name} | {phys}".format(
                    path=self.device.path, name=self.device.name, phys=self.device.phys
                )
            )
        elif self.open():
            logging.info(
                "{path} | {name} | {phys}".format(
                    path=self.device.path, name=self.device.name, phys=self.device.phys
                )
            )
            logging.debug("{}".format(self.device.capabilities(verbose=True)))
        else:
            logging.error("Could not print info for event {}".format(self.event_path))

    def is_open(self):
        return self.device is not None

    def open(self) -> bool:
        if self.is_open():
            return True
        try:
            self.device = evdev.InputDevice(self.event_path)
            return True
        except:
            logging.error("Cannot start listener for event {}".format(self.event_path))
            return False

    def close(self):
        pass

    def is_running(self):
        return self.event_loop is not None and self.event_loop.is_running()

    async def _listen_events(self):
        async for ev in self.device.async_read_loop():
            if ev.type != ecodes.EV_SYN and ev.code in self.events_whitelist:
                logging.warning("{}".format(evdev.util.categorize(ev)))

    def run(self) -> bool:
        if not self.is_open():
            logging.error("Controller cannot run without device. Call open() first")
            return False
        if self.is_running():
            logging.warning(
                "Controller already running and listening device {}".format(
                    self.device.name
                )
            )
            return True
        self.event_loop = asyncio.get_event_loop()
        asyncio.ensure_future(self._listen_events())
        self.event_loop.run_forever()  # to be improved

    def shutdown(self):
        self.event_loop.stop()
        self.close()
        pass


logging.basicConfig(
    format="%(asctime)s |%(levelname)s| %(message)s", level=logging.DEBUG
)


def main():
    try:
        mouse = InputDeviceController(_TEST_CONFIG)
        mouse.info()
        mouse.open()
        mouse.run()
    except KeyboardInterrupt:
        pass
    finally:
        logging.info("Shutting down InputController")
        mouse.shutdown()


if __name__ == "__main__":
    main()

import asyncio
import evdev
import logging
from evdev import ecodes


class EventHandler(object):
    def __init__(self, decoder_function, name) -> None:
        self.name = name
        self.decoder_function = decoder_function

    def __call__(self, *args, **kwargs):
        logging.info("Name: {name} ".format(name=self.name))
        payload = self.decoder_function(*args, **kwargs)
        if payload is None:
            logging.error(
                "Empty payload, decorated decoders must return a json dictionary"
            )
            return
        # notify function to external
        logging.info("Payload: {payload} ".format(payload=payload))


def EventHandlerDecorator(name):
    def _create_decorator(function):
        return EventHandler(function, name)

    return _create_decorator


@EventHandlerDecorator(name="MouseLeftButtonHandler")
def handle_mouse_left_button(value):
    return {"value": value}


INPUT_HANDLERS_MAPPING = {
    hash((ecodes.EV_KEY, ecodes.BTN_LEFT)): handle_mouse_left_button
}


class InputDeviceController:
    def __init__(self, input_config) -> None:
        self.event_path = input_config["event_path"]
        self.event_handlers = INPUT_HANDLERS_MAPPING
        self.device = None
        self.event_loop = None

    def info(self) -> None:
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

    def close(self) -> None:
        pass

    def is_running(self) -> bool:
        return self.event_loop is not None and self.event_loop.is_running()

    async def _listen_events(self) -> None:
        async for ev in self.device.async_read_loop():
            handler_key = hash((ev.type, ev.code))
            if handler_key in self.event_handlers:
                self.event_handlers[handler_key](ev.value)
                logging.debug("{}".format(evdev.util.categorize(ev)))

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
        return True

    def shutdown(self) -> None:
        self.event_loop.stop()
        self.close()

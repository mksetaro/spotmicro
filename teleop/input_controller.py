import asyncio
import evdev
import logging
from evdev import ecodes
import time


class EventHandler(object):
    def __init__(self, decoder_function, name) -> None:
        self.name = name
        self.decoder_function = decoder_function
        self.cache = dict()
        self.payload = {"name": self.name, "timestamp": 0, "value": None}

    def __call__(self, *args, **kwargs):
        logging.info("Name: {name} ".format(name=self.name))
        kwargs.update({"cache": self.cache, "payload": self.payload})
        done = self.decoder_function(*args, **kwargs)

        # notify function to external
        if done:
            self._fire_event()

        return done

    def _fire_event(self):
        if not all(key in self.payload for key in ["name", "timestamp", "value"]):
            logging.error(
                "Payload non valid, it must contain keys: {}".format(
                    ["name", "timestamp", "value"]
                )
            )
            return
        self.payload["timestamp"] = time.time()
        logging.info("Payload: {payload} fired".format(payload=self.payload))
        self._reset()

    def _reset(self):
        self.payload["timestamp"] = 0
        self.payload["value"] = None
        self.cache.clear()


def EventHandlerDecorator(name):
    def _create_decorator(function):
        return EventHandler(function, name)

    return _create_decorator


@EventHandlerDecorator(name="MouseLeftButtonHandler")
def handle_mouse_left_button(input_event, cache, payload, *args, **kwargs):
    print(cache)
    print(input_event.type)
    print(input_event.code)
    payload["value"] = input_event.value
    return True


@EventHandlerDecorator(name="MouseRelativePositionHandler")
def handle_mouse_position(input_event, cache, payload, *args, **kwargs):
    if "pos_ts" not in cache or cache["pos_ts"] > input_event.timestamp():
        cache["pos_ts"] = input_event.timestamp()
    elif cache["pos_ts"] < input_event.timestamp():
        logging.warning("Package dated in the past, dropping message")
        payload["value"] = None
        return True

    if input_event.type == ecodes.EV_REL:
        cache[ecodes.REL[input_event.code]] = input_event.value

    if input_event.type == ecodes.EV_SYN:
        payload["value"] = {
            key: value for key, value in cache.items() if key != "pos_ts"
        }
        return True

    return False


INPUT_HANDLERS_MAPPING = {
    hash((ecodes.EV_KEY, ecodes.BTN_LEFT)): handle_mouse_left_button,
    hash((ecodes.EV_REL, ecodes.REL_X)): handle_mouse_position,
    hash((ecodes.EV_REL, ecodes.REL_Y)): handle_mouse_position,
    hash((ecodes.EV_SYN, ecodes.SYN_REPORT)): handle_mouse_position,
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
                self.event_handlers[handler_key](ev)
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

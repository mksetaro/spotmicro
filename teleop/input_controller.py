import asyncio
import evdev
import logging

from teleop import event_handler

POLL_SLEEP_DURATION = 40e-3


class InputDeviceController:
    def __init__(self, input_config, fire_event_func) -> None:
        self.event_path = input_config["event_path"]
        self.fire_event_func = fire_event_func
        self.event_handlers = dict()
        self.device = None
        self.event_loop = None
        self.task = None
        self.shutdown_requested = False

    def info(self) -> None:
        if self.is_open():
            logging.info(
                "{path} | {name} | {phys}".format(
                    path=self.device.path, name=self.device.name, phys=self.device.phys
                )
            )
            logging.info("{}".format(self.device.capabilities(verbose=True)))
        elif self.open():
            logging.info(
                "{path} | {name} | {phys}".format(
                    path=self.device.path, name=self.device.name, phys=self.device.phys
                )
            )
            logging.info("{}".format(self.device.capabilities(verbose=True)))
        else:
            logging.error("Could not print info for event {}".format(self.event_path))

    def is_open(self) -> bool:
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

    def register_handler(self, key) -> None:
        try:
            tmp = key.split("_")
            if "REL" in tmp:
                self.event_handlers.update(
                    {
                        hash(
                            (evdev.ecodes.EV_REL, evdev.ecodes.ecodes[key])
                        ): event_handler.handle_relative_position_event
                    }
                )

                self.event_handlers.update(
                    {
                        hash(
                            (evdev.ecodes.EV_SYN, evdev.ecodes.SYN_REPORT)
                        ): event_handler.handle_relative_position_event
                    }
                )

            elif "KEY" in tmp or "BTN" in tmp:
                self.event_handlers.update(
                    {
                        hash(
                            (evdev.ecodes.EV_KEY, evdev.ecodes.ecodes[key])
                        ): event_handler.handle_button_event
                    }
                )

            else:
                logging.error("Event type EV_{} not supported yet".format(tmp[0]))

        except:
            logging.error("Device does not support {}, dropping handler".format(key))

    def _fire_event(self, payload) -> None:
        if not all(key in payload for key in ["name", "timestamp", "value"]):
            logging.error(
                "Payload non valid, it must contain keys: {}".format(
                    ["name", "timestamp", "value"]
                )
            )
            return
        self.fire_event_func(payload)

    def is_running(self) -> bool:
        return (
            self.event_loop is not None
            and self.task is not None
            and self.event_loop.is_running()
            and not self.task.done()
        )

    async def _listen_events(self) -> None:
        async for ev in self.device.async_read_loop():
            handler_key = hash((ev.type, ev.code))
            if handler_key in self.event_handlers:
                status, payload = self.event_handlers[handler_key](ev)
                logging.debug("{}".format(evdev.util.categorize(ev)))
                if status == event_handler.HandlerStatus.Done:
                    self._fire_event(payload)

    def run(self) -> bool:
        try:
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
            self.task = self.event_loop.create_task(self._listen_events())
            self.event_loop = self.event_loop.run_until_complete(self.task)
        except asyncio.CancelledError:
            logging.info("{} Input Controller Shutdown ".format(self.device.name))
            return True

    def shutdown(self) -> None:
        self.task.cancel()

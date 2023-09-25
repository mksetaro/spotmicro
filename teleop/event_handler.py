import evdev
import logging
import time
import enum
import copy


class HandlerStatus(enum.Enum):
    Done = (0,)
    Pending = (1,)


class EventHandler(object):
    def __init__(self, decoder_function, name) -> None:
        self.name = name
        self.decoder_function = decoder_function
        self.cache = dict()
        self.payload = {"name": self.name, "timestamp": 0, "value": None}

    def __call__(self, *args, **kwargs) -> None:
        logging.info("Name: {name} ".format(name=self.name))
        kwargs.update({"cache": self.cache, "payload": self.payload})
        done = self.decoder_function(*args, **kwargs)

        if done:
            payload = copy.deepcopy(self.payload)
            payload["timestamp"] = time.time()
            self._reset()
            return (HandlerStatus.Done, payload)

        return (HandlerStatus.Pending, None)

    def _reset(self) -> None:
        self.payload["timestamp"] = 0
        self.payload["value"] = None
        self.cache.clear()


def EventHandlerDecorator(name):
    def _create_decorator(function):
        return EventHandler(function, name)

    return _create_decorator


@EventHandlerDecorator(name="ButtonHandler")
def handle_button_event(input_event, cache, payload, *args, **kwargs) -> bool:
    payload["value"] = {
        "type": evdev.ecodes.bytype[evdev.ecodes.EV_KEY][input_event.code],
        "pressed": bool(input_event.value),
    }
    return True


@EventHandlerDecorator(name="RelativePositionHandler")
def handle_relative_position_event(
    input_event, cache, payload, *args, **kwargs
) -> bool:
    # SYN_REPORT are emitted after a multipacket event happens, thus if it's received but the cache has no REL the event should be discarded
    if input_event.type == evdev.ecodes.EV_SYN and len(list(cache)) == 0:
        return True

    if "pos_ts" not in cache or cache["pos_ts"] > input_event.timestamp():
        cache["pos_ts"] = input_event.timestamp()
    elif cache["pos_ts"] < input_event.timestamp():
        logging.warning(
            "Package dated in the past [cahced_ts {cached_ts} - event_ts {event_ts}], dropping message".format(
                cached_ts=cache["pos_ts"], event_ts=input_event.timestamp()
            )
        )
        payload["value"] = None
        return True

    if input_event.type == evdev.ecodes.EV_REL:
        cache[evdev.ecodes.REL[input_event.code]] = input_event.value
    if input_event.type == evdev.ecodes.EV_SYN:
        payload["value"] = {
            key: value for key, value in cache.items() if key != "pos_ts"
        }
        payload["value"]["type"] = "REL_POS"
        return True

    return False

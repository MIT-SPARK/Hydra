"""Module containing code for running Hydra."""

import logging


class DataCallbackRegistry:
    """Factory for callbacks."""

    __shared_state = None

    def __init__(self):
        """Make a handle to the underlying state."""
        # Borg pattern: set class state to global state
        if not DataCallbackRegistry.__shared_state:
            DataCallbackRegistry.__shared_state = self.__dict__
            self._factories = {}
        else:
            self.__dict__ = DataCallbackRegistry.__shared_state

    @staticmethod
    def register(cls, name):
        """Register a function that gets triggered before new data is given to Hydra."""
        instance = DataCallbackRegistry()
        instance._factories[name] = cls

    @staticmethod
    def create(names):
        """Create a list of callbacks."""
        instance = DataCallbackRegistry()
        callbacks = []
        for name, config in names.items():
            if name not in instance._factories:
                logging.warning("Unknown factory!")

            callbacks.append(instance._factories[name](**config))

        return callbacks


def register_data_callback(name):
    """Register a class with a callback."""

    def decorator(cls):
        DataCallbackRegistry.register(cls, name)
        return cls

    return decorator


@register_data_callback("image_viewer")
class ImageVisualizer:
    """GUI for showing images."""

    def __init__(self):
        """Initialize pyqtgraph."""
        self._init = False

        try:
            import pyqtgraph.multiprocess as mp
        except Exception as e:
            logging.error(f"pyqt issue: {e}")
            return

        self._init = True
        proc = mp.QtProcess(processRequests=False)
        self._pg = proc._import("pyqtgraph")
        self._pg.setConfigOptions(imageAxisOrder="row-major")
        self._view = None

    def __call__(self, packet):
        """Show an image."""
        if not self._init:
            return

        kwargs = {"autoLevels": False, "levels": (0, 255)}
        if self._view is None:
            self._view = self._pg.image(packet.color, **kwargs)
        else:
            self._view.setImage(packet.color, **kwargs)

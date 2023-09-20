"""Module containing utilities for interfacing a new simulator with Hydra."""
import abc


class DataInterface(abc.ABC):
    """Interface to inherit from for new data sources."""

    @abc.abstractmethod
    def set_pose(self, timestamp, world_T_pose):
        """Set pose (and optionally timestamp) for the dataset in question."""
        pass

    @property
    @abc.abstractmethod
    def colormap(self):
        """Get current colormap."""
        pass

    @property
    @abc.abstractmethod
    def camera_info(self):
        """Get current sensor."""
        pass

    @property
    @abc.abstractmethod
    def depth(self):
        """Get last depth image."""
        pass

    @property
    @abc.abstractmethod
    def labels(self):
        """Get last label image."""
        pass

    @property
    @abc.abstractmethod
    def rgb(self):
        """Get last RGB image."""
        pass

    @property
    def semantics(self):
        """Get last semantic image."""
        return self.colormap(self.labels())

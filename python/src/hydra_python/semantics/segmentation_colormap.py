"""Colormaps and label converters."""

import distinctipy
import numpy as np
import yaml
import csv


class LabelConverter:
    """Converter between one label space and another labels."""

    def __init__(self, index_array, names):
        """
        Construct a converter between one label space and another.

        Args:
            index_array (np.ndarray): Row vector containing the new categories
            names (List[str]): Name for each category
        """
        self._index_array = index_array
        self._names = names

    def __call__(self, labels):
        """Convert original label image to new label image."""
        return np.take(self._index_array, labels, mode="raise")

    def reindex(self, remap):
        """Combine two converters."""
        remapped_array = []
        remapped_names = []
        for source_idx, target_idx in enumerate(self._index_array):
            new_target = remap.get(target_idx, target_idx)
            remapped_array.append(new_target)
            remapped_names.append(self.name(new_target))

        remapped_array = np.array(remapped_array, dtype=self._index_array.dtype)
        return LabelConverter(remapped_array, remapped_names)

    def name(self, idx):
        """Get name for category."""
        return "unknown" if idx < 0 or idx >= len(self._names) else self._names[idx]

    @property
    def names(self):
        """Get category names."""
        return self.names

    @classmethod
    def from_mapping(cls, sublabel_to_label, name_mapping=None, unknown=0):
        """
        Construct conversion from map between original and new label.

        Args:
            sublabel_to_label (Dict[int, int]): Map from old to new labels
            name_mapping (Optional[Dict[int, str]]): Map from label to name
            unknown (int): Default label
        """
        N_sublabels = max(sublabel_to_label) + 1
        label_map = unknown * np.ones(N_sublabels, dtype=np.uint8)
        for sublabel, label in sublabel_to_label.items():
            label_map[sublabel] = label

        names = []
        name_mapping = name_mapping if name_mapping is not None else {}
        max_category = np.max(label_map)
        names = [name_mapping.get(x, "unknown") for x in range(max_category)]
        return cls(label_map, names)


class SegmentationColormap:
    """Colormap between labels and colors used for Hydra."""

    def __init__(self, colors, names=None):
        """Construct a colormap directly from an array of colors."""
        self._colors = colors
        self._names = names if names else []

    @classmethod
    def from_names(cls, names, pastel_factor=0.5, rng_seed=1234):
        """Construct a colormap from names."""
        N_colors = len(names)
        colors = distinctipy.get_colors(
            N_colors, pastel_factor=pastel_factor, rng=rng_seed
        )
        colors = (255 * np.array(colors)).astype(np.uint8)
        return cls(colors.transpose(), names=names)

    @classmethod
    def from_yaml(cls, filepath):
        """Build colormap from semantic recolor yaml file."""
        with filepath.open("r") as fin:
            contents = yaml.safe_load(fin.read())

        label_to_color = {}
        sublabel_to_label = {}

        classes = contents["classes"]
        for label in classes:
            color = 255 * np.array(contents[f"class_info/{label}/color"])
            color = np.squeeze(color).astype(np.uint8)
            label_to_color[label] = color

            sublabels = contents[f"class_info/{label}/labels"]
            for sublabel in sublabels:
                sublabel_to_label[sublabel] = label

        default_color = np.array(contents["default_color"])
        default_color = np.squeeze(255 * default_color).astype(np.uint8)

        N_colors = max(label_to_color) + 1
        colors = np.zeros((3, N_colors), dtype=np.uint8)
        for label in range(N_colors):
            colors[:, label] = (
                label_to_color[label] if label in label_to_color else default_color
            )

        return cls(colors)

    def __call__(self, labels):
        """Apply colormap to label image."""
        semantic_img = np.take(self._colors, labels, mode="raise", axis=1)
        return np.transpose(semantic_img, [1, 2, 0])

    def __str__(self):
        """Show colormap."""
        to_return = "Colormap:\n"
        for i in range(self._colors.shape[1]):
            name = f"class_{i}" if i > len(self._names) else self._names[i]
            to_return += f"  - {i}: {name} → {np.squeeze(self._colors[:, i])}\n"

        return to_return

    def save(self, fout):
        """Save to CSV file."""
        writer = csv.writer(fout)
        writer.writerow(["name", "red", "green", "blue", "alpha", "id"])

        for i in range(self._colors.shape[1]):
            name = self._names[i] if i < len(self._names) else f"class_{i}"
            writer.writerow(
                [
                    name,
                    self._colors[0, i],
                    self._colors[1, i],
                    self._colors[2, i],
                    255,
                    i,
                ]
            )

        fout.flush()

    def fill_label_space(self, config):
        """Fill label space config with appropriate information."""
        config.total_labels = self._colors.shape[1]

    @property
    def colors(self):
        """Get label colors."""
        return self._colors

    @property
    def names(self):
        """Get label names."""
        return self._names

    def set_names(self, names):
        """Set label names."""
        self._names = names

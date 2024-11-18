"""Encoder for clip feature."""

from hydra_python.data_callbacks import register_data_callback
from hydra_python.data_loader import InputPacket


@register_data_callback("clip")
class ClipEncoder:
    """Image feature encoder."""

    def __init__(self, model_name="ViT-L/14"):
        """Load clip."""
        import semantic_inference.models as models

        config = models.ClipConfig()
        config.model_name = model_name
        self.device = models.default_device()
        self.model = models.ClipVisionWrapper(config).to(self.device)
        self.model.eval()
        self.transform = models.get_image_preprocessor(self.model.input_size).to(
            self.device
        )
        self.center_crop = models.center_crop

    def __call__(self, packet: InputPacket):
        """Add clip feature encoding to input packet."""
        import torch

        with torch.no_grad():
            img = torch.from_numpy(packet.color).to(self.device).permute((2, 0, 1))
            img = self.center_crop(self.transform(img), self.model.input_size)
            feature = torch.squeeze(self.model(img.unsqueeze(0)).cpu()).numpy()

        packet.extras["feature"] = feature

"""Semantic segmentation plugin for running a onnx model."""


class OnnxSegmenter:
    """Class to do 2d semantic segmentation using onnx."""

    def __init__(self, filename, colormap):
        """Load an onnx file and set up the model."""
        try:
            import torchvision
            import onnxruntime as ort

            self._initialized = True
        except ImportError:
            self._initialized = False
            print("missing dependencies; either torchvision or onnxruntime are missing")

        options = ort.SessionOptions()
        options.inter_op_num_threads = 1
        options.intra_op_num_threads = 1
        self._session = ort.InferenceSession(
            filename,
            sess_options=options,
            providers=[
                # "TensorrtExecutionProvider",
                "CUDAExecutionProvider",
                # "CPUExecutionProvider",
            ],
        )

        self._to_tensor = torchvision.transforms.ToTensor()
        self._semantic_colormap = colormap

    def __call__(self, rgb):
        """Convert an rgb image to a colored 2d semantic image."""
        if self._to_tensor is None:
            return None

        # TODO(nathan) resize input and output
        input_rgb = self._to_tensor(rgb)
        input_rgb.unsqueeze_(0)

        inputs = {self._session.get_inputs()[0].name: input_rgb.cpu().numpy()}
        labels = self._session.run(None, inputs)[0]
        return self._semantic_colormap(labels)

    @property
    def colormap(self):
        """Get colormap for segmenter."""
        return self._semantic_colormap

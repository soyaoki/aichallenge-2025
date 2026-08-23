"""ONNX Runtime wrapper around comma.ai's driving_supercombo network.

Replicates the input-queue semantics of openpilot's tinygrad runner
(``openpilot/selfdrive/modeld/compile_modeld.py``, commit 084747c7): the model
runs at ``MODEL_RUN_FREQ`` but consumes context at ``MODEL_CONTEXT_FREQ``, so
frames, features and desire pulses are pushed every step and sampled every
``frame_skip``-th step.
"""

import codecs
import hashlib
import pickle

import numpy as np
import onnxruntime as ort

from openpilot_controller.model_constants import ModelConstants
from openpilot_controller.parse_model_outputs import Parser

# Provider aliases accepted by the `model.provider` parameter.
PROVIDER_ALIASES = {
    'cpu': ['CPUExecutionProvider'],
    'cuda': ['CUDAExecutionProvider', 'CPUExecutionProvider'],
    'tensorrt': ['TensorrtExecutionProvider', 'CUDAExecutionProvider', 'CPUExecutionProvider'],
}
# `auto` picks the fastest provider this onnxruntime build actually offers.
AUTO_PROVIDER_ORDER = ['TensorrtExecutionProvider', 'CUDAExecutionProvider', 'CPUExecutionProvider']

_ORT_DTYPES = {
    'tensor(uint8)': np.uint8,
    'tensor(float16)': np.float16,
    'tensor(float)': np.float32,
}


def resolve_providers(requested: str) -> list:
    """Map a provider name to an onnxruntime provider list, dropping unavailable ones."""
    available = ort.get_available_providers()
    requested = (requested or 'auto').strip().lower()
    if requested == 'auto':
        wanted = [p for p in AUTO_PROVIDER_ORDER if p in available]
    elif requested in PROVIDER_ALIASES:
        wanted = [p for p in PROVIDER_ALIASES[requested] if p in available]
    else:
        raise ValueError(
            f"unknown provider '{requested}', expected one of: auto, {', '.join(PROVIDER_ALIASES)}")
    if not wanted:
        raise RuntimeError(
            f"provider '{requested}' is not available in this onnxruntime build "
            f"(available: {', '.join(available)})")
    return wanted


def sha256_of(path: str) -> str:
    digest = hashlib.sha256()
    with open(path, 'rb') as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b''):
            digest.update(chunk)
    return digest.hexdigest()


class SupercomboRunner:
    """Stateful driving_supercombo inference.

    ``expected_sha256`` guards the ``output_slices`` metadata, which is stored
    in the ONNX file as a pickle and therefore must come from a known file.
    Pass an empty string only when deliberately running a locally built model.
    """

    def __init__(self, model_path: str, provider: str = 'auto', expected_sha256: str = '',
                 intra_op_threads: int = 0):
        if expected_sha256:
            actual = sha256_of(model_path)
            if actual != expected_sha256:
                raise RuntimeError(
                    f"model checksum mismatch for {model_path}: expected {expected_sha256}, got {actual}. "
                    "Re-download it with `make openpilot-models`.")

        self.providers = resolve_providers(provider)
        options = ort.SessionOptions()
        options.log_severity_level = 3
        if intra_op_threads > 0:
            options.intra_op_num_threads = int(intra_op_threads)
        self.session = ort.InferenceSession(model_path, options, providers=self.providers)
        self.active_providers = self.session.get_providers()

        metadata = self.session.get_modelmeta().custom_metadata_map
        if 'output_slices' not in metadata:
            raise RuntimeError(f"{model_path} has no 'output_slices' metadata; not a supercombo model")
        self.output_slices = pickle.loads(codecs.decode(metadata['output_slices'].encode(), 'base64'))
        self.model_checkpoint = metadata.get('model_checkpoint', 'unknown')

        self.input_shapes = {i.name: tuple(i.shape) for i in self.session.get_inputs()}
        self.input_dtypes = {i.name: _ORT_DTYPES[i.type] for i in self.session.get_inputs()}

        img_shape = self.input_shapes['img']  # (1, 12, 128, 256)
        self.n_frames = img_shape[1] // 6
        self.frame_height, self.frame_width = img_shape[2], img_shape[3]
        self.frame_skip = ModelConstants.MODEL_RUN_FREQ // ModelConstants.MODEL_CONTEXT_FREQ

        self.feature_shape = self.input_shapes['features_buffer']  # (1, 24, 512)
        self.desire_shape = self.input_shapes['desire_pulse']      # (1, 25, 8)

        self.prev_feat = np.zeros(self.feature_shape[2], dtype=np.float32)
        self.prev_desire = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)
        self.parser = Parser()
        self._allocate_queues()

    def _allocate_queues(self) -> None:
        img_queue_len = self.frame_skip * (self.n_frames - 1) + 1
        self.img_q = np.zeros((img_queue_len, 6, self.frame_height, self.frame_width), dtype=np.uint8)
        self.big_img_q = np.zeros_like(self.img_q)
        self.feat_q = np.zeros((self.frame_skip * self.feature_shape[1], self.feature_shape[2]),
                               dtype=np.float32)
        self.desire_q = np.zeros((self.frame_skip * self.desire_shape[1], self.desire_shape[2]),
                                 dtype=np.float32)

    def set_frame_skip(self, frame_skip: int) -> None:
        """Retune how many steps separate the two stacked frames.

        The network reads context at MODEL_CONTEXT_FREQ, so the pair of frames it
        sees should be ~200 ms apart. Upstream runs at a fixed 20 Hz; when the
        camera runs slower, a smaller skip keeps that spacing. Only the queues are
        rebuilt, so this is cheap, but it drops the temporal history.
        """
        frame_skip = max(1, int(frame_skip))
        if frame_skip == self.frame_skip:
            return
        self.frame_skip = frame_skip
        self._allocate_queues()
        self.prev_feat[:] = 0
        self.prev_desire[:] = 0

    @property
    def model_frame_shape(self):
        return (6, self.frame_height, self.frame_width)

    def reset(self) -> None:
        self.img_q[:] = 0
        self.big_img_q[:] = 0
        self.feat_q[:] = 0
        self.desire_q[:] = 0
        self.prev_feat[:] = 0
        self.prev_desire[:] = 0

    @staticmethod
    def _push(queue: np.ndarray, value: np.ndarray) -> None:
        queue[:-1] = queue[1:]
        queue[-1] = value

    def run(self, img_frame: np.ndarray, big_img_frame: np.ndarray,
            desire_pulse: np.ndarray, traffic_convention: np.ndarray,
            action_t: np.ndarray) -> dict:
        # The model decides when an action is complete, so desire is a pulse on the rising edge.
        desire_pulse = np.asarray(desire_pulse, dtype=np.float32).copy()
        desire_pulse[0] = 0
        desire = np.where(desire_pulse - self.prev_desire > 0.99, desire_pulse, 0)
        self.prev_desire[:] = desire_pulse

        self._push(self.img_q, img_frame)
        self._push(self.big_img_q, big_img_frame)
        self._push(self.desire_q, desire)
        self._push(self.feat_q, self.prev_feat)

        feeds = {
            'img': self.img_q[::self.frame_skip].reshape(1, -1, self.frame_height, self.frame_width),
            'big_img': self.big_img_q[::self.frame_skip].reshape(1, -1, self.frame_height, self.frame_width),
            'features_buffer': self.feat_q[::self.frame_skip][None],
            # sample_desire: max-pool each frame_skip-sized group
            'desire_pulse': self.desire_q.reshape(-1, self.frame_skip, self.desire_q.shape[1]).max(axis=1)[None],
            'traffic_convention': np.asarray(traffic_convention, dtype=np.float32).reshape(1, -1),
            'action_t': np.asarray(action_t, dtype=np.float32).reshape(1, -1),
        }
        feeds = {name: np.ascontiguousarray(value, dtype=self.input_dtypes[name])
                 for name, value in feeds.items()}

        raw = self.session.run(None, feeds)[0][0].astype(np.float32)
        self.prev_feat[:] = raw[self.output_slices['hidden_state']]

        sliced = {name: raw[np.newaxis, sl] for name, sl in self.output_slices.items() if name != 'pad'}
        return self.parser.parse_outputs(sliced)

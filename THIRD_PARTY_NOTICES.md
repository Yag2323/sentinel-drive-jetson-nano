# Third-Party Notices

This snapshot contains project-authored integration code but does not vendor
the third-party YOLOv5 source tree or model weights.

## Ultralytics YOLOv5 v6.0

- Upstream repository: <https://github.com/ultralytics/yolov5>
- Release tag: `v6.0`
- Pinned commit: `956be8e642b5c10af4a1533e09084ca32ff4f21f`
- Provisioning helper: `prepare_yolov5_v6.py`
- Expected `yolov5n.pt` size: `3,952,441 bytes`
- Expected `yolov5n.pt` SHA-256:
  `649e089f59b78ac021025de035b2d9c45dc26e544ea252955d0ffcefc1099e2f`
- Expected `yolov5s.pt` size: `14,698,491 bytes`
- Expected `yolov5s.pt` SHA-256:
  `c3b140f32001a9eec4afa07120b3851eb1b6c2c7c7e7a4303af9eadfacbeb598`

`prepare_yolov5_v6.py` clones the pinned upstream revision, verifies its Git
identity and downloads the two official release assets with byte-count and
SHA-256 verification. Review and comply with the `LICENSE` and notices in that
exact upstream revision before redistribution or deployment. The upstream
project and weights are not relicensed by this repository.

## NVIDIA JetPack platform components

The runtime depends on platform-provided components such as CUDA-enabled
PyTorch, OpenCV with GStreamer support, Argus camera elements, GStreamer and
PyGObject. These binaries are not included here and remain subject to their
respective NVIDIA, distribution and upstream terms.

## Project licensing

No open-source license has been selected for the project-authored files in this
snapshot. Nothing in this notice grants rights beyond those provided by the
applicable rights holder or upstream license.

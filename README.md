# microMVP (modern rewrite)

This is a clean Python 3.12 rewrite of the legacy microMVP stack.
It preserves the same high-level flow (vision -> localization -> path follow -> control)
while modernizing structure, typing, and UI.

## Install

Create/activate your conda env, then:

```bash
python -m pip install -e .
```

Note: We use `pygame-ce` with `pygame-gui` for Python 3.12 compatibility.

## Run (demo/sim)

```bash
micromvp --sim
```

## Run (vision publisher)

```bash
python -m micromvp.vision.aruco_publisher 0 5556 --dict DICT_4X4_50 --calibration path/to/calib.yml
```

## Run (real hardware)

```bash
micromvp --real --vision-sub tcp://localhost:5556
```

## Notes

- UI is built with PyQt6.
- ZMQ message format matches legacy: `<id> x0 y0 x1 y1 x2 y2 x3 y3`.
- Hardware support is cleanly isolated in `micromvp/io`.
- Demo mode does not require hardware.

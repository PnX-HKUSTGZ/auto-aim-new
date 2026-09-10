`SceneUpdate.json` and `FrameTransform.json` are official, self-contained Foxglove JSON schemas from
https://github.com/foxglove/schemas/blob/main/schemas/jsonschema/SceneUpdate.json
and https://github.com/foxglove/schemas/blob/main/schemas/jsonschema/FrameTransform.json
(retrieved 2026-09-06). Their MIT license is included in `LICENSE`.

CMake embeds these files in a generated C++ header, so running the executable does
not require downloading schemas or locating this directory at runtime.

# Third-Party Licenses

## Cyclo Duck MJCF assets

- MD walking reference: Pollen Robotics, revision
  `273afe0b31c4ab365b9ff806a927b63ac92b5ddd`.
- Adapted robot/scene definitions and four reference collision meshes:
  Apache-2.0, Copyright 2026 Pollen Robotics. Full original license:
  `source/assets/robots/cyclo_duck/LICENSE.md-reference`.
- Cyclo CAD visuals and inertial data: revision
  `d1151ac4a12ce2067b774dae6c00f06eaebc5456`, assembly `Cyclo_Duck`.
  These retain the original Proprietary declaration from
  `third_party/cyclo_duck/cyclo_duck_description/Cyclo_Duck/LICENSE`.
  Re-encoded MSH visuals retain that declaration in
  `source/assets/robots/cyclo_duck/visual/LICENSE`.
- See `source/assets/robots/cyclo_duck/README.md` for transformations, source
  paths and the distinction between reference contact geometry
  and Cyclo visual geometry.

This project uses third-party open-source software and includes code adapted
from third-party open-source projects.

## mujocolab/mjlab

- Source: https://github.com/mujocolab/mjlab
- Version: 1.2.0
- License: Apache-2.0
- Used as: The core simulation and reinforcement-learning framework

```text
Copyright 2025, The mjlab Developers
```

MJLab and cyclo_mjlab are both distributed under the Apache License 2.0. The
license text shared by both projects is provided in `LICENSE`.

## HybridRobotics/whole_body_tracking

- Source: https://github.com/HybridRobotics/whole_body_tracking
- License: MIT
- Used in:
  - `source/tasks/mimic`
  - `scripts/tools/motion/csv_to_npz.py`

The mimic task stack and motion-conversion workflow adapt reference-motion
tracking components from `whole_body_tracking`.

```text
Copyright (c) 2024, The Isaac Lab Project Developers.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

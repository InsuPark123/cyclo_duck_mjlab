# cyclo_duck_mjlab

[![MuJoCo](https://img.shields.io/badge/MuJoCo-3.10.0-silver.svg)](https://mujoco.readthedocs.io/en/3.10.0/)
[![MJLab](https://img.shields.io/badge/MJLab-1.3.0-silver.svg)](https://github.com/mujocolab/mjlab)
[![Python](https://img.shields.io/badge/python-3.12-blue.svg)](https://docs.python.org/3/whatsnew/3.12.html)
[![Linux platform](https://img.shields.io/badge/platform-linux--64-orange.svg)](https://releases.ubuntu.com/22.04/)
[![License](https://img.shields.io/badge/license-Apache2.0-yellow.svg)](https://opensource.org/license/apache-2-0)

## Overview

`cyclo_duck_mjlab` is based on [ROBOTIS cyclo_mjlab](https://github.com/ROBOTIS-GIT/cyclo_mjlab), built on
[MJLab](https://github.com/mujocolab/mjlab) and
[MuJoCo](https://mujoco.org/). It provides reinforcement learning environments,
task configurations, and motion-processing tools for developing locomotion and
motion-imitation policies for the ROBOTIS K1 humanoid robot and walking policies
for Cyclo Duck.

The repository currently provides:

- **Velocity**: flat-ground velocity-tracking locomotion
- **Mimic**: Dance1 and Dance2 reference-motion tracking
- Automatic generation of `params/sim2real.yaml` for the supported K1 PD tasks
- Automatic export of `exported/policy.onnx` and `exported/policy.pt` during play

> [!IMPORTANT]
> This repository currently uses MJLab 1.3.0, MuJoCo 3.10.0, and Python 3.12.

Cyclo Duck MJCF assets, a [14-joint BAM configuration](source/assets/robots/cyclo_duck/README.md),
and the [flat walking task](docs/cyclo_duck_walking.md) are available. See the
[policy input/output contract](docs/cyclo_duck_policy_io.md) for observation
ordering, HOME-relative actions and control timing, and the
[single-joint BAM validation report](docs/bam_single_joint_validation.md) for
the aligned runtime, measured results and the known upstream M6 friction difference.

### Locomotion

![Locomotion demo](docs/videos/k1_rl_locomotion_mjlab.webp)

### Motion imitation

![Motion-imitation demo](docs/videos/k1_mimic_dance_mjlab.webp)

## Installation (Docker)

Docker provides a consistent environment with the required Python packages,
MuJoCo libraries, and GPU configuration pre-installed. Conda and a host Python
environment are not required.

### Prerequisites

- Docker Engine with Docker Compose
- NVIDIA GPU with an appropriate NVIDIA driver
- NVIDIA Container Toolkit
- Git and access to this repository

### Steps

1. Clone the repository with its submodules:

   ```bash
   git clone --recurse-submodules git@github.com:InsuPark123/cyclo_duck_mjlab.git
   cd cyclo_duck_mjlab
   ```

   If the repository was cloned without submodules, initialize them separately:

   ```bash
   git submodule update --init --recursive
   ```

2. Build the Docker image when needed and start the container:

   ```bash
   ./docker/container.sh start
   ```

3. Enter the running container:

   ```bash
   ./docker/container.sh enter
   ```

After entering the container, the training and playback commands below can be
run directly from `/workspace/cyclo_duck_mjlab`. This path mounts the host
`cyclo_duck_mjlab` checkout. The Compose project uses its own pip and Warp cache
volumes, separate from the original `cyclo_mjlab` environment.

The default container name is `cyclo_duck`, the image is `cyclo-duck:latest`,
and the Compose project name is `cyclo_duck_mjlab`. Override the container or
image with `CYCLO_DUCK_CONTAINER` or `CYCLO_DUCK_IMAGE`, respectively.

### Docker commands

| Command | Description |
| --- | --- |
| `./docker/container.sh start` | Build the image if needed, initialize submodules, and start the container |
| `./docker/container.sh enter` | Open an interactive shell in the running container |
| `./docker/container.sh stop` | Stop the container |
| `./docker/container.sh logs` | Follow the container logs |
| `./docker/container.sh clean` | Remove the container and image while preserving cache volumes |

The Docker image includes:

- Python 3.12
- MJLab 1.3.0
- MuJoCo 3.10.0 and MuJoCo Warp 3.8.1
- PyTorch 2.9.1 and RSL-RL 5.0.1
- BAM M6 from the pinned Rhoban revision in `pyproject.toml`
- Warp 1.12.0
- All dependencies declared in `pyproject.toml`

The project source and `logs/` directory are shared with the host, so training
results remain available after the container is removed.

## Try Examples

### Cyclo Duck flat walking

```bash
python scripts/reinforcement_learning/train.py Cyclo-Velocity-Flat-Duck-v0 \
  --env.scene.num-envs 32 --agent.max-iterations 10
```

This is a short training check, not a trained walking policy. Logs and checkpoints
use `logs/rsl_rl/cyclo_duck_velocity/`. BAM training saves `env.yaml` and
`agent.yaml`; the existing K1 PD `sim2real.yaml` schema does not describe BAM.

### Velocity

#### Train

```bash
python scripts/reinforcement_learning/train.py Cyclo-Velocity-Flat-K1-Rev1-v0 \
  --env.scene.num-envs 4096
```

#### Play

```bash
python scripts/reinforcement_learning/play.py Cyclo-Velocity-Flat-K1-Rev1-v0 \
  --checkpoint-file logs/rsl_rl/k1_velocity/<run>/model_<iteration>.pt \
  --num-envs 1
```

### Mimic

#### Train Dance1

```bash
python scripts/reinforcement_learning/train.py Cyclo-Mimic-K1-Rev1-Dance1 \
  --env.scene.num-envs 4096
```

#### Train Dance2

```bash
python scripts/reinforcement_learning/train.py Cyclo-Mimic-K1-Rev1-Dance2 \
  --env.scene.num-envs 4096
```

#### Play Dance1

```bash
python scripts/reinforcement_learning/play.py Cyclo-Mimic-K1-Rev1-Dance1 \
  --checkpoint-file logs/rsl_rl/k1_mimic/<run>/model_<iteration>.pt \
  --num-envs 1
```

To display the reference trajectory next to the trained policy during playback:

```bash
python scripts/reinforcement_learning/play.py Cyclo-Mimic-K1-Rev1-Dance1 \
  --checkpoint-file logs/rsl_rl/k1_mimic/<run>/model_<iteration>.pt \
  --num-envs 1 \
  --reference-ghost-offset 1.4,0,0
```

#### Play Dance2

```bash
python scripts/reinforcement_learning/play.py Cyclo-Mimic-K1-Rev1-Dance2 \
  --checkpoint-file logs/rsl_rl/k1_mimic/<run>/model_<iteration>.pt \
  --num-envs 1
```

Replace `<run>` and `<iteration>` with the timestamped run directory and model
iteration to use.

## Motion Utilities

The motion tools convert K1 CSV motion data into the MuJoCo-ordered NPZ format
and validate or replay the converted trajectory:

```bash
python scripts/tools/motion/csv_to_npz.py --help
python scripts/tools/motion/replay_npz.py --help
```

## Cyclo Duck MJCF

The Cyclo Duck model uses Cyclo CAD visuals/inertias and the MD walking model's
14-joint interface, fixed jaw, floating base, joint limits, HOME pose and sensors.
The robot and scene XML files are in `source/assets/robots/cyclo_duck/`.
See the [asset documentation](source/assets/robots/cyclo_duck/README.md) for
model details and source provenance. The Cyclo Duck RL task and BAM training
actuator are not registered yet.

## License

This repository is licensed under the
[Apache License 2.0](LICENSE).

### Third-party components

- **MJLab**: Apache License 2.0; see
  [THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md)
- **HybridRobotics/whole_body_tracking**: MIT License; see
  [THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md)
- **ROBOTIS AI Sapiens K1 Rev.1 assets**: Apache License 2.0; see
  [LICENSE.ai_sapiens](source/assets/robots/robotis_k1/LICENSE.ai_sapiens)
- **Cyclo Duck / MD model assets**: CAD-derived assets retain their original
  Proprietary declaration; MD adaptations and reference collision meshes are
  Apache-2.0. See [THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md).

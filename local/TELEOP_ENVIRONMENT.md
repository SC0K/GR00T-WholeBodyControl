# Local teleoperation Python environment

Created on 2026-09-29 in `.venv_teleop` with Python 3.10.12.
This is the current Python setup for full decoupled WBC teleoperation;
`SETUP_LOCAL.md` describes a separate SONIC simulation/native-controller setup.

## Activate

From the repository root, in each Bash terminal:

```bash
source local/activate-teleop.sh
```

This sources the existing `/opt/ros/humble/setup.bash`, then activates
`.venv_teleop`. For Python-only use, `source .venv_teleop/bin/activate` also works.
The repository's `local/start-sim.sh` expects a separate `.venv_sim`; to use
this environment, invoke the Python entry point directly.

## Installed

- Editable `decoupled_wbc[full,dev]` and `gear_sonic[teleop,sim]`.
- Editable Unitree SDK 2 and the compiled XRoboToolkit Python binding.
- PyTorch 2.6.0 + CUDA 12.4, verified on the RTX 4090.
- NumPy 1.26.4, SciPy 1.15.3, MuJoCo 3.14.0, Pinocchio 2.7.0,
  Pink, LeRobot (repository-pinned commit), PyVista, and SMPL-X.
- `isaacteleop[cloudxr]` 1.3.132rc1, following the repository's
  `~=1.3.0` prerelease installation instructions.
- Development tools, including pytest, Ruff, Black, CMake, and pybind11.

Exact installed versions: `.venv_teleop/requirements.freeze.txt`.
Validation output: `.venv_teleop/validation-tests.log` and `*-help.log`.
These artifacts live inside the ignored environment directory.

## Verification

- `python -m pip check`: no broken requirements.
- 39 existing robot-model and teleoperation IK tests passed.
- PyTorch CUDA tensor operation succeeded.
- SONIC's 43-DOF G1 robot model loaded; its MuJoCo scene stepped locally.
- ROS 2, Unitree DDS, XRoboToolkit, Isaac Teleop/CloudXR, and LeRobot imports passed.
- Decoupled WBC controller/teleoperation and SONIC simulator CLI help loaded.

```bash
python -m pip check
python -m pytest -q \
  decoupled_wbc/tests/control/robot_model/robot_model_test.py \
  decoupled_wbc/tests/control/teleop/test_teleop_retargeting_ik.py
python decoupled_wbc/control/main/teleop/run_teleop_policy_loop.py --help
python gear_sonic/scripts/run_sim_loop.py --help
```

This sets up Python dependencies. Live robot/headset operation, CloudXR service
launch, optional device-specific SDKs such as UltraLeap, native SONIC controller
builds/TensorRT, and licensed SMPL model assets were not configured or tested.
Isaac Lab training uses a separate environment.

## Recreate

Run from the repository root, using a new/empty environment:

```bash
python3.10 -m venv .venv_teleop
source .venv_teleop/bin/activate
python -m pip install --upgrade pip uv
export UV_CACHE_DIR="$PWD/.venv_teleop/.uv-cache"
GIT_LFS_SKIP_SMUDGE=1 uv pip install \
  -e 'decoupled_wbc[full,dev]' -e 'gear_sonic[teleop,sim]' \
  -e external_dependencies/unitree_sdk2_python \
  cmake pybind11 wheel 'setuptools>=77' smplx
CMAKE_PREFIX_PATH="$(python -m pybind11 --cmakedir)" \
  uv pip install --no-build-isolation \
  -e external_dependencies/XRoboToolkit-PC-Service-Pybind_X86_and_ARM64/
uv pip install 'isaacteleop[cloudxr]~=1.3.0' --prerelease=allow \
  --extra-index-url https://pypi.nvidia.com
python -m pip check
```

The decoupled WBC package metadata was corrected to use inline description
and source-code license metadata, matching SONIC's packaging approach. The
previous parent-directory README/license references caused setuptools to reject
installation. Repository licensing terms in the root `LICENSE` are unchanged.

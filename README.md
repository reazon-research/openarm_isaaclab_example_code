## Installation Guide

### Specification
This repository has been tested on Ubuntu 22.04 and Windows 11.
We do not guarantee that it will work on other platforms.

### ① Install ISAAC SIM and ISAAC LAB
The required versions are:

* **ISAAC SIM v4.5.0**: [Installation Guide](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/install_workstation.html)
* **ISAAC SIM Assets**: [Setup Instructions](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/install_faq.html#isaac-sim-setup-assets-content-pack)
* **ISAAC LAB v2.1.0**: [Binaries Installation Guide](https://isaac-sim.github.io/IsaacLab/v2.1.0/source/setup/installation/binaries_installation.html)

Note that you must use a binary install to access the code inside Isaac Sim and Lab.
From this point on, it is assumed that you have created a virtual environment named env_isaaclab using miniconda or anaconda and will be working within that environment.

### ② Install openarm-isaaclab

```bash
conda activate env_isaaclab
cd IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based
git clone https://github.com/reazon-research/openarm_manipulation.git
cd ../../../..
```

### ③ Run Training
You can run different tasks by specifying the task name using the `--task` argument.

Replace `<TASK_NAME>` with one of the following available tasks:

| Task Description        | Task Name                      |
| ----------------------- | ------------------------------ |
| Reach target position   | `Isaac-Reach-OpenArm-v1`       |
| Lift a cube             | `Isaac-Lift-Cube-OpenArm-v0`   |
| Open a cabinet's drawer | `Isaac-Open-Drawer-OpenArm-v0` |

#### General Format

**Linux:**

```bash
./isaaclab.sh -p ./scripts/reinforcement_learning/rsl_rl/train.py --task <TASK_NAME> --headless
```

**Windows:**

```bash
isaaclab.bat -p .\scripts\reinforcement_learning\rsl_rl\train.py --task <TASK_NAME> --headless
```

### ④ Replay Trained Model
#### General Format

**Linux:**

```bash
./isaaclab.sh -p ./scripts/reinforcement_learning/rsl_rl/play.py --task <TASK_NAME> --num_envs 64
```

**Windows:**

```bash
isaaclab.bat -p .\scripts\reinforcement_learning\rsl_rl\play.py --task <TASK_NAME> --num_envs 64
```

### ⑤ Analyze logs
**Linux:**

```bash
./isaaclab.sh -p -m tensorboard.main --logdir=logs
```

**Windows:**

```bash
isaaclab.bat -p -m tensorboard.main --logdir=logs
```

And open the google and go to `http://localhost:6006/`

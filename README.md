## Installation Guide

### Specification
This repository has been tested on Ubuntu 22.04 and Windows 11.
We do not guarantee that it will work on other platforms.

### ① Install ISAAC SIM and ISAAC LAB
The required version of ISAAC SIM is 4.5.0.
From this point on, it is assumed that you have created a virtual environment named env_isaaclab using pyenv and will be working within that environment.

### ② Install openarm-isaaclab
Navigate to the same directory where IsaacLab is cloned and execute the following commands:

```bash
cd IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based
git clone https://github.com/reazon-research/openarm_manipulation.git
cd ../../../..
```

### ③ Run Training
Run the following command to start training:

```bash
./isaaclab.sh -p ./scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Reach-OpenArm-v1 --headless
```

If you are running this on Windows, type this:
```bash
isaaclab.bat -p .\scripts\reinforcement_learning\rsl_rl\train.py --task Isaac-Reach-OpenArm-v1 --headless
```

### ④ Replay Trained Model
To replay a trained model on Linux, use the following command:

```bash
./isaaclab.sh -p ./scripts/reinforcement_learning/rsl_rl/play.py --task Isaac-Reach-OpenArm-v1 --num_envs 64
```

If you are running this on Windows, type this:
```bash
isaaclab.bat -p .\scripts\reinforcement_learning\rsl_rl\play.py --task Isaac-Reach-OpenArm-v1 --num_envs 64
```

### ⑤ Analyze logs
To compare and analyze trained model on Linux, use the following command:
```bash
./isaaclab.sh -p -m tensorboard.main --logdir=logs
```

If you are running this on Windows, type this:
```bash
isaaclab.bat -p -m tensorboard.main --logdir=logs
```

And open the google and go to `http://localhost:6006/`

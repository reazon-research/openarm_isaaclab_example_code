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
cd IsaacLab
git clone https://github.com/reazon-research/openarm_isaaclab_example_code/tree/main
python -m pip install -e ./openarm_isaaclab_example_code/source
```

After completing this step, the directory structure should look like this:

```
├─IsaacLab
  └─openarm_isaaclab
```

This ensures that the environment is properly registered and recognized.

### ④ Run Training
Run the following command to start training on Linux:

```bash
bash ./isaaclab.sh -p ./openarm_isaaclab_example_code/scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Reach-OpenArm --num_envs 2048 --headless
```

If you are running this on Windows, type this:
```bash
isaaclab.bat -p .\openarm_isaaclab_example_code\scripts\reinforcement_learning\rsl_rl\train.py --task Isaac-Reach-OpenArm --num_envs 2048 --headless
```

### ⑤ Replay Trained Model
To replay a trained model on Linux, use the following command:

```bash
bash ./isaaclab.sh -p ./openarm_isaaclab_example_code/scripts/reinforcement_learning/rsl_rl/play.py --task Isaac-Reach-OpenArm --num_envs 64
```

If you are running this on Windows, type this:
```bash
isaaclab.bat -p .\openarm_isaaclab_example_code\scripts\reinforcement_learning\rsl_rl\play.py --task Isaac-Reach-OpenArm --num_envs 64
```

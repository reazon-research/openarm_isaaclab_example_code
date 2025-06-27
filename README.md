## Installation Guide
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
bash ./isaaclab.sh -p ./scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Reach-OpenArm-v1 --num_envs 2048 --headless
```

If you are running this on Windows, type this:
```bash
isaaclab.bat -p .\scripts\reinforcement_learning\rsl_rl\train.py --task Isaac-Reach-OpenArm-v1 --num_envs 2048 --headless
```

### ⑤ Replay Trained Model
To replay a trained model on Linux, use the following command:

```bash
bash ./isaaclab.sh -p ./scripts/reinforcement_learning/rsl_rl/play.py --task Isaac-Reach-OpenArm-v1 --num_envs 64
```

If you are running this on Windows, type this:
```bash
isaaclab.bat -p .\scripts\reinforcement_learning\rsl_rl\play.py --task Isaac-Reach-OpenArm-v1 --num_envs 64
```


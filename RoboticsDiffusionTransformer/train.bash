#!/bin/bash

# Run this script with: sbatch -J thermal_mixer train.bash

#SBATCH --partition gpu
#SBATCH --nodes 1
#SBATCH --ntasks 16
#SBATCH --mem 64G
#SBATCH --gres=gpu:1
#SBATCH --time 16:00:00
#SBATCH --comment CLI
#SBATCH --output logs/stdout.%j
#SBATCH --error logs/stderr.%j

# LEROBOT_NUM_EPISODES=20
EXP_NAME=$SLURM_JOB_NAME-32-$SLURM_JOB_ID
LEROBOT_TASK=${LEROBOT_TASK:-$SLURM_JOB_NAME}

if [[ "$LEROBOT_TASK" == thermal_cycler_combined* ]]; then
    FLAVOR_SUFFIX=${LEROBOT_TASK#thermal_cycler_combined}
    export LEROBOT_MULTI_TASKS=${LEROBOT_MULTI_TASKS:-thermal_cycler_close${FLAVOR_SUFFIX},thermal_cycler_open${FLAVOR_SUFFIX}}
else
    unset LEROBOT_MULTI_TASKS
fi

LEROBOT_TASK=$LEROBOT_TASK LEROBOT_ROOT="$PWD/../openpi" EXP_NAME=$EXP_NAME CUDA_VISIBLE_DEVICES=0 WANDB_API_KEY= \
    ~/miniforge3/condabin/conda run -n rdt --no-capture-output --live-stream bash finetune.sh

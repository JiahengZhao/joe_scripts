#!/usr/bin/bash

# Target folder for all repos
BASE_DIR=~/g1_ws/src
# mkdir -p "$BASE_DIR"

# Define repos and branches
declare -A repos=(
    ["unitree-g1-moveit"]="git@git.anitron.com:anitron/g1/unitree-g1-moveit.git|main"
    ["unitree-g1-task-planner"]="git@git.anitron.com:anitron/g1/unitree-g1-task-planner.git|dev"
    ["unitree_g1_interfaces"]="git@git.anitron.com:anitron/g1/unitree_g1_interfaces.git|main"
    ["unitree_g1_perception"]="git@git.anitron.com:anitron/g1/unitree_g1_perception.git|dev"
    ["unitree_g1_demo"]="git@git.anitron.com:anitron/g1/unitree_g1_demo.git|training_plate"
    ["unitree_g1_ros2_control"]="git@git.anitron.com:anitron/g1/unitree_g1_ros2_control.git|main"
    ["unitree_g1_description"]="git@git.anitron.com:anitron/g1/unitree_g1_description.git|main"
    ["unitree_g1_vla"]="git@git.anitron.com:anitron/g1/unitree-g1-vla.git|main"
    # ["unitree_g1_docker"]="git@git.anitron.com:anitron/g1/g1-docker.git|main"
    ["fast_livo"]="git@git.anitron.com:anitron/g1/fast-livo2.git|ros2"
    ["unitree-g1-ros2"]="https://git.anitron.com/anitron/g1/unitree-g1-ros2.git|jazzy"
    
)

cd "$BASE_DIR"

for repo in "${!repos[@]}"; do
    IFS="|" read -r url branch <<< "${repos[$repo]}"

    if [ -d "$repo/.git" ]; then
        echo "🔄 Updating $repo ..."
        git -C "$repo" fetch origin
        git -C "$repo" checkout "$branch"
        git -C "$repo" pull origin "$branch"
    else
        echo "⬇️ Cloning $repo ..."
        git clone -b "$branch" "$url" "$repo"
    fi
done


#!/usr/bin/env bash
# or use: #!/usr/bin/env zsh
# Compatible with both bash and zsh
#
# Usage: ./build_ws_locally.sh [MODE]
#   MODE: simulation | local | all (default: all)

# --- Detect shell and set base directory ---
if [ -n "$ZSH_VERSION" ]; then
  SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
elif [ -n "$BASH_VERSION" ]; then
  SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
else
  echo "Unsupported shell. Please run with bash or zsh."
  exit 1
fi

# --- Configurable paths ---
WS_DIR="$SCRIPT_DIR/g1_ws"
SRC_DIR="$WS_DIR/src"
DOCKER_DIR="$WS_DIR/docker"
SDK_DIR="$WS_DIR/sdk"
# THIRDPARTY_SRC="$SRC_DIR/thirdparty"

# --- Colors ---
GREEN="\033[0;32m"
YELLOW="\033[1;33m"
RED="\033[0;31m"
NC="\033[0m"

# --- Function for logging ---
log() { echo -e "${GREEN}[INFO]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
error() { echo -e "${RED}[ERROR]${NC} $1"; }

# --- Parse deployment mode ---
MODE="${1:-all}"
log "Deployment mode: $MODE"

# --- 1. Create workspace folders ---
mkdir -p "$SRC_DIR" "$SDK_DIR" #"$THIRDPARTY_SRC"
log "Workspace structure created."

# --- 2. Clone docker folder ---
declare -A dockerRepo="git@git.anitron.com:anitron/g1/g1-docker.git|main"
IFS="|" read -r url branch <<< "${dockerRepo}"
if [ -d "$DOCKER_DIR/.git" ]; then
    log "🔄 Updating $dockerRepo ..."
    cd $DOCKER_DIR
    git fetch origin
    git checkout "$branch"
    git pull origin "$branch"
else
    log "⬇️ Cloning $dockerRepo ..."
    git clone -b $branch $url $DOCKER_DIR
fi

# --- 3. Clone repositories if not already cloned ---
cd "$SRC_DIR"

# src Repos - format: "url|branch|tags" (tags: comma-separated)
declare -A srcRepos=(
    ["unitree_g1_description"]="git@git.anitron.com:anitron/g1/unitree_g1_description.git|main|simulation,local,all"
    ["unitree_g1_moveit"]="git@git.anitron.com:anitron/g1/unitree-g1-moveit.git|main|simulation,local,all"
    ["unitree_g1_task_planner"]="git@git.anitron.com:anitron/g1/demos/unitree-g1-task-planner.git|dev|simulation,local,all"
    ["unitree_g1_vla"]="git@git.anitron.com:anitron/g1/unitree-g1-vla.git|main|simulation,local,all"
    ["unitree_sim_isaaclab"]="git@git.anitron.com:anitron/g1/unitree_sim_isaaclab.git|main|simulation,all"
    ["unitree_g1_pick_place"]="git@git.anitron.com:anitron/g1/demos/unitree_g1_pick_place.git|main|local,all"
    ["unitree_g1_interfaces"]="git@git.anitron.com:anitron/g1/demos/unitree_g1_interfaces.git|main|local,all"
    ["unitree_g1_perception"]="git@git.anitron.com:anitron/g1/unitree_g1_perception.git|dev|local,all"
    ["unitree_g1_ros2_control"]="git@git.anitron.com:anitron/g1/unitree_g1_ros2_control.git|main|local,all"
    ["fast_livo"]="git@git.anitron.com:anitron/g1/fast-livo2|ros2|local,all"
    ["rpg_vikit"]="https://github.com/integralrobotics/rpg_vikit.git|ros2|local,all"
    ["livox_ros_driver2"]="git@git.anitron.com:anitron/g1/livox_ros_driver2.git|jazzy|local,all"
)
for repo in "${!srcRepos[@]}"; do
    IFS="|" read -r url branch tags <<< "${srcRepos[$repo]}"

    # Filter by mode
    if [ "$MODE" != "all" ]; then
        if [[ ! ",$tags," =~ ,$MODE, ]]; then
            warn "⏭️  Skipping $repo (not in '$MODE' mode)"
            continue
        fi
    fi

    if [ -d "$repo/.git" ]; then
        log "🔄 Updating $repo ..."
        git -C "$repo" fetch origin
        git -C "$repo" checkout "$branch"
        git -C "$repo" pull origin "$branch"
    else
        log "⬇️ Cloning $repo ..."
        git clone -b "$branch" "$url" "$repo"
    fi
done

# --- 4. Install ROS dependencies.
sudo apt install ros-$ROS_DISTRO-sophus \
    ros-$ROS_DISTRO-hardware-interface  \
    ros-$ROS_DISTRO-serial-driver \
    ros-$ROS_DISTRO-asio-cmake-module \
    ros-$ROS_DISTRO-controller-interface \
    ros-$ROS_DISTRO-moveit \
    ros-$ROS_DISTRO-compressed-image-transport \
    ros-$ROS_DISTRO-compressed-depth-image-transport \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-rmw-zenoh-cpp \
    ros-$ROS_DISTRO-rosidl-generator-dds-idl \
    ros-$ROS_DISTRO-controller-manager \
    ros-$ROS_DISTRO-ros2-controllers \

# --- 4. Source ROS2 environment ---
cd "$WS_DIR"
if [[ -f "/opt/ros/$ROS_DISTRO/setup.sh" ]]; then
  source /opt/ros/$ROS_DISTRO/setup.sh
else
  error "ROS2 $ROS_DISTRO not found!"
  exit 1
fi

if [[ -n "$ZSH_VERSION" ]]; then
    if [[ -f "/opt/ros/$ROS_DISTRO/setup.zsh" ]]; then
        source /opt/ros/$ROS_DISTRO/setup.zsh
    fi
elif [[ -n "$BASH_VERSION" ]]; then
    if [[ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
        source /opt/ros/$ROS_DISTRO/setup.bash
    fi
fi

# --- 5. Access Virtual Environment ---
if [[ -f "$WS_DIR/../env.sh" ]]; then
    log "Activating external environment (env.sh)"
    source "$WS_DIR/../env.sh"
elif [[ -f "$WS_DIR/.venv/bin/activate" ]]; then
    log "Activating local virtual environment (.venv)"
    source "$WS_DIR/.venv/bin/activate"
else
    log "Creating virtual environment"
    python3 -m venv $WS_DIR/.venv
    source "$WS_DIR/.venv/bin/activate"
    pip3 install catkin_pkg empy numpy==1.26.4 lark
fi

# --- 6. Build SDKs ---
cd "$SDK_DIR"
# Ignore colcon build
if [[ ! -f "COLCON_IGNORE" ]]; then
    warn "Ignore SDK in Colcon..."
    touch COLCON_IGNORE
fi
# Build Livox SDK
declare -A sdkRepos=(
    ["Livox_SDK2"]="git@git.anitron.com:anitron/g1/Livox_SDK2.git|ubuntu24|simulation,local,all"
    ["unitree_sdk2"]="git@github.com:unitreerobotics/unitree_sdk2.git|main|simulation,local,all"
    ["unitree_sdk2_python"]="git@github.com:unitreerobotics/unitree_sdk2_python.git|master|simulation,local,all"
    ["cyclonedds"]="https://github.com/eclipse-cyclonedds/cyclonedds.git|releases/0.10.x|simulation,local,all"
)
for repo in "${!sdkRepos[@]}"; do
    IFS="|" read -r url branch tags <<< "${sdkRepos[$repo]}"

    if [ -d "$repo/.git" ]; then
        log "🔄 Updating $repo ..."
        git -C "$repo" fetch origin
        git -C "$repo" checkout "$branch"
        git -C "$repo" pull origin "$branch"
    else
        log "⬇️ Cloning $repo ..."
        git clone -b "$branch" "$url" "$repo"
    fi
done

# Build Livox_SDK2
if [[ -d "Livox_SDK2" ]]; then
    log "Building Livox SDK2..."
    cd Livox_SDK2
    mkdir -p build && cd build
    cmake .. -DCMAKE_INSTALL_PREFIX="$SDK_DIR/livox_sdk2"
    make -j$(nproc)
    make install
    log "Livox SDK2 installed to $SDK_DIR"
    cd "$SDK_DIR"
fi

# Build Unitree SDK
if [[ -d "unitree_sdk2" ]]; then
    log "Building unitree_sdk2..."
    cd unitree_sdk2
    mkdir -p build && cd build
    cmake .. -DCMAKE_INSTALL_PREFIX="$SDK_DIR/unitree_robotics"
    make -j$(nproc)
    make install
    log "unitree_robotics installed to $SDK_DIR"
    cd "$SDK_DIR"
fi

# Build CycloneDDS
if [[ -d "cyclonedds" ]]; then
    log "Building cyclonedds..."
    cd cyclonedds
    mkdir build && cd build
    cmake .. -DCMAKE_INSTALL_PREFIX="$SDK_DIR/cyclonedds/install"
    make -j$(nproc)
    make install
    # cmake --build . --target "$SDK_DIR/cyclonedds/install" 
    cd "$SDK_DIR"
fi

# --- 7. Build workspace ---
cd $WS_DIR
log "Starting colcon build..."
colcon build --symlink-install
if [ -n "$ZSH_VERSION" ]; then
    if [ -f "install/setup.zsh" ]; then
        source install/setup.zsh
    fi
elif [ -n "$BASH_VERSION" ]; then
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
    fi
fi
log "Build complete ✅"

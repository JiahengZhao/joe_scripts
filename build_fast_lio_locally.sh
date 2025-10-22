#!/usr/bin/env bash
# or use: #!/usr/bin/env zsh
# Compatible with both bash and zsh

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
SDK_DIR="$WS_DIR/sdk"
THIRDPARTY_SRC="$SRC_DIR/thirdparty"
THIRDPARTY_INSTALL="$WS_DIR/thirdparty_install"

# --- Colors ---
GREEN="\033[0;32m"
YELLOW="\033[1;33m"
RED="\033[0;31m"
NC="\033[0m"

# --- Function for logging ---
log() { echo -e "${GREEN}[INFO]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
error() { echo -e "${RED}[ERROR]${NC} $1"; }

# --- 1. Create workspace folders ---
mkdir -p "$SRC_DIR" "$SDK_DIR" "$THIRDPARTY_SRC" "$THIRDPARTY_INSTALL"
log "Workspace structure created."

# --- 2. Clone repositories if not already cloned ---
cd "$SRC_DIR"

# src Repos
declare -A srcRepos=(
    # ["unitree-g1-moveit"]="git@git.anitron.com:anitron/g1/unitree-g1-moveit.git|main"
    # ["unitree-g1-task-planner"]="git@git.anitron.com:anitron/g1/unitree-g1-task-planner.git|dev"
    # ["unitree_g1_interfaces"]="git@git.anitron.com:anitron/g1/unitree_g1_interfaces.git|main"
    # ["unitree_g1_perception"]="git@git.anitron.com:anitron/g1/unitree_g1_perception.git|dev"
    # ["unitree_g1_demo"]="git@git.anitron.com:anitron/g1/unitree_g1_demo.git|training_plate"
    # ["unitree_g1_ros2_control"]="git@git.anitron.com:anitron/g1/unitree_g1_ros2_control.git|main"
    # ["unitree_g1_description"]="git@git.anitron.com:anitron/g1/unitree_g1_description.git|main"
    # ["unitree_g1_vla"]="git@git.anitron.com:anitron/g1/unitree-g1-vla.git|main"
    # ["unitree_g1_docker"]="git@git.anitron.com:anitron/g1/g1-docker|main"
    ["fast_livo"]="git@git.anitron.com:anitron/g1/fast-livo2|ros2"
    ["rpg_vikit"]="https://github.com/integralrobotics/rpg_vikit.git|ros2"
    ["livox_ros_driver2"]="git@git.anitron.com:anitron/g1/livox_ros_driver2.git|jazzy"
)

for repo in "${!srcRepos[@]}"; do
    IFS="|" read -r url branch <<< "${srcRepos[$repo]}"

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

# Install jazzy sophus
sudo apt install ros-$ROS_DISTRO-sophus

# Build SDKs
cd "$SDK_DIR"
# Build Livox SDK
declare -A sdkRepos=(
    ["Livox_SDK2"]="git@git.anitron.com:anitron/g1/Livox_SDK2.git|ubuntu24"
    ["unitree_sdk2"]="git@github.com:unitreerobotics/unitree_sdk2.git|main"
)
for repo in "${!sdkRepos[@]}"; do
    IFS="|" read -r url branch <<< "${sdkRepos[$repo]}"

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

log "Building Livox SDK2..."
cd Livox_SDK2
mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX="$SDK_DIR/livox_sdk2"
make -j$(nproc)
make install
log "Livox SDK2 installed to $SDK_DIR"
cd "$SDK_DIR"

# Build Unitree SDK
cd unitree_sdk2
mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX="$SDK_DIR/unitree_robotics"
make -j$(nproc)
make install
log "unitree_sdk2 installed to $SDK_DIR"

# --- 4. Source ROS2 environment ---
if [ -f /opt/ros/jazzy/setup.sh ]; then
  source /opt/ros/jazzy/setup.sh
else
  error "ROS2 Jazzy not found!"
  exit 1
fi

if [ -n "$ZSH_VERSION" ]; then
    if [ -f /opt/ros/jazzy/setup.zsh ]; then
        source /opt/ros/jazzy/setup.zsh
    fi
elif [ -n "$BASH_VERSION" ]; then
    if [ -f /opt/ros/jazzy/setup.bash ]; then
        source /opt/ros/jazzy/setup.bash
    fi
fi
# --- 5. Build workspace ---
cd "$WS_DIR"
log "Starting colcon build..."
colcon build --symlink-install
log "Build complete ✅"

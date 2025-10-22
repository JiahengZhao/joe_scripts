#!/usr/bin/env zsh
#
# ROS 2 Build Helper Script
# Usage:
#   ./build.bash all
#   ./build.bash pkg1 pkg2 ...
#   ./build.bash clean
#   ./build.bash rebuild pkg1
#
# Features:
#   - Auto sources ROS and workspace
#   - Supports symlink install
#   - Allows selective builds
#   - Colorful status output

set -e

# === CONFIG ===
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$WS_DIR/src"
BUILD_DIR="$WS_DIR/build"
INSTALL_DIR="$WS_DIR/install"
LOG_DIR="$WS_DIR/log"
ROS_DISTRO=${ROS_DISTRO}

# === COLORS ===
C_GREEN="\033[1;32m"
C_YELLOW="\033[1;33m"
C_RED="\033[1;31m"
C_RESET="\033[0m"

# === FUNCTIONS ===
function info() { echo -e "${C_GREEN}[INFO]${C_RESET} $*"; }
function warn() { echo -e "${C_YELLOW}[WARN]${C_RESET} $*"; }
function error() { echo -e "${C_RED}[ERROR]${C_RESET} $*"; }

# === ACTIVATE CONDA/VENV IF EXISTS ===
if [ -f "$WS_DIR/../env.sh" ]; then
    info "Activating external environment (env.sh)"
    source "$WS_DIR/../env.sh"
elif [ -f "$WS_DIR/.venv/bin/activate" ]; then
    info "Activating local virtual environment (.venv)"
    source "$WS_DIR/.venv/bin/activate"
fi

# === SOURCE WORKSPACE IF BUILT ===
#if [ -f "$INSTALL_DIR/setup.zsh" ]; then
#    source "$INSTALL_DIR/setup.zsh"
#fi

# === COMMANDS ===
if [ $# -eq 0 ]; then
    echo "Usage: $0 [all|clean|rebuild|pkg1 pkg2 ...]"
    exit 0
fi

CMD="$1"
shift || true

# === CLEAN FUNCTION ===
clean_packages() {
    if [ $# = 0 ] || [ "$1" = "all" ]; then
        warn "Cleaning entire workspace..."
        rm -rf "$BUILD_DIR" "$INSTALL_DIR" "$LOG_DIR"
        info "✅ Workspace cleaned."
        exit 0
    fi
    warn "Cleaning selected packages: $*"
    for pkg in "$@"; do
        rm -rf "$BUILD_DIR/$pkg" "$INSTALL_DIR/$pkg"
    done
    info "✅ Selected packages cleaned."
}

# === REBUILD FUNCTION ===
rebuild_packages() {
    clean_packages "$@"
    #exec "$0" "$@"
}

case $CMD in
    clean)
        clean_packages "$@"
        exit 0
        ;;

    rebuild)
        rebuild_packages "$@"
        PACKAGES=("$@")
        info "Building selected packages: $CMD $*"
        ;;

    all)
        info "Building all packages in $SRC_DIR"
        PACKAGES=()
        ;;

    *)
        info "Building selected packages: $CMD $*"
        PACKAGES=("$CMD" "$@")
        ;;
esac

# === BUILD ===
BUILD_CMD=(colcon build --symlink-install)

if [ ${#PACKAGES[@]} -eq 0 ]; then
    info "Running: ${BUILD_CMD[*]}"
    "${BUILD_CMD[@]}"
else
    info "Running: ${BUILD_CMD[*]} --packages-select ${PACKAGES[*]}"
    "${BUILD_CMD[@]}" --packages-select "${PACKAGES[@]}"
fi

# === POST BUILD ===
if [ -f "$INSTALL_DIR/setup.zsh" ]; then
    info "Sourcing new workspace environment"
    source "$INSTALL_DIR/setup.zsh"
fi

info "✅ Build finished successfully."

#!/usr/bin/env bash
# Builds (if needed) and runs the C++ port of ../yolo_live.py.
# Optional arguments are passed to the binary. The default engine is selected
# by yolo_live.cpp, while scripts/yolo-basket-gazebo-vision supplies the
# repository's Gazebo topic and YOLO_ENGINE_PATH explicitly.
set -eu
LAUNCHER_DIR=$(CDPATH= cd -- "$(dirname "$0")" && pwd)
cd "$LAUNCHER_DIR"

# Load Repo A's platform-specific build and TensorRT paths when this script is
# invoked directly, outside a higher-level launcher.
REPO_ROOT=$(CDPATH= cd -- "$LAUNCHER_DIR/../.." && pwd)
if [ -f "$REPO_ROOT/scripts/project-env.sh" ]; then
    # shellcheck disable=SC1091
    . "$REPO_ROOT/scripts/project-env.sh"
fi

if [ -n "${TENSORRT_LIB_DIR:-}" ]; then
    export LD_LIBRARY_PATH="/usr/lib/wsl/lib:$TENSORRT_LIB_DIR:/usr/local/cuda/lib64:/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
fi

# Do not reuse a CMake cache created by another checkout. This repository can
# be built from WSL or a Jetson clone, so the cache must belong to this source.
BUILD_DIR=${YOLO_LIVE_BUILD_DIR:-"$LAUNCHER_DIR/build-repo"}
SOURCE_DIR="$LAUNCHER_DIR"
ONNX_PATH=${YOLO_ONNX_PATH:-"$LAUNCHER_DIR/../best_v5.onnx"}
ENGINE_PATH=""
if [ -n "${YOLO_ENGINE_PATH:-}" ]; then
    ENGINE_PATH="$YOLO_ENGINE_PATH"
elif [ -n "${YOLO_ENGINE:-}" ]; then
    ENGINE_PATH="$YOLO_ENGINE"
else
    printf '%s\n' '[run_yolo_live] TensorRT engine is not selected; set YOLO_ENGINE_PATH or YOLO_ENGINE.' >&2
    exit 2
fi

# Gazebo's camera plugin exposes image data only after its streaming switch is
# enabled. Keep this here so direct C++ runs and the higher-level autonomy
# launcher use the same camera startup behavior.
FRAME_SOURCE=""
GAZEBO_TOPIC=""
RUN_ARGS=("$@")
for ((i = 0; i < ${#RUN_ARGS[@]}; i++)); do
    case "${RUN_ARGS[i]}" in
        --frame-source)
            i=$((i + 1))
            FRAME_SOURCE="${RUN_ARGS[i]:-}"
            ;;
        --gazebo-topic)
            i=$((i + 1))
            GAZEBO_TOPIC="${RUN_ARGS[i]:-}"
            ;;
    esac
done

mkdir -p "$BUILD_DIR"
if [ ! -f "$BUILD_DIR/CMakeCache.txt" ] || [ "${YOLO_FORCE_CONFIGURE:-0}" = 1 ]; then
    cmake_version=$(cmake --version | sed -n '1s/.*version //p')
    cmake_major=${cmake_version%%.*}
    cmake_minor=${cmake_version#*.}
    cmake_minor=${cmake_minor%%.*}
    if [ "$cmake_major" -gt 3 ] || { [ "$cmake_major" -eq 3 ] && [ "$cmake_minor" -ge 13 ]; }; then
        cmake -S "$SOURCE_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=RelWithDebInfo >/dev/null
    else
        (cd "$BUILD_DIR" && cmake "$SOURCE_DIR" -DCMAKE_BUILD_TYPE=RelWithDebInfo >/dev/null)
    fi
fi

build_jobs=${YOLO_BUILD_JOBS:-$(nproc)}
if cmake --help 2>/dev/null | grep -q -- '--build <dir>'; then
    cmake --build "$BUILD_DIR" -- -j"$build_jobs"
else
    make -C "$BUILD_DIR" -j"$build_jobs"
fi

cd "$BUILD_DIR"

if [ "${1:-}" = "--build-only" ]; then
    exit 0
fi

if [ "$FRAME_SOURCE" = "gazebo" ] && [ -n "$GAZEBO_TOPIC" ] && command -v gz >/dev/null 2>&1; then
    STREAM_TOPIC="${GAZEBO_TOPIC%/}/enable_streaming"
    if gz topic -l 2>/dev/null | grep -Fqx "$STREAM_TOPIC"; then
        gz topic -t "$STREAM_TOPIC" -m gz.msgs.Boolean -p 'data: 1' >/dev/null 2>&1 ||
            printf '%s\n' "[run_yolo_live] warning: failed to enable Gazebo streaming: $STREAM_TOPIC" >&2
    else
        printf '%s\n' "[run_yolo_live] warning: Gazebo streaming topic not found: $STREAM_TOPIC" >&2
    fi
fi

exec ./yolo_live --onnx "$ONNX_PATH" --engine "$ENGINE_PATH" "$@"

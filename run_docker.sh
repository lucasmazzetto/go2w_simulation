#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage: ./run_docker.sh [--headless] [--cpu-rendering] [--] [command [args...]]

Start Gazebo and RViz, forwarding the Linux X11/XWayland display.
Use --headless to run the simulation without opening windows.
Automatically uses an available NVIDIA or Intel/AMD GPU, including headless LiDAR.
Use --cpu-rendering to force CPU rendering instead.
A supplied command replaces the image's default launch command.

Examples:
  ./run_docker.sh
  ./run_docker.sh --headless
  ./run_docker.sh --cpu-rendering
  ./run_docker.sh bash
  ./run_docker.sh --headless ros2 launch go2w_description description.launch.py

Set GO2W_DOCKER_IMAGE to override the default go2w_simulation:jazzy tag.
The default simulation container is named go2w_simulation.
Supplied commands use automatic names so extra terminals can run alongside it.
Set GO2W_DOCKER_CONTAINER to override the container name for any invocation.
For GUI mode, DISPLAY and an Xauthority cookie file must be available.
EOF
}

headless=false
cpu_rendering=false
while [[ $# -gt 0 ]]; do
    case "$1" in
        --headless)
            headless=true
            shift
            ;;
        --cpu-rendering)
            cpu_rendering=true
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        --)
            shift
            break
            ;;
        -*)
            printf 'Unknown option: %s\n' "$1" >&2
            usage >&2
            exit 2
            ;;
        *)
            break
            ;;
    esac
done

image_tag="${GO2W_DOCKER_IMAGE:-go2w_simulation:jazzy}"
if ! docker image inspect "$image_tag" >/dev/null 2>&1; then
    printf 'Cannot access image %s. Check Docker and run build_docker.sh first.\n' \
        "$image_tag" >&2
    exit 1
fi

docker_args=(
    --rm
    --init
    --interactive
    --network host
    --ipc host
    --user "$(id -u):$(id -g)"
    # Resolve host/GPU group IDs without changing numeric permissions or the image.
    --mount type=bind,src=/etc/group,dst=/etc/group,readonly
    --env HOME=/tmp
    --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
    --env "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}"
    --env "GZ_PARTITION=${GZ_PARTITION:-go2w_${ROS_DOMAIN_ID:-0}}"
)
container_name="${GO2W_DOCKER_CONTAINER:-}"
if [[ -z "$container_name" && $# -eq 0 ]]; then
    container_name=go2w_simulation
fi
if [[ -n "$container_name" ]]; then
    docker_args+=(--name "$container_name")
fi
if [[ -t 0 && -t 1 ]]; then
    docker_args+=(--tty)
fi

# Give interactive Bash sessions a readable prompt and colored common tools.
# Keep scripted and headless commands unchanged.
if [[ -t 0 && -t 1 && $# -gt 0 && "$1" == "bash" ]]; then
    set -- bash --rcfile /opt/go2w_ws/docker/bashrc "${@:2}"
fi

if [[ "$headless" == true ]]; then
    if [[ $# -eq 0 ]]; then
        set -- ros2 launch go2w_gazebo sim.launch.py gui:=false rviz:=false
    fi
else
    if [[ -z "${DISPLAY:-}" ]]; then
        printf 'DISPLAY is not set. Run from a desktop session or use --headless.\n' >&2
        exit 1
    fi
    if [[ ! -d /tmp/.X11-unix ]]; then
        printf 'X11 socket directory /tmp/.X11-unix is missing. Use --headless.\n' >&2
        exit 1
    fi

    xauthority_file="${XAUTHORITY:-$HOME/.Xauthority}"
    if [[ ! -f "$xauthority_file" || ! -r "$xauthority_file" ]]; then
        printf 'Cannot read Xauthority file: %s\nSet XAUTHORITY or use --headless.\n' \
            "$xauthority_file" >&2
        exit 1
    fi
    if [[ "$xauthority_file" != /* ]]; then
        xauthority_file="$PWD/$xauthority_file"
    fi

    docker_args+=(
        --hostname "$(hostname)"
        --env DISPLAY
        --env XAUTHORITY=/tmp/.Xauthority
        --mount type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix,readonly
        --mount "type=bind,src=$xauthority_file,dst=/tmp/.Xauthority,readonly"
    )

fi

rendering=cpu
if [[ "$cpu_rendering" == false ]]; then
    if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1; then
        # A working host driver alone does not imply Docker GPU support.
        # Probe before starting the user's command; never retry that command.
        nvidia_capabilities=compute,utility,graphics,display
        if timeout 15 docker run --rm --gpus all \
            --user "$(id -u):$(id -g)" \
            --env "NVIDIA_DRIVER_CAPABILITIES=$nvidia_capabilities" \
            --entrypoint nvidia-smi "$image_tag" -L >/dev/null 2>&1; then
            rendering=nvidia
            docker_args+=(
                --gpus all
                --env "NVIDIA_DRIVER_CAPABILITIES=$nvidia_capabilities"
                --env __NV_PRIME_RENDER_OFFLOAD=1
                --env __GLX_VENDOR_LIBRARY_NAME=nvidia
            )
        else
            printf 'NVIDIA GPU detected, but Docker GPU access failed. Using CPU rendering.\n' >&2
            printf 'Check the NVIDIA Container Toolkit configuration on the host.\n' >&2
        fi
    else
        # Mesa drivers use DRM render nodes, with the device groups retained
        # even though the container runs as the host user's UID/GID.
        declare -A rendering_groups=()
        for render_node in /dev/dri/renderD*; do
            [[ -c "$render_node" && -r "$render_node" && -w "$render_node" ]] || continue
            vendor_file="/sys/class/drm/${render_node##*/}/device/vendor"
            [[ -r "$vendor_file" ]] || continue
            read -r vendor < "$vendor_file"
            [[ "$vendor" == 0x8086 || "$vendor" == 0x1002 ]] || continue
            docker_args+=(--device "$render_node")
            rendering_groups["$(stat -c '%g' "$render_node")"]=1
            rendering=mesa
        done
        for rendering_group in "${!rendering_groups[@]}"; do
            docker_args+=(--group-add "$rendering_group")
        done
    fi
fi

if [[ "$rendering" == cpu ]]; then
    docker_args+=(--env LIBGL_ALWAYS_SOFTWARE=1)
    printf 'Using CPU rendering.\n' >&2
else
    docker_args+=(--env LIBGL_ALWAYS_SOFTWARE=0)
    printf 'Using %s GPU rendering.\n' "$rendering" >&2
fi

exec docker run "${docker_args[@]}" "$image_tag" "$@"

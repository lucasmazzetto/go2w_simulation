#!/usr/bin/env bash
set -euo pipefail

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
    cat <<'EOF'
Usage: ./build_docker.sh [docker build options]

Build the ROS 2 Jazzy image for Go2-W simulation and visualization.
Extra options, such as --no-cache or --pull, are passed to docker build.
Set GO2W_DOCKER_IMAGE to override the default go2w_simulation:jazzy tag.
EOF
    exit 0
fi

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
image_tag="${GO2W_DOCKER_IMAGE:-go2w_simulation:jazzy}"

exec docker build \
    --tag "$image_tag" \
    --file "$repo_dir/Dockerfile" \
    "$@" \
    "$repo_dir"

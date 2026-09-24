#!/usr/bin/env bash
# Links every lab's ROS 2 packages into ~/ros2_ws/src and does a clean build.
#
# Safe to re-run any time (idempotent) -- new machine, new lab, or after pulling
# package changes. It always removes build/install/log first: colcon does not
# reliably regenerate a package's ament resource hooks (see r7021e_bringup's
# missing-maintainer incident, 2026-09-24) once it has cached a bad identification
# for that package, so a stale partial rebuild can leave a package invisible to
# `ros2 launch`/`ros2 pkg` even though colcon reports it as built. A clean build
# is cheap (a few seconds for this workspace) and removes that failure mode.
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$HOME/ros2_ws"

mkdir -p "$WS/src"

linked_any=false
for pkg_xml in "$REPO_DIR"/lab*-files/ros2_ws/src/*/package.xml; do
    pkg_dir="$(dirname "$pkg_xml")"
    pkg_name="$(basename "$pkg_dir")"
    link="$WS/src/$pkg_name"

    if [ -L "$link" ]; then
        if [ "$(readlink -f "$link")" = "$(readlink -f "$pkg_dir")" ]; then
            continue
        fi
        echo "Repointing stale symlink for $pkg_name"
        rm "$link"
    elif [ -e "$link" ]; then
        echo "error: $link already exists and is not a symlink -- refusing to touch it" >&2
        exit 1
    fi

    ln -s "$pkg_dir" "$link"
    echo "Linked $pkg_name -> $pkg_dir"
    linked_any=true
done

# Fail loudly on the exact bug that caused today's problem, instead of letting
# colcon silently downgrade the package and produce a workspace that "builds"
# but can't be found by ros2 launch.
missing_maintainer=false
for pkg_xml in "$WS"/src/*/package.xml; do
    if ! grep -q "<maintainer" "$pkg_xml"; then
        echo "error: $pkg_xml has no <maintainer> tag -- colcon will silently" >&2
        echo "       fail to identify this as a ROS package and ros2 launch won't find it." >&2
        missing_maintainer=true
    fi
done
if [ "$missing_maintainer" = true ]; then
    exit 1
fi

echo "Clean build..."
rm -rf "$WS/build" "$WS/install" "$WS/log"
( cd "$WS" && colcon build --symlink-install )

echo
echo "Done. In every new terminal: source $WS/install/setup.bash"
echo "(ROS_DOMAIN_ID is already exported for you by ~/.bashrc.)"

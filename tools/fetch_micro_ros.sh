#!/usr/bin/env bash
#
# Fetch the micro_ros_arduino precompiled library into libraries/.
#
# The library is not in the Arduino library index, so sketch.yaml references it
# as a local directory instead. It ships a 22MB libmicroros.a per architecture,
# which is why it is fetched on demand rather than committed.
#
# Usage: tools/fetch_micro_ros.sh [distro] [version]
#   distro  ROS 2 distribution: jazzy (default), humble, kilted, rolling
#   version micro_ros_arduino release, default 2.0.8
#
# Verified on hardware: 2.0.8-jazzy links against m5stack:esp32@3.3.8
# (ESP-IDF v5.5.4) for the plain ESP32 target. The upstream README still lists
# arduino-esp32 v2.0.2 in its support table, but that entry is out of date.

set -euo pipefail

DISTRO="${1:-jazzy}"
VERSION="${2:-2.0.8}"
TAG="v${VERSION}-${DISTRO}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LIB_DIR="${REPO_ROOT}/libraries"
TARGET="${LIB_DIR}/micro_ros_arduino"
URL="https://github.com/micro-ROS/micro_ros_arduino/archive/refs/tags/${TAG}.zip"

if [ -f "${TARGET}/library.properties" ]; then
  have="$(sed -n 's/^version=//p' "${TARGET}/library.properties")"
  if [ "${have}" = "${VERSION}-${DISTRO}" ]; then
    echo "micro_ros_arduino ${have} is already in place."
    exit 0
  fi
  echo "Replacing micro_ros_arduino ${have} with ${VERSION}-${DISTRO}."
fi

echo "Downloading ${TAG}..."
tmp="$(mktemp -d)"
trap 'rm -rf "${tmp}"' EXIT
curl -fsSL -o "${tmp}/mra.zip" "${URL}"

echo "Extracting..."
unzip -q "${tmp}/mra.zip" -d "${tmp}"
extracted="${tmp}/micro_ros_arduino-${VERSION}-${DISTRO}"
if [ ! -d "${extracted}" ]; then
  echo "Unexpected archive layout: $(ls "${tmp}")" >&2
  exit 1
fi

mkdir -p "${LIB_DIR}"
rm -rf "${TARGET}"
mv "${extracted}" "${TARGET}"

echo "Installed micro_ros_arduino ${VERSION}-${DISTRO} to libraries/micro_ros_arduino"
echo "Precompiled archives available for: $(ls "${TARGET}/src" | grep -E '^(esp32|cortex|imxrt|mk[0-9])' | tr '\n' ' ')"

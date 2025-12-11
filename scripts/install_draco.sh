#!/usr/bin/env bash

# Draco를 git clone으로 받아 빌드·설치하는 스크립트
# - 기본 설치 경로: /usr/local (PREFIX로 변경 가능)
# - 기본 버전: 1.5.7 (DRACO_VERSION으로 변경 가능)
# - 기본 소스 위치: REPO_ROOT/.deps/draco (DRACO_SRC_DIR로 변경 가능)

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PREFIX="${PREFIX:-/usr/local}"
DRACO_VERSION="${DRACO_VERSION:-1.5.7}"
DRACO_REPO_URL="${DRACO_REPO_URL:-https://github.com/google/draco.git}"
DRACO_SRC_DIR="${DRACO_SRC_DIR:-"$ROOT_DIR/.deps/draco"}"
DRACO_BUILD_DIR="$DRACO_SRC_DIR/build"

detect_jobs() {
  if command -v nproc >/dev/null 2>&1; then
    nproc
  elif command -v sysctl >/dev/null 2>&1; then
    sysctl -n hw.logicalcpu
  else
    getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4
  fi
}
JOBS="${JOBS:-$(detect_jobs)}"

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "필수 명령어가 없습니다: $1" >&2
    exit 1
  fi
}

require_cmd git
require_cmd cmake
require_cmd c++

mkdir -p "$(dirname "$DRACO_SRC_DIR")"

if [ ! -d "$DRACO_SRC_DIR/.git" ]; then
  rm -rf "$DRACO_SRC_DIR"
  git clone --depth 1 --branch "$DRACO_VERSION" "$DRACO_REPO_URL" "$DRACO_SRC_DIR"
else
  echo "기존 draco 소스를 재사용합니다: $DRACO_SRC_DIR"
fi

cmake -S "$DRACO_SRC_DIR" -B "$DRACO_BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DCMAKE_INSTALL_PREFIX="$PREFIX" \
  -DDRACO_BUILD_UNITY=OFF

cmake --build "$DRACO_BUILD_DIR" -- -j"$JOBS"
cmake --install "$DRACO_BUILD_DIR"

cat <<EOF
[완료] Draco 설치가 끝났습니다.
- 설치 경로: $PREFIX
- 소스 경로: $DRACO_SRC_DIR
- 빌드 경로: $DRACO_BUILD_DIR

lg_draco를 빌드하려면 colcon에서 CMAKE_PREFIX_PATH에 설치 경로가
포함되어야 할 수 있습니다. 예:
  CMAKE_PREFIX_PATH=$PREFIX:\${CMAKE_PREFIX_PATH:-} colcon build --packages-select lg_draco
EOF

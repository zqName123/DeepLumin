#!/bin/bash
# Build & install script for the syslog library.
#
# Usage:
#   ./build.sh                       # install to <repo>/temp/lib
#   ./build.sh /path/to/install      # install to custom location
#   SYSLOG_INSTALL_PREFIX=/p ./build.sh
#
# The install layout is:
#   <prefix>/include/syslog/*.h
#   <prefix>/lib/libsyslog.a
#   <prefix>/lib/cmake/syslog/syslogConfig.cmake

set -e

# Resolve the script directory (common/syslog/) and the workspace root (temp/).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_DIR="${SCRIPT_DIR}"

# Default install prefix: <temp>/lib  (two levels up from common/syslog/).
DEFAULT_PREFIX="$(cd "${SCRIPT_DIR}/../.." && pwd)/lib"

# Allow override via $1 or environment variable.
if [ -n "$1" ]; then
  INSTALL_PREFIX="$1"
elif [ -n "${SYSLOG_INSTALL_PREFIX}" ]; then
  INSTALL_PREFIX="${SYSLOG_INSTALL_PREFIX}"
else
  INSTALL_PREFIX="${DEFAULT_PREFIX}"
fi

BUILD_DIR="${SCRIPT_DIR}/build"

echo "============================================================"
echo " syslog build"
echo "   source  : ${SOURCE_DIR}"
echo "   build   : ${BUILD_DIR}"
echo "   install : ${INSTALL_PREFIX}"
echo "============================================================"

# Fresh build directory for a clean configure (safe to re-run).
rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"

cmake -S "${SOURCE_DIR}" -B "${BUILD_DIR}" \
      -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
      -DCMAKE_BUILD_TYPE=Release

cmake --build "${BUILD_DIR}" --parallel "$(nproc)"

cmake --install "${BUILD_DIR}"

echo "------------------------------------------------------------"
echo " Installation complete."
echo "   libsyslog.a           : ${INSTALL_PREFIX}/lib/libsyslog.a"
echo "   headers               : ${INSTALL_PREFIX}/include/syslog/"
echo "   cmake package config  : ${INSTALL_PREFIX}/lib/cmake/syslog/"
echo "------------------------------------------------------------"
echo ""
echo " Downstream CMake projects can find syslog with:"
echo "   find_package(syslog REQUIRED)"
echo "   target_link_libraries(your_target syslog::syslog)"
echo ""
echo " If not in a standard location, hint the search path with:"
echo "   cmake -Dsyslog_DIR=${INSTALL_PREFIX}/lib/cmake/syslog .."

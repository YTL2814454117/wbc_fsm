#!/usr/bin/env bash
set -euo pipefail

INSTALL_BASE="${INSTALL_BASE:-/opt/qianer/unitree_g1}"
APP_ROOT="${QIANER_G1_ROOT:-${INSTALL_BASE}/app}"
BINARY="${APP_ROOT}/wbc_fsm"
LOG_DIR="${INSTALL_BASE}/shared/logs"
PID_FILE="${INSTALL_BASE}/wbc_fsm.pid"

if [ ! -x "${BINARY}" ]; then
    echo "[start] Controller binary not executable: ${BINARY}" >&2
    exit 1
fi

mkdir -p "${LOG_DIR}"

export QIANER_G1_ROOT="${APP_ROOT}"
export LD_LIBRARY_PATH="${APP_ROOT}/lib:${APP_ROOT}/lib/onnxruntime:${LD_LIBRARY_PATH:-}"

cd "${APP_ROOT}"

if [ "${1:-}" = "--background" ]; then
    LOG_FILE="${LOG_DIR}/wbc_fsm_$(date +%Y%m%d_%H%M%S).log"
    nohup "${BINARY}" >> "${LOG_FILE}" 2>&1 &
    echo "$!" > "${PID_FILE}"
    echo "[start] Started wbc_fsm pid=$(cat "${PID_FILE}") log=${LOG_FILE}"
else
    exec "${BINARY}"
fi

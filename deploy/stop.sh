#!/usr/bin/env bash
set -euo pipefail

INSTALL_BASE="${INSTALL_BASE:-/opt/qianer/unitree_g1}"
PID_FILE="${INSTALL_BASE}/wbc_fsm.pid"
APP_ROOT="${QIANER_G1_ROOT:-${INSTALL_BASE}/app}"

if [ -f "${PID_FILE}" ]; then
    PID="$(cat "${PID_FILE}")"
    if [ -n "${PID}" ] && kill -0 "${PID}" >/dev/null 2>&1; then
        kill "${PID}" || true
        for _ in $(seq 1 30); do
            if ! kill -0 "${PID}" >/dev/null 2>&1; then
                rm -f "${PID_FILE}"
                echo "[stop] Stopped pid=${PID}"
                exit 0
            fi
            sleep 0.2
        done
        kill -9 "${PID}" || true
        rm -f "${PID_FILE}"
        echo "[stop] Force stopped pid=${PID}"
        exit 0
    fi
    rm -f "${PID_FILE}"
fi

pkill -f "${APP_ROOT}/wbc_fsm" >/dev/null 2>&1 || true
echo "[stop] Stop command sent."

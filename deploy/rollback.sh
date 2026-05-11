#!/usr/bin/env bash
set -euo pipefail

INSTALL_BASE="${INSTALL_BASE:-/opt/qianer/unitree_g1}"
SERVICE_NAME="${SERVICE_NAME:-qianer-g1.service}"
PREVIOUS_FILE="${INSTALL_BASE}/previous_release"

if [ ! -f "${PREVIOUS_FILE}" ]; then
    echo "[rollback] No previous release recorded: ${PREVIOUS_FILE}" >&2
    exit 1
fi

PREVIOUS_RELEASE="$(cat "${PREVIOUS_FILE}")"
if [ ! -d "${PREVIOUS_RELEASE}" ]; then
    echo "[rollback] Previous release does not exist: ${PREVIOUS_RELEASE}" >&2
    exit 1
fi

CURRENT_RELEASE=""
if [ -L "${INSTALL_BASE}/app" ]; then
    CURRENT_RELEASE="$(readlink -f "${INSTALL_BASE}/app" || true)"
fi

if command -v systemctl >/dev/null 2>&1 && systemctl list-unit-files "${SERVICE_NAME}" >/dev/null 2>&1; then
    systemctl stop "${SERVICE_NAME}" || true
elif [ -x "${INSTALL_BASE}/scripts/stop.sh" ]; then
    bash "${INSTALL_BASE}/scripts/stop.sh" || true
fi

ln -sfn "${PREVIOUS_RELEASE}" "${INSTALL_BASE}/app.next"
mv -Tf "${INSTALL_BASE}/app.next" "${INSTALL_BASE}/app"

if [ -n "${CURRENT_RELEASE}" ] && [ "${CURRENT_RELEASE}" != "${PREVIOUS_RELEASE}" ]; then
    echo "${CURRENT_RELEASE}" > "${PREVIOUS_FILE}"
fi

if command -v systemctl >/dev/null 2>&1 && systemctl list-unit-files "${SERVICE_NAME}" >/dev/null 2>&1; then
    systemctl restart "${SERVICE_NAME}" || true
else
    bash "${INSTALL_BASE}/scripts/start.sh" --background || true
fi

echo "[rollback] Rolled back to: ${PREVIOUS_RELEASE}"

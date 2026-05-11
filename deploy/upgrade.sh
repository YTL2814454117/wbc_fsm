#!/usr/bin/env bash
set -euo pipefail

INSTALL_MODE=0
if [ "${1:-}" = "--install" ]; then
    INSTALL_MODE=1
    shift
fi

if [ $# -lt 1 ]; then
    echo "Usage: sudo bash ${0} [--install] /path/to/qianer_g1_<version>.tar.gz" >&2
    exit 1
fi

PACKAGE_PATH="$1"
INSTALL_BASE="${INSTALL_BASE:-/opt/qianer/unitree_g1}"
SERVICE_NAME="${SERVICE_NAME:-qianer-g1.service}"

if [ ! -f "${PACKAGE_PATH}" ]; then
    echo "[upgrade] Package not found: ${PACKAGE_PATH}" >&2
    exit 1
fi

if [ "$(id -u)" -ne 0 ] && [[ "${INSTALL_BASE}" == /opt/* ]]; then
    echo "[upgrade] Please run with sudo, or set INSTALL_BASE to a writable directory." >&2
    exit 1
fi

TMP_DIR="$(mktemp -d)"
cleanup() {
    rm -rf "${TMP_DIR}"
}
trap cleanup EXIT

extract_package() {
    tar -xzf "${PACKAGE_PATH}" -C "${TMP_DIR}"
    if [ ! -d "${TMP_DIR}/app" ] || [ ! -f "${TMP_DIR}/manifest.json" ]; then
        echo "[upgrade] Invalid package layout. Expected app/ and manifest.json." >&2
        exit 1
    fi
}

read_manifest_value() {
    local key="$1"
    grep -m1 "\"${key}\"" "${TMP_DIR}/manifest.json" | sed -E 's/.*"[^"]+"[[:space:]]*:[[:space:]]*"([^"]+)".*/\1/'
}

verify_package() {
    if [ -f "${TMP_DIR}/manifest.sha256" ]; then
        echo "[upgrade] Verifying package checksums..."
        (cd "${TMP_DIR}" && sha256sum -c manifest.sha256)
    else
        echo "[upgrade] Warning: manifest.sha256 missing; checksum verification skipped." >&2
    fi
}

stop_current() {
    if command -v systemctl >/dev/null 2>&1 && systemctl list-unit-files "${SERVICE_NAME}" >/dev/null 2>&1; then
        systemctl stop "${SERVICE_NAME}" || true
    elif [ -x "${INSTALL_BASE}/scripts/stop.sh" ]; then
        bash "${INSTALL_BASE}/scripts/stop.sh" || true
    fi
}

prepare_shared_dirs() {
    mkdir -p \
        "${INSTALL_BASE}/releases" \
        "${INSTALL_BASE}/shared/config" \
        "${INSTALL_BASE}/shared/license" \
        "${INSTALL_BASE}/shared/keys" \
        "${INSTALL_BASE}/shared/logs" \
        "${INSTALL_BASE}/scripts"
}

copy_missing_config() {
    local defaults_dir="$1"
    if [ ! -d "${defaults_dir}" ]; then
        return
    fi

    find "${defaults_dir}" -type f | while read -r src; do
        local rel="${src#${defaults_dir}/}"
        local dst="${INSTALL_BASE}/shared/config/${rel}"
        mkdir -p "$(dirname "${dst}")"
        if [ ! -f "${dst}" ]; then
            cp "${src}" "${dst}"
            echo "[upgrade] Installed default config: shared/config/${rel}"
        else
            echo "[upgrade] Preserved existing config: shared/config/${rel}"
        fi
    done
}

prepare_release_links() {
    local release_dir="$1"

    if [ -d "${release_dir}/config" ]; then
        mkdir -p "${release_dir}/config.defaults"
        cp -a "${release_dir}/config/." "${release_dir}/config.defaults/"
        copy_missing_config "${release_dir}/config.defaults"
        rm -rf "${release_dir}/config"
    fi

    if [ -d "${release_dir}/keys" ]; then
        cp -a "${release_dir}/keys/." "${INSTALL_BASE}/shared/keys/" 2>/dev/null || true
        rm -rf "${release_dir}/keys"
    fi

    rm -rf "${release_dir}/license" "${release_dir}/logs"
    ln -s ../../shared/config "${release_dir}/config"
    ln -s ../../shared/keys "${release_dir}/keys"
    ln -s ../../shared/license "${release_dir}/license"
    ln -s ../../shared/logs "${release_dir}/logs"
}

install_runtime_scripts() {
    if [ -d "${TMP_DIR}/deploy" ]; then
        cp -a "${TMP_DIR}/deploy/." "${INSTALL_BASE}/scripts/"
        chmod +x "${INSTALL_BASE}/scripts/"*.sh
    fi

    if [ -f "${TMP_DIR}/deploy/qianer-g1.service" ] && command -v systemctl >/dev/null 2>&1 && [ "$(id -u)" -eq 0 ]; then
        cp "${TMP_DIR}/deploy/qianer-g1.service" "/etc/systemd/system/${SERVICE_NAME}"
        systemctl daemon-reload
        systemctl enable "${SERVICE_NAME}" >/dev/null 2>&1 || true
    fi
}

switch_release() {
    local new_release="$1"
    local previous_release=""
    if [ -L "${INSTALL_BASE}/app" ]; then
        previous_release="$(readlink -f "${INSTALL_BASE}/app" || true)"
    fi

    if [ -n "${previous_release}" ] && [ "${previous_release}" != "${new_release}" ]; then
        echo "${previous_release}" > "${INSTALL_BASE}/previous_release"
    fi

    ln -sfn "${new_release}" "${INSTALL_BASE}/app.next"
    mv -Tf "${INSTALL_BASE}/app.next" "${INSTALL_BASE}/app"
}

start_current() {
    if [ "${QIANER_NO_AUTO_START:-0}" = "1" ]; then
        echo "[upgrade] QIANER_NO_AUTO_START=1, skip starting service."
        return 0
    fi

    if [ ! -f "${INSTALL_BASE}/shared/license/qianer_license.lic" ]; then
        echo "[upgrade] License file is missing: ${INSTALL_BASE}/shared/license/qianer_license.lic"
        echo "[upgrade] Service installed but not started. Activate the robot first, then run:"
        echo "          sudo systemctl start ${SERVICE_NAME}"
        return 0
    fi

    if command -v systemctl >/dev/null 2>&1 && systemctl list-unit-files "${SERVICE_NAME}" >/dev/null 2>&1; then
        systemctl restart "${SERVICE_NAME}"
        systemctl --no-pager --full status "${SERVICE_NAME}" || true
    else
        bash "${INSTALL_BASE}/scripts/start.sh" --background
    fi
}

rollback_on_failure() {
    echo "[upgrade] Upgrade failed. Trying rollback..." >&2
    if [ -x "${INSTALL_BASE}/scripts/rollback.sh" ]; then
        bash "${INSTALL_BASE}/scripts/rollback.sh" || true
    fi
}

main() {
    extract_package
    verify_package

    local version
    version="$(read_manifest_value version)"
    if [ -z "${version}" ]; then
        echo "[upgrade] Missing version in manifest.json" >&2
        exit 1
    fi

    local new_release="${INSTALL_BASE}/releases/${version}"
    if [ -e "${new_release}" ]; then
        new_release="${INSTALL_BASE}/releases/${version}_$(date +%H%M%S)"
    fi

    echo "[upgrade] Installing version ${version} to ${new_release}"
    prepare_shared_dirs
    stop_current

    mkdir -p "${new_release}"
    cp -a "${TMP_DIR}/app/." "${new_release}/"
    prepare_release_links "${new_release}"
    install_runtime_scripts
    switch_release "${new_release}"

    trap rollback_on_failure ERR
    start_current
    trap - ERR

    if [ "${INSTALL_MODE}" -eq 1 ]; then
        echo "[upgrade] Install completed: ${version}"
    else
        echo "[upgrade] Upgrade completed: ${version}"
    fi
    echo "[upgrade] Current app: ${INSTALL_BASE}/app -> $(readlink -f "${INSTALL_BASE}/app")"
}

main

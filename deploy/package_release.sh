#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

BUILD_DIR="${BUILD_DIR:-${PROJECT_ROOT}/build}"
BINARY="${BINARY:-${BUILD_DIR}/wbc_fsm}"
OUTPUT_DIR="${OUTPUT_DIR:-${PROJECT_ROOT}/dist}"
VERSION="${VERSION:-$(date +%Y%m%d_%H%M%S)}"
DEPLOY_IFACE="${DEPLOY_IFACE:-eth0}"
CERT_SRC="${CERT_SRC:-${PROJECT_ROOT}/../qianer_auth_project/keys/ZJUDES.crt}"

STAGE_DIR="$(mktemp -d)"
cleanup() {
    rm -rf "${STAGE_DIR}"
}
trap cleanup EXIT

if [ ! -f "${BINARY}" ]; then
    echo "[package] Missing controller binary: ${BINARY}" >&2
    echo "[package] Build first, or set BINARY=/path/to/wbc_fsm" >&2
    exit 1
fi

mkdir -p "${STAGE_DIR}/app" "${STAGE_DIR}/deploy" "${OUTPUT_DIR}"

cp "${BINARY}" "${STAGE_DIR}/app/wbc_fsm"
chmod +x "${STAGE_DIR}/app/wbc_fsm"

for dir in config model motion_data; do
    if [ -d "${PROJECT_ROOT}/${dir}" ]; then
        cp -a "${PROJECT_ROOT}/${dir}" "${STAGE_DIR}/app/${dir}"
    else
        echo "[package] Warning: missing ${PROJECT_ROOT}/${dir}, skipped."
    fi
done

mkdir -p "${STAGE_DIR}/app/keys" "${STAGE_DIR}/app/license" "${STAGE_DIR}/app/logs"
if [ -f "${CERT_SRC}" ]; then
    cp "${CERT_SRC}" "${STAGE_DIR}/app/keys/ZJUDES.crt"
else
    echo "[package] Warning: certificate not found: ${CERT_SRC}" >&2
    echo "[package] The robot must provide keys/ZJUDES.crt before license verification can pass." >&2
fi

cat > "${STAGE_DIR}/app/config/qianer_auth.json" <<EOF
{
  "cert_path": "keys/ZJUDES.crt",
  "license_path": "license/qianer_license.lic",
  "iface": "${DEPLOY_IFACE}"
}
EOF

for script in install.sh upgrade.sh rollback.sh start.sh stop.sh; do
    cp "${SCRIPT_DIR}/${script}" "${STAGE_DIR}/deploy/${script}"
    chmod +x "${STAGE_DIR}/deploy/${script}"
done
cp "${SCRIPT_DIR}/qianer-g1.service" "${STAGE_DIR}/deploy/qianer-g1.service"

ONNXRUNTIME_ROOT="${ONNXRUNTIME_ROOT:-}"
if [ -z "${ONNXRUNTIME_ROOT}" ]; then
    for candidate in \
        "${PROJECT_ROOT}/onnxruntime-linux-aarch64-1.22.0" \
        "${PROJECT_ROOT}/onnxruntime-linux-x64-1.22.0"; do
        if [ -d "${candidate}/lib" ]; then
            ONNXRUNTIME_ROOT="${candidate}"
            break
        fi
    done
fi

if [ -n "${ONNXRUNTIME_ROOT}" ] && [ -d "${ONNXRUNTIME_ROOT}/lib" ]; then
    mkdir -p "${STAGE_DIR}/app/lib/onnxruntime"
    cp -a "${ONNXRUNTIME_ROOT}/lib/." "${STAGE_DIR}/app/lib/onnxruntime/"
else
    echo "[package] Warning: ONNX Runtime lib directory not found. Robot must already have runtime libraries available." >&2
fi

GIT_COMMIT="unknown"
if command -v git >/dev/null 2>&1 && git -C "${PROJECT_ROOT}" rev-parse --short HEAD >/dev/null 2>&1; then
    GIT_COMMIT="$(git -C "${PROJECT_ROOT}" rev-parse --short HEAD)"
fi

cat > "${STAGE_DIR}/manifest.json" <<EOF
{
  "name": "qianer-g1",
  "version": "${VERSION}",
  "created_at": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
  "git_commit": "${GIT_COMMIT}",
  "binary": "app/wbc_fsm",
  "install_base": "/opt/qianer/unitree_g1",
  "deploy_iface": "${DEPLOY_IFACE}"
}
EOF

(
    cd "${STAGE_DIR}"
    find app deploy -type f -print0 | sort -z | xargs -0 sha256sum > manifest.sha256
)

PACKAGE_NAME="qianer_g1_${VERSION}.tar.gz"
tar -czf "${OUTPUT_DIR}/${PACKAGE_NAME}" -C "${STAGE_DIR}" .

cp "${SCRIPT_DIR}/install.sh" "${OUTPUT_DIR}/install.sh"
cp "${SCRIPT_DIR}/upgrade.sh" "${OUTPUT_DIR}/upgrade.sh"
chmod +x "${OUTPUT_DIR}/install.sh" "${OUTPUT_DIR}/upgrade.sh"

echo "[package] Created ${OUTPUT_DIR}/${PACKAGE_NAME}"
echo "[package] Created ${OUTPUT_DIR}/install.sh and ${OUTPUT_DIR}/upgrade.sh"
echo "[package] Copy all files in ${OUTPUT_DIR} to the USB drive, then run on the robot:"
echo "          sudo bash /media/\$USER/<USB>/install.sh /media/\$USER/<USB>/${PACKAGE_NAME}"
echo "       or sudo bash /media/\$USER/<USB>/upgrade.sh /media/\$USER/<USB>/${PACKAGE_NAME}"

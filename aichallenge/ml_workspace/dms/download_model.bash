#!/usr/bin/env bash

set -euo pipefail

# openpilot's driver monitoring model, pinned to v0.11.1 and verified by SHA-256.
#
# master's copy cannot be fetched: its Git LFS object is missing from comma's
# server ("Object does not exist on the server"), so this pins the last release
# whose object resolves. The input contract has not changed since v0.9.7.

readonly release="v0.11.1"
readonly oid="3e7b31dfbc0a5234f1baf196513b77fc6af12204b8a8ffe8ee0417e48352f316"
readonly size=7494962
readonly script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly destination="${script_dir}/models/dmonitoring_model.onnx"

mkdir -p "${script_dir}/models"

if [[ -f "${destination}" ]] && printf '%s  %s\n' "${oid}" "${destination}" | sha256sum --check --status; then
    echo "[dms-model] Already verified: ${destination}"
    exit 0
fi

echo "[dms-model] Resolving the Git LFS object for openpilot ${release}..."
href="$(curl --fail --silent --show-error \
    -X POST \
    -H 'Accept: application/vnd.git-lfs+json' \
    -H 'Content-Type: application/vnd.git-lfs+json' \
    -d "{\"operation\":\"download\",\"transfers\":[\"basic\"],\"objects\":[{\"oid\":\"${oid}\",\"size\":${size}}]}" \
    https://github.com/commaai/openpilot.git/info/lfs/objects/batch \
    | python3 -c 'import json,sys; o=json.load(sys.stdin)["objects"][0]; sys.exit(o["error"]["message"]) if "error" in o else print(o["actions"]["download"]["href"])')"

temporary="$(mktemp "${script_dir}/models/.dmonitoring.XXXXXX")"
trap 'rm -f "${temporary:-}"' EXIT
echo "[dms-model] Downloading..."
curl --fail --location --retry 3 --output "${temporary}" "${href}"
printf '%s  %s\n' "${oid}" "${temporary}" | sha256sum --check --status || {
    echo "[dms-model] Checksum mismatch" >&2
    exit 1
}
mv "${temporary}" "${destination}"
trap - EXIT
echo "[dms-model] Ready: ${destination}"

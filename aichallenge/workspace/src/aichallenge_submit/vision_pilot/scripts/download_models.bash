#!/usr/bin/env bash

set -euo pipefail

readonly upstream_commit="e93eceaaa70ee4e7b55fd4ecc04c023c163c8098"
readonly base_url="https://raw.githubusercontent.com/soyaoki/autoware_vision_pilot/${upstream_commit}/VisionPilot/modules/models/weights"
readonly script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly output_dir="${script_dir}/../modules/models/weights"

readonly model_manifest="
831ba3b3dcaaf9150d07152d12295a512033853d7e2996e5a0f7c4dc899bf137  autodrive_int8.onnx
97e78ce4eba2228ae30c16669ce0338314d57f54f7664b8ddbd41746d0bd9abd  autospeed_int8.onnx
7bef4ae9cb1fadd213c747198d6ccc93372e03aa6c263144c5cf75919bf6a607  autosteer_int8.onnx
"

mkdir -p "${output_dir}"

while read -r expected_sha filename; do
    [[ -n "${filename:-}" ]] || continue
    destination="${output_dir}/${filename}"

    if [[ -f "${destination}" ]] && printf '%s  %s\n' "${expected_sha}" "${destination}" | sha256sum --check --status; then
        echo "[vision-pilot-models] Already verified: ${filename}"
        continue
    fi

    temporary="$(mktemp "${output_dir}/.${filename}.XXXXXX")"
    trap 'rm -f "${temporary:-}"' EXIT
    echo "[vision-pilot-models] Downloading ${filename}..."
    curl --fail --location --retry 3 --output "${temporary}" "${base_url}/${filename}"
    printf '%s  %s\n' "${expected_sha}" "${temporary}" | sha256sum --check --status || {
        echo "[vision-pilot-models] Checksum mismatch: ${filename}" >&2
        exit 1
    }
    mv "${temporary}" "${destination}"
    trap - EXIT
done <<< "${model_manifest}"

echo "[vision-pilot-models] All INT8 models are ready in ${output_dir}."

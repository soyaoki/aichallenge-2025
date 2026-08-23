#!/usr/bin/env bash

set -euo pipefail

# Downloads the pinned comma.ai driving model from openpilot's Git LFS storage
# and verifies its SHA-256. Idempotent: an already verified file is left alone.

readonly upstream_commit="084747c75d2cbd23af65ab7a9e770bbd7b98bac9"
readonly base_url="https://github.com/commaai/openpilot/raw/${upstream_commit}/openpilot/selfdrive/modeld/models"
readonly script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly output_dir="${script_dir}/../models"

readonly model_manifest="
659727c4d4839adc4992a254409a54259a8756a743f2d567bf5fdc6579f8009b  driving_supercombo.onnx
"

mkdir -p "${output_dir}"

while read -r expected_sha filename; do
    [[ -n "${filename:-}" ]] || continue
    destination="${output_dir}/${filename}"

    if [[ -f "${destination}" ]] && printf '%s  %s\n' "${expected_sha}" "${destination}" | sha256sum --check --status; then
        echo "[openpilot-models] Already verified: ${filename}"
        continue
    fi

    temporary="$(mktemp "${output_dir}/.${filename}.XXXXXX")"
    trap 'rm -f "${temporary:-}"' EXIT
    echo "[openpilot-models] Downloading ${filename}..."
    curl --fail --location --retry 3 --output "${temporary}" "${base_url}/${filename}"
    printf '%s  %s\n' "${expected_sha}" "${temporary}" | sha256sum --check --status || {
        echo "[openpilot-models] Checksum mismatch: ${filename}" >&2
        exit 1
    }
    mv "${temporary}" "${destination}"
    trap - EXIT
done <<< "${model_manifest}"

echo "[openpilot-models] Model is ready in ${output_dir}."

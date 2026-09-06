#!/usr/bin/env bash
# Sync code and recreate the selected Raspberry Pi services without rebuilding
# their images. Images contain system dependencies; project sources are bind
# mounted and therefore do not belong in the routine deploy path.

set -euo pipefail

PI_HOST="${1:-pi.local}"
PI_USER="${PI_USER:-pi}"
PI_ROOT="${PI_ROOT:-RTK2026}"
SERVICES="${SERVICES:-ros}"
REBUILD_IMAGES="${REBUILD_IMAGES:-0}"

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
REMOTE="${PI_USER}@${PI_HOST}"
COMPOSE_FILE="pi/docker/docker-compose.pi.yml"

# shellcheck disable=SC2206 # SERVICES is intentionally a space-separated list.
services=( ${SERVICES} )
if [ "${#services[@]}" -eq 0 ]; then
    printf 'SERVICES must contain at least one compose service\n' >&2
    exit 2
fi

"${ROOT}/pi/tools/sync_to_pi.sh" "${PI_HOST}"

quoted_services=()
for service in "${services[@]}"; do
    printf -v quoted_service '%q' "${service}"
    quoted_services+=("${quoted_service}")
done
service_args="${quoted_services[*]}"

if [ "${REBUILD_IMAGES}" = "1" ]; then
    printf '\nrebuilding dependency images on %s\n' "${REMOTE}"
    ssh "${REMOTE}" \
        "cd '${PI_ROOT}' && docker compose -f '${COMPOSE_FILE}' build ${service_args}"
fi

printf '\nrecreating %s on %s (image build disabled)\n' "${SERVICES}" "${REMOTE}"
if ! ssh "${REMOTE}" \
    "cd '${PI_ROOT}' && docker compose -f '${COMPOSE_FILE}' up -d --no-build --force-recreate ${service_args}"; then
    printf '%s\n' \
        'Deploy failed. If an image is missing or its dependencies changed, retry with:' \
        "REBUILD_IMAGES=1 SERVICES='${SERVICES}' $0 ${PI_HOST}" >&2
    exit 1
fi


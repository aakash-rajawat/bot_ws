#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPOS_FILE="${WS_ROOT}/third_party.repos"
THIRD_PARTY_DIR="${WS_ROOT}/third_party"

if ! command -v vcs >/dev/null 2>&1; then
  echo "Missing dependency: vcs. Install python3-vcstool." >&2
  exit 1
fi

if ! command -v git >/dev/null 2>&1; then
  echo "Missing dependency: git." >&2
  exit 1
fi

if [[ ! -f "${REPOS_FILE}" ]]; then
  echo "Missing third-party manifest: ${REPOS_FILE}" >&2
  exit 1
fi

mkdir -p "${THIRD_PARTY_DIR}"
touch "${THIRD_PARTY_DIR}/COLCON_IGNORE"

vcs import "${THIRD_PARTY_DIR}" < "${REPOS_FILE}"

shopt -s nullglob
for repo in "${THIRD_PARTY_DIR}"/*; do
  if [[ -d "${repo}/.git" && -f "${repo}/.gitmodules" ]]; then
    git -C "${repo}" submodule update --init --recursive
  fi
done

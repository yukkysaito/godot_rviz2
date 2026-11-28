#!/usr/bin/env bash
set -euo pipefail

# Downloads the official Godot editor (and with --templates, the Linux export templates) of the
# version in GODOT_VERSION (at the repository root, which the extension is built for too) into the
# cache, and prints the editor's path:
#   scripts/get_godot.sh [--templates]

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GODOT_CACHE="${GODOT_CACHE:-${XDG_CACHE_HOME:-$HOME/.cache}/godot_rviz2}"

fail() {
  printf '[%s] ERROR: %s\n' "$(basename "$0")" "$*" >&2
  exit 1
}

godot_version() {
  local version=""

  # (read fails at a last line without a newline, after reading it)
  read -r version <"${REPO_ROOT}/GODOT_VERSION" || [[ -n "$version" ]] || \
    fail "cannot read ${REPO_ROOT}/GODOT_VERSION"
  version="${version%$'\r'}"
  [[ "$version" =~ ^[0-9]+\.[0-9]+(\.[0-9]+)?$ ]] || fail "unexpected Godot version: $version"
  printf '%s\n' "$version"
}

# Downloads release files into DIR and checks them against the release's SHA512 sums
download() {
  local dir="$1"
  shift
  local base_url="https://github.com/godotengine/godot/releases/download/${VERSION}-stable"
  local file=""

  curl -fsSL -o "${dir}/SHA512-SUMS.txt" "${base_url}/SHA512-SUMS.txt"
  for file in "$@"; do
    curl -fsSL -o "${dir}/${file}" "${base_url}/${file}"
    (cd "$dir" && grep " ${file}\$" SHA512-SUMS.txt | sha512sum -c --quiet) || \
      fail "checksum mismatch: ${file}"
  done
}

main() {
  local templates=0
  local dir=""
  local editor=""
  local archive=""

  case "${1:-}" in
    "") ;;
    --templates) templates=1 ;;
    *) fail "usage: $(basename "$0") [--templates]" ;;
  esac

  VERSION="$(godot_version)"
  dir="${GODOT_CACHE}/godot-${VERSION}"
  editor="${dir}/Godot_v${VERSION}-stable_linux.x86_64"
  mkdir -p "$dir"

  if [[ ! -x "$editor" ]]; then
    archive="Godot_v${VERSION}-stable_linux.x86_64.zip"
    printf 'Downloading Godot %s into %s\n' "$VERSION" "$dir" >&2
    download "$dir" "$archive"
    unzip -oq "${dir}/${archive}" -d "$dir"
    rm -f "${dir}/${archive}"
  fi

  if (( templates )) && [[ ! -f "${dir}/templates/linux_release.x86_64" ]]; then
    archive="Godot_v${VERSION}-stable_export_templates.tpz"
    printf 'Downloading the Godot %s export templates into %s\n' "$VERSION" "$dir" >&2
    download "$dir" "$archive"
    unzip -oq "${dir}/${archive}" 'templates/linux_release.*' -d "$dir"
    rm -f "${dir}/${archive}"
  fi

  printf '%s\n' "$editor"
}

main "$@"

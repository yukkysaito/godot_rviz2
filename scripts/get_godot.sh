#!/usr/bin/env bash
set -euo pipefail

# Downloads the official Godot editor (and with --templates, the Linux export templates) of the
# version godot-cpp is pinned to, into the cache, and prints the editor's path:
#   scripts/get_godot.sh [--templates]
# The version comes from the godot-cpp submodule's tag (godot-<version>-stable), so that the editor
# always matches the API the extension is built against.

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GODOT_CPP="${REPO_ROOT}/godot_rviz2/thirdparty/godot-cpp"
GODOT_CACHE="${GODOT_CACHE:-${XDG_CACHE_HOME:-$HOME/.cache}/godot_rviz2}"

fail() {
  printf '[%s] ERROR: %s\n' "$(basename "$0")" "$*" >&2
  exit 1
}

godot_version() {
  local tag=""

  tag="$(git -C "$GODOT_CPP" describe --tags --exact-match 2>/dev/null)" || \
    fail "godot-cpp is not at a release tag (git submodule update --init?)"
  [[ "$tag" =~ ^godot-(.+)-stable$ ]] || fail "unexpected godot-cpp tag: $tag"
  printf '%s\n' "${BASH_REMATCH[1]}"
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

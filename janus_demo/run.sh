#!/usr/bin/env bash
# Janus demonstrator helper.  Usage: ./run.sh <command> [options]
#
#   setup [--dev]   create .venv and install requirements (--dev adds pytest + pygame)
#   doctor [--quick] check Python, packages, ports, main.py, a headless sim and a live UDP link
#   ui [args]       start the console (args go to run_janus.py, e.g. --port 7861 --share)
#   tour [args]     start the console with the hands-free tour already playing
#   share [args]    start the console with a temporary public Gradio link
#   lan [args]      serve the console to phones/laptops on the same Wi-Fi or hotspot
#   test [args]     run the test suite (args go to pytest, e.g. -k planner)
#   logs            show the latest relay/drone process logs
#   clean           remove .venv and caches
#   help            show this text
#
# Linux/macOS: works as is.  Windows: run from Git Bash or WSL
# (or use:  python run_janus.py  directly).
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"
VENV=".venv"
MIN_MAJOR=3
MIN_MINOR=10

c_ok=$'\033[32m'; c_warn=$'\033[33m'; c_err=$'\033[31m'; c_dim=$'\033[2m'; c_off=$'\033[0m'
[ -t 1 ] || { c_ok=""; c_warn=""; c_err=""; c_dim=""; c_off=""; }
say()  { printf '%s\n' "${c_dim}[janus]${c_off} $*"; }
warn() { printf '%s\n' "${c_warn}[janus] $*${c_off}" >&2; }
die()  { printf '%s\n' "${c_err}[janus] $*${c_off}" >&2; exit 1; }

# ------------------------------------------------------------ interpreters
version_ok() {  # $@ = interpreter command
  "$@" -c "import sys; sys.exit(0 if sys.version_info >= (${MIN_MAJOR}, ${MIN_MINOR}) else 1)" >/dev/null 2>&1
}

find_system_python() {
  if [ -n "${PYTHON:-}" ]; then
    version_ok "$PYTHON" && { echo "$PYTHON"; return; }
    die "PYTHON=$PYTHON is older than ${MIN_MAJOR}.${MIN_MINOR}"
  fi
  for c in python3.13 python3.12 python3.11 python3.10 python3 python; do
    if command -v "$c" >/dev/null 2>&1 && version_ok "$c"; then echo "$c"; return; fi
  done
  if command -v py >/dev/null 2>&1 && version_ok py -3; then echo "py -3"; return; fi
  die "Python ${MIN_MAJOR}.${MIN_MINOR}+ not found. Install it from python.org (tick 'Add to PATH' on Windows), or set PYTHON=/path/to/python"
}

venv_python() {
  if [ -x "$VENV/bin/python" ]; then echo "$VENV/bin/python"
  elif [ -x "$VENV/Scripts/python.exe" ]; then echo "$VENV/Scripts/python.exe"
  else echo ""; fi
}

need_venv() {
  local py; py="$(venv_python)"
  [ -n "$py" ] || die "No environment yet. Run:  ./run.sh setup"
  echo "$py"
}

# ------------------------------------------------------------ commands
cmd_setup() {
  local dev=0
  for a in "$@"; do [ "$a" = "--dev" ] && dev=1; done
  local sys_py; sys_py="$(find_system_python)"
  say "using $($sys_py -c 'import sys; print(sys.executable, sys.version.split()[0])')"
  if [ -z "$(venv_python)" ]; then
    say "creating $VENV"
    $sys_py -m venv "$VENV" || die "could not create $VENV (on Debian/Ubuntu: sudo apt install python3-venv)"
  else
    say "$VENV already exists"
  fi
  local py; py="$(venv_python)"
  say "upgrading pip"
  "$py" -m pip install --upgrade pip --quiet
  say "installing requirements (gradio pinned)"
  "$py" -m pip install -r requirements.txt --quiet
  if [ "$dev" = 1 ]; then
    say "installing test tools (pytest, pygame)"
    "$py" -m pip install pytest "pygame>=2.6.1" --quiet || warn "pygame failed to install; parity test will be skipped"
  fi
  printf '%s\n' "${c_ok}[janus] setup done.${c_off} Next:  ./run.sh doctor   then   ./run.sh ui"
}

cmd_doctor() {
  local py; py="$(venv_python)"
  if [ -z "$py" ]; then
    warn "no $VENV yet; checking with the system Python instead (run ./run.sh setup first)"
    py="$(find_system_python)"
  fi
  $py -m janus.doctor "$@"
}

ui_port() {  # value of --port in the arguments, default 7860
  local port=7860 prev=""
  for a in "$@"; do
    [ "$prev" = "--port" ] && port="$a"
    case "$a" in --port=*) port="${a#--port=}" ;; esac
    prev="$a"
  done
  echo "$port"
}

cmd_ui() {
  local py; py="$(need_venv)"
  say "starting Janus at http://127.0.0.1:$(ui_port "$@") (Ctrl+C to stop)"
  exec "$py" run_janus.py "$@"
}

cmd_test() {
  local py; py="$(need_venv)"
  "$py" -c "import pytest" 2>/dev/null || die "pytest missing. Run:  ./run.sh setup --dev"
  "$py" -m pytest tests -q -p no:warnings "$@"
}

cmd_lan() {
  local ip=""
  ip="$( (hostname -I 2>/dev/null | awk '{print $1}') || true)"
  [ -n "$ip" ] || ip="$( (ipconfig getifaddr en0 2>/dev/null) || true)"
  say "serving on your network${ip:+: open http://$ip:$(ui_port "$@") on the other device}"
  cmd_ui --ui-host 0.0.0.0 "$@"
}

cmd_logs() {
  local tmp="${TMPDIR:-/tmp}" latest
  latest="$(ls -dt "$tmp"/janus_logs_* 2>/dev/null | head -1 || true)"
  [ -n "$latest" ] || die "no logs found in $tmp (they appear after the first ./run.sh ui)"
  say "logs in $latest"
  for f in "$latest"/*.log; do printf '\n== %s\n' "$f"; tail -n 25 "$f"; done
}

cmd_clean() {
  say "removing $VENV and caches"
  rm -rf "$VENV" .pytest_cache
  find . -name "__pycache__" -type d -prune -exec rm -rf {} + 2>/dev/null || true
  say "done"
}

cmd_help() { awk 'NR>1 && /^#/ {sub(/^# ?/, ""); print; next} NR>1 {exit}' "$0"; }

cmd="${1:-help}"
shift || true
case "$cmd" in
  setup)  cmd_setup "$@" ;;
  doctor) cmd_doctor "$@" ;;
  ui|start|run) cmd_ui "$@" ;;
  tour)   cmd_ui --tour "$@" ;;
  share)  cmd_ui --share "$@" ;;
  lan)    cmd_lan "$@" ;;
  test)   cmd_test "$@" ;;
  logs)   cmd_logs ;;
  clean)  cmd_clean ;;
  help|-h|--help) cmd_help ;;
  *) warn "unknown command: $cmd"; cmd_help; exit 1 ;;
esac

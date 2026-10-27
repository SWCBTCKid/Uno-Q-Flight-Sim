#!/bin/sh
# Fed to `ssh BOARD sh -s -- REMOTE` by deploy_m1_s1.ps1 (no shell syntax on
# the ssh command line). Creates a fresh, private run directory.
set -eu
remote=${1:?usage: m1_s1_remote_prepare.sh REMOTE_DIR}
if [ -e "$remote" ]; then
    echo "remote path already exists: $remote" >&2
    exit 30
fi
if pgrep -x gx-app >/dev/null 2>&1; then
    echo "a gx-app process is already running" >&2
    exit 31
fi
umask 077
mkdir -p "$remote/bin"
exit 0

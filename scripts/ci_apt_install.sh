#!/usr/bin/env bash
# Installs apt packages on a GitHub runner and survives short mirror/DNS outages (the runner's
# default mirror azure.archive.ubuntu.com sometimes fails to resolve): apt retries per
# download, the whole update+install is repeated with a pause, and from the second attempt on
# the sources point at the main Ubuntu archive instead of the Azure mirror.
#
#     scripts/ci_apt_install.sh gcc-mingw-w64-x86-64 nsis
set -uo pipefail

opts=(-o Acquire::Retries=5 -o Acquire::http::Timeout=30 -o Acquire::https::Timeout=30)

for attempt in 1 2 3 4 5; do
    if [ "$attempt" -ge 2 ]; then
        echo "apt attempt $attempt: switching to archive.ubuntu.com"
        sudo find /etc/apt -type f \( -name '*.list' -o -name '*.sources' \) -exec \
            sed -i 's#azure\.archive\.ubuntu\.com#archive.ubuntu.com#g' {} + 2>/dev/null || true
    fi
    if sudo apt-get "${opts[@]}" update -qq \
        && sudo apt-get "${opts[@]}" install -y -qq --no-install-recommends "$@"; then
        exit 0
    fi
    echo "apt attempt $attempt failed, retrying in $((attempt * 10)) s" >&2
    sleep $((attempt * 10))
done
echo "apt install of $* failed after 5 attempts" >&2
exit 1

#!/usr/bin/env bash
# Installs apt packages on a GitHub runner and survives mirror/DNS outages. The runner image
# points apt at azure.archive.ubuntu.com through /etc/apt/apt-mirrors.txt; when that host does
# not resolve, every apt call hangs for minutes and fails. So before each attempt this picks the
# first mirror host that resolves and rewrites the apt mirror list and sources to it, and every
# attempt is capped with `timeout` so a hanging resolver cannot eat the whole job.
#
#     scripts/ci_apt_install.sh gcc-mingw-w64-x86-64 nsis
set -uo pipefail

mirrors=(azure.archive.ubuntu.com archive.ubuntu.com mirrors.edge.kernel.org us.archive.ubuntu.com)
opts=(-o Acquire::Retries=3 -o Acquire::http::Timeout=20 -o Acquire::https::Timeout=20
    -o Acquire::ForceIPv4=true)

# Prints the first mirror host that resolves and answers on HTTP.
pick_mirror() {
    local host
    for host in "${mirrors[@]}"; do
        if timeout 10 getent hosts "$host" >/dev/null 2>&1 \
            && curl -fsS -o /dev/null --max-time 15 "http://$host/ubuntu/dists/" 2>/dev/null; then
            echo "$host"
            return 0
        fi
        echo "mirror $host not reachable" >&2
    done
    return 1
}

use_mirror() {
    local host="$1" file
    echo "apt mirror: $host"
    for file in /etc/apt/apt-mirrors*.txt /etc/apt/sources.list /etc/apt/sources.list.d/*; do
        [ -f "$file" ] || continue
        sudo sed -i -E "s#(https?://)(azure\.archive\.ubuntu\.com|archive\.ubuntu\.com|us\.archive\.ubuntu\.com|mirrors\.edge\.kernel\.org)/#\1$host/#g" "$file"
    done
}

for attempt in 1 2 3 4 5; do
    if host="$(pick_mirror)"; then
        use_mirror "$host"
    else
        echo "no mirror resolves, trying anyway" >&2
    fi
    if sudo timeout 300 apt-get "${opts[@]}" update -qq \
        && sudo timeout 600 apt-get "${opts[@]}" install -y -qq --no-install-recommends "$@"; then
        exit 0
    fi
    echo "apt attempt $attempt failed, retrying in $((attempt * 15)) s" >&2
    sleep $((attempt * 15))
done
echo "apt install of $* failed after 5 attempts" >&2
exit 1

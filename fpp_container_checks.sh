#!/usr/bin/env bash
# Shared read-only capability check; temporary mounts stay in a private namespace.
check_fpp_source_mounts() {
    docker exec "$1" unshare --mount --propagation private bash -c '
        set -euo pipefail
        probe_dir=$(mktemp -d /tmp/fpp-source-mount.XXXXXX)
        mounted=0
        cleanup() {
            if [[ $mounted -eq 1 ]]; then umount "$probe_dir"; fi
            rmdir "$probe_dir"
        }
        trap cleanup EXIT
        mount --bind /opt/mf_system "$probe_dir"
        mounted=1
        test -x "$probe_dir/sim"
    '
}

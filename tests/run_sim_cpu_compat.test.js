const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const cp = require('node:child_process');

const script = fs.readFileSync(path.resolve(__dirname, '../run_sim.sh'), 'utf8');
const start = script.indexOf('ensure_container() {');
const end = script.indexOf('\n# Mviz ', start);
assert(start >= 0 && end > start);
const ensure = script.slice(start, end);
const harness = `
set -euo pipefail
CONTAINER_NAME=test
docker() {
    printf '%s\\n' "$*" >&2
    if [[ "$1" == inspect ]]; then
        [[ "\${TEST_CONTAINER:-running}" != missing ]] || return 1
        if [[ "\${2:-}" == -f ]]; then
            [[ "\${TEST_CONTAINER:-running}" != stopped ]] && echo true || echo false
        fi
        return 0
    fi
    [[ "$1" != cp ]] || return 1
    if [[ "$*" == *'test -x'* ]]; then
        [[ "\${TEST_CONTAINER:-running}" != unmounted ]]
        return
    fi
    if [[ "$*" == *'grep -q'* ]]; then
        [[ "\${TEST_CUDA:-none}" == container ]]
    elif [[ "$*" == *'dirname '* ]]; then
        echo /usr/lib/x86_64-linux-gnu
    elif [[ "$*" == *'bash -c ldconfig'* ]]; then
        return 1
    fi
}
ldconfig() {
    if [[ "\${TEST_CUDA:-none}" == host ]]; then
        echo 'libcuda.so.1 (libc6) => /usr/lib/x86_64-linux-gnu/libcuda.so.1'
    fi
}
` + ensure + '\nensure_container\necho CONTINUE\n';

for (const cuda of ['none', 'container', 'host']) {
  const result = cp.spawnSync('bash', ['-c', harness], {
    encoding: 'utf8', env: { ...process.env, TEST_CUDA: cuda, TEST_CONTAINER: 'running' },
  });
  assert.equal(result.status, 0, result.stderr);
  assert(result.stdout.includes('CONTINUE'));
  assert(!result.stderr.includes('nvidia-smi'));
  if (cuda === 'none') assert(result.stdout.includes('CUDA'));
  if (cuda === 'host') assert(result.stdout.includes('/usr/lib/x86_64-linux-gnu/'));
}

for (const container of ['missing', 'stopped', 'unmounted']) {
  const result = cp.spawnSync('bash', ['-c', harness], {
    encoding: 'utf8', env: { ...process.env, TEST_CONTAINER: container, TEST_CUDA: 'none' },
  });
  assert.equal(result.status, 1, result.stderr);
  assert(!result.stdout.includes('CONTINUE'));
}

assert(!script.includes('--cpu'));
assert(!script.includes('CPU_MODE'));
assert(!script.includes('-e USE_HANS='));
console.log('PASS: optional CUDA, missing GPU, tolerated CUDA preparation failures, container validation');

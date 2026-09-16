const assert = require('node:assert/strict');
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const cp = require('node:child_process');

const scriptPath = path.resolve(__dirname, '../run_sim.sh');
const script = fs.readFileSync(scriptPath, 'utf8');
const split = script.indexOf('# \u2500\u2500 \u4e3b\u6d41\u7a0b');
assert(split > 0);
const temp = fs.mkdtempSync(path.join(os.tmpdir(), 'run-sim-source-tests-'));
const debug = path.join(temp, 'debug');
const repos = path.join(temp, 'workdir/Devcar/repos_dir');
const source = path.join(repos, 'driving_control');
const worktree = path.join(repos, 'driving_control_0001');
const fixtureScript = path.join(debug, 'run_sim.sh');
// bash -c has no BASH_SOURCE, so supply the fixture script path as $0.
const prefix = script.slice(0, split).replace('${BASH_SOURCE[0]}', '$0');
const baseArgs = ['--container', 'test', '--skip-mviz-login'];
const invoke = (code, options = [], env = {}) => cp.spawnSync(
  'bash', ['-c', prefix + '\n' + code, fixtureScript, ...baseArgs, ...options,
    '--batch', 'unused.txt', 'close-loop', 'LC6-EC65130'],
  { encoding: 'utf8', cwd: repos, env: { ...process.env,
    REQUIRE_KOOPMAN_ACTIVE: '0', KOOPMAN_SIM_MINFER_LIB: '', ...env } },
);

try {
  for (const directory of [debug, source, worktree]) fs.mkdirSync(directory, { recursive: true });
  fs.writeFileSync(path.join(temp, 'sim'), '', { mode: 0o755 });
  for (const directory of [source, worktree]) fs.writeFileSync(path.join(directory, 'Makefile'), '');
  fs.symlinkSync(worktree, path.join(repos, 'selected'));

  for (const [options, expected] of [
    [[], source],
    [['--source-dir', 'driving_control_0001'], worktree],
    [['--source-dir', worktree], worktree],
    [['--source-dir', 'selected'], worktree],
  ]) {
    const result = invoke('printf "%s\\n" "$SOURCE_DIR" "$DEPENDS_JSON" "$CONTAINER_SOURCE_DIR"', options);
    assert.equal(result.status, 0, result.stderr);
    assert.equal(result.stdout, `${expected}\n${expected}/depends.json\n/opt/mf_system${expected.slice(temp.length)}\n`);
  }

  const noMakefile = path.join(temp, 'no-makefile');
  fs.mkdirSync(noMakefile);
  for (const options of [
    ['--source-dir', 'missing'], ['--source-dir', noMakefile],
    ['--source-dir', os.tmpdir()], ['--source-dir'],
  ]) assert.equal(invoke(':', options).status, 1, options.join(' '));

  const capture = path.join(temp, 'docker-args');
  for (const options of [[], ['--source-dir', worktree]]) {
    const result = invoke(`
docker() { printf '%s\\0' "$@" > "$CAPTURE"; }
build_once
`, options, { CAPTURE: capture });
    assert.equal(result.status, 0, result.stderr);
    const args = fs.readFileSync(capture, 'utf8').split('\0').slice(0, -1);
    const code = args[args.indexOf('-c') + 1];
    assert.equal(cp.spawnSync('bash', ['-n'], { input: code }).status, 0);
    const expected = options.length ? worktree : source;
    assert.deepEqual(args.slice(-3), ['bash', `/opt/mf_system${expected.slice(temp.length)}`, temp]);
    assert(code.indexOf('mount --bind /opt/mf_system "$3"') < code.indexOf('mount --bind "$1" "$2"'));
    assert(code.includes('unshare --mount --propagation private'));
    assert(!args.some(arg => /USE_HANS|USE_KTRANS|USE_LOWMU/.test(arg)));
    assert(result.stdout.includes(expected));
  }

  const failed = invoke('docker() { return 7; }\nbuild_once');
  assert.equal(failed.status, 7);
  console.log('PASS: default/relative/absolute/symlink source, validation, container build arguments, build failure');
} finally {
  fs.rmSync(temp, { recursive: true, force: true });
}

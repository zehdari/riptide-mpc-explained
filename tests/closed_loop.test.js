import { test } from 'node:test';
import assert from 'node:assert';
import { Simulation, Mode } from '../src/engine/sim.js';
import { yawQuat } from '../src/engine/math.js';

function run(settings, seconds, refFn) {
  const sim = new Simulation({ settings });
  const ref0 = sim.holdHere();
  const log = [];
  let ms = 0, n = 0, maxIt = 0;
  for (let k = 0; k * 0.05 < seconds; k++) {
    const out = sim.tick(refFn(sim.t, ref0));
    ms += out.solve_ms; n++; maxIt = Math.max(maxIt, out.qp_iterations);
    log.push({ t: sim.t, p: sim.basePose().position, cmd: out.command });
  }
  return { log, avgMs: ms / n, maxIt, sim };
}

test('surge step settles on target', () => {
  const step = (t, r) => (t < 1 ? r : { ...r, position: [r.position[0] + 2, r.position[1], r.position[2]] });
  for (const model_step of [0.002, 0.01]) {
    const { log, avgMs, maxIt } = run({ model_step }, 25, step);
    const end = log[log.length - 1].p;
    const peak = Math.max(...log.map((l) => l.p[0]));
    console.log(`model_step ${model_step}: avg solve ${avgMs.toFixed(1)} ms, max qp it ${maxIt}, end x ${end[0].toFixed(4)} z ${end[2].toFixed(4)}, peak x ${peak.toFixed(4)}`);
    for (let i = 0; i < log.length; i += 40) console.log(log[i].t.toFixed(1), log[i].p.map((v) => v.toFixed(3)).join(' '), log[i].cmd.map((v) => v.toFixed(1)).join(' '));
    assert.ok(Math.abs(end[0] - 2) < 0.03, 'arrives');
    assert.ok(Math.abs(end[2] + 1) < 0.03, 'holds depth');
  }
});

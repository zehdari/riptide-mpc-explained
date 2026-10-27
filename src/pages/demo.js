// Loads precomputed closed-loop runs (tools/precompute.mjs) and Nereus traces.
const cache = new Map();
export function loadDemo(name) {
  if (!cache.has(name)) cache.set(name, fetch(`${import.meta.env.BASE_URL}data/demo_${name}.json`).then((r) => r.json()).then(index));
  return cache.get(name);
}
export function loadTrace(name) {
  if (!cache.has('n:' + name)) cache.set('n:' + name, fetch(`${import.meta.env.BASE_URL}data/${name}.json`).then((r) => r.json()));
  return cache.get('n:' + name);
}
function index(d) {
  d.objs = d.rows.map((r) => Object.fromEntries(d.cols.map((c, i) => [c, r[i]])));
  return d;
}
export const series = (d, f, every = 1) => d.objs.filter((_, i) => i % every === 0).map((r) => ({ x: r.t, y: f(r) }));

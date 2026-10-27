// Opens the hero's attitude panel, sets roll 30 / pitch -15 / yaw 60, and reports the vehicle's attitude.
import { chromium } from 'playwright';
const browser = await chromium.launch({ executablePath: process.env.HOME + '/.cache/ms-playwright/chromium-1248/chrome-linux64/chrome', args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
const p = await browser.newPage({ viewport: { width: +(process.env.W || 1400), height: +(process.env.H || 900) } });
const errs = [];
p.on('pageerror', (e) => errs.push(e.message));
await p.goto(process.argv[2] || 'http://127.0.0.1:5199/riptide-mpc-explained/', { waitUntil: 'networkidle' });
await p.waitForTimeout(2500);
await p.click('[data-act="attitude"]');
for (const [axis, v] of [[0, 30], [1, -15], [2, 60]]) {
  await p.locator(`[data-axis="${axis}"]`).evaluate((el, v) => { el.value = v; el.dispatchEvent(new Event('input', { bubbles: true })); }, v);
  await p.waitForTimeout(200);
}
await p.waitForTimeout(7000);
const q = await p.evaluate(() => window.__last.orientation);
const [w, x, y, z] = q, d = 180 / Math.PI;
console.log('rpy', [Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) * d, Math.asin(2 * (w * y - z * x)) * d, Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * d].map((v) => v.toFixed(1)).join(' '));
await p.screenshot({ path: '/tmp/attitude.png' });
console.log(errs.join('\n') || 'ok');
await browser.close();

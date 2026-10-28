// Draws an S-curve in the hero with draw mode, then reports the path progress.
import { chromium } from 'playwright';
const browser = await chromium.launch({ executablePath: process.env.HOME + '/.cache/ms-playwright/chromium-1248/chrome-linux64/chrome', args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
const p = await browser.newPage({ viewport: { width: 1400, height: 900 } });
const errs = [];
p.on('pageerror', (e) => errs.push(e.message));
await p.goto(process.argv[2] || 'http://127.0.0.1:5199/riptide-mpc-explained/', { waitUntil: 'networkidle' });
await p.waitForTimeout(2500);
await p.click('[data-act="draw"]');
const box = await p.locator('#hero-canvas').boundingBox();
const cx = box.x + box.width * 0.62, cy = box.y + box.height * 0.6;
await p.mouse.move(cx, cy);
await p.mouse.down();
for (let i = 0; i <= 40; i++) {
  const t = i / 40;
  await p.mouse.move(cx + 380 * t, cy - 120 * t + 90 * Math.sin(t * Math.PI * 2), { steps: 2 });
}
await p.waitForTimeout(300);
await p.screenshot({ path: '/tmp/draw_0.png' });
await p.mouse.up();
await p.waitForTimeout(9000);
await p.screenshot({ path: '/tmp/draw_1.png' });
console.log(await p.evaluate(() => JSON.stringify(window.__last.progress)));
console.log(errs.join('\n') || 'ok');
await browser.close();

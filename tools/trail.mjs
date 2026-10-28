// Flies the hero path, then screenshots it from a few orbit angles.
import { chromium } from 'playwright';
const browser = await chromium.launch({ executablePath: process.env.HOME + '/.cache/ms-playwright/chromium-1248/chrome-linux64/chrome', args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
const p = await browser.newPage({ viewport: { width: 1400, height: 900 } });
const errs = [];
p.on('pageerror', (e) => errs.push(e.message));
await p.goto(process.argv[2] || 'http://127.0.0.1:5199/riptide-mpc-explained/', { waitUntil: 'networkidle' });
await p.waitForTimeout(2500);
await p.click('[data-act="path"]');
await p.waitForTimeout(14000);
const canvas = p.locator('#hero-canvas');
const box = await canvas.boundingBox();
for (const [i, dx, dy] of [[0, 0, 0], [1, 260, 0], [2, -200, 120], [3, 0, -160]]) {
  if (dx || dy) {
    await p.mouse.move(box.x + box.width * 0.7, box.y + box.height * 0.5);
    await p.mouse.down();
    await p.mouse.move(box.x + box.width * 0.7 + dx, box.y + box.height * 0.5 + dy, { steps: 12 });
    await p.mouse.up();
    await p.waitForTimeout(900);
  }
  await canvas.screenshot({ path: `/tmp/trail_${i}.png` });
}
console.log(errs.join('\n') || 'ok');
await browser.close();

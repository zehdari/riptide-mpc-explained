// Flies the preset path, taps the floor mid-path, screenshots before/after.
import { chromium } from 'playwright';
const browser = await chromium.launch({ executablePath: process.env.HOME + '/.cache/ms-playwright/chromium-1248/chrome-linux64/chrome', args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
const p = await browser.newPage({ viewport: { width: 1400, height: 900 } });
const errs = [];
p.on('pageerror', (e) => errs.push(e.message));
await p.goto('http://127.0.0.1:5199/riptide-mpc-explained/', { waitUntil: 'networkidle' });
await p.waitForTimeout(3000);
await p.click('[data-act="path"]');
await p.waitForTimeout(6000);
await p.locator('#hero-canvas').screenshot({ path: '/tmp/cp_0.png' });
const box = await p.locator('#hero-canvas').boundingBox();
await p.mouse.click(box.x + box.width * 0.5, box.y + box.height * 0.8);
await p.waitForTimeout(3000);
await p.locator('#hero-canvas').screenshot({ path: '/tmp/cp_1.png' });
console.log(await p.evaluate(() => JSON.stringify({ progress: window.__last.progress, goal: window.__last.goal })));
console.log(errs.join('\n') || 'ok');
await browser.close();

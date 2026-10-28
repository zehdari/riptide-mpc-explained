// Default hero view, zoomed in on Talos. Arg 2: query string (e.g. "orig").
import { chromium } from 'playwright';
const out = process.argv[2];
const browser = await chromium.launch({ executablePath: process.env.HOME + '/.cache/ms-playwright/chromium-1248/chrome-linux64/chrome', args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
const p = await browser.newPage({ viewport: { width: 1400, height: 900 }, deviceScaleFactor: 2 });
p.setDefaultTimeout(180000);
await p.goto('http://127.0.0.1:5199/riptide-mpc-explained/' + (process.argv[3] ? '?' + process.argv[3] : ''), { waitUntil: 'load' });
await p.waitForTimeout(9000);
const box = await p.locator('#hero-canvas').boundingBox();
await p.mouse.move(box.x + box.width * 0.7, box.y + box.height * 0.45);
for (let i = 0; i < +(process.env.ZOOM || 4); i++) { await p.mouse.wheel(0, -200); await p.waitForTimeout(150); }
await p.waitForTimeout(2500);
await p.locator('#hero-canvas').screenshot({ path: out });
await browser.close();

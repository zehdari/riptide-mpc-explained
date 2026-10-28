// Orbits the hero camera behind Talos, zooms in, screenshots. Arg 2: "double" to force DoubleSide materials (debug).
import { chromium } from 'playwright';
const out = process.argv[2] || '/tmp/back.png';
const browser = await chromium.launch({ executablePath: process.env.HOME + '/.cache/ms-playwright/chromium-1248/chrome-linux64/chrome', args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
const p = await browser.newPage({ viewport: { width: 1400, height: 900 }, deviceScaleFactor: 2 }); p.setDefaultTimeout(180000);
await p.goto('http://127.0.0.1:5199/riptide-mpc-explained/' + (process.argv[3] ? '?' + process.argv[3] : ''), { waitUntil: 'load', timeout: 120000 }); await p.waitForSelector('#hero-canvas'); await p.waitForTimeout(9000);
await p.waitForTimeout(3000);
const box = await p.locator('#hero-canvas').boundingBox();
const cx = box.x + box.width * 0.62, cy = box.y + box.height * 0.45;
await p.mouse.move(cx, cy);
await p.mouse.down();
await p.mouse.move(cx + 520, cy - 40, { steps: 20 });
await p.mouse.up();
for (let i = 0; i < 6; i++) { await p.mouse.wheel(0, -200); await p.waitForTimeout(120); }
await p.waitForTimeout(1500);
await p.locator('#hero-canvas').screenshot({ path: out });
await browser.close();

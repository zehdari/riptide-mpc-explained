// Close-up of the Talos model in the thrusters chapter's allocation view (auto-rotating, no sim).
import { chromium } from 'playwright';
const out = process.argv[2] || '/tmp/model.png';
const browser = await chromium.launch({ executablePath: process.env.HOME + '/.cache/ms-playwright/chromium-1248/chrome-linux64/chrome', args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
const p = await browser.newPage({ viewport: { width: 1400, height: 900 }, deviceScaleFactor: 2 });
await p.goto('http://127.0.0.1:5199/riptide-mpc-explained/', { waitUntil: 'networkidle' });
await p.waitForTimeout(4000);
await p.locator('#hero-canvas').screenshot({ path: out });
await browser.close();

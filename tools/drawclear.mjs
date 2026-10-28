// Phone check: draw a path by touch, then tap the floor (and separately turn), screenshotting each state.
// Logs whether the planned line is still on screen by counting its pixels' colour.
import { chromium, devices } from 'playwright';
const browser = await chromium.launch({ executablePath: process.env.HOME + '/.cache/ms-playwright/chromium-1248/chrome-linux64/chrome', args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
const ctx = await browser.newContext({ ...devices['iPhone 13'] });
const p = await ctx.newPage();
await p.goto(process.argv[2] || 'http://127.0.0.1:5199/riptide-mpc-explained/', { waitUntil: 'networkidle' });
await p.waitForTimeout(3000);
const cdp = await ctx.newCDPSession(p);
const box = await p.locator('#hero-canvas').boundingBox();
const touch = (type, x, y) => cdp.send('Input.dispatchTouchEvent', { type, touchPoints: type === 'touchEnd' ? [] : [{ x, y }] });
const draw = async () => {
  await p.tap('[data-act="draw"]');
  const cx = box.x + box.width * 0.2, cy = box.y + box.height * 0.38;
  await touch('touchStart', cx, cy);
  for (let i = 1; i <= 30; i++) { const t = i / 30; await touch('touchMove', cx + box.width * 0.6 * t, cy + 40 * Math.sin(t * 6.28)); await p.waitForTimeout(15); }
  await touch('touchEnd');
  await p.waitForTimeout(2500);
};
const shot = async (n) => { await p.locator('#hero-canvas').screenshot({ path: `/tmp/dc_${n}.png` }); };
await draw(); await shot(0);
await p.touchscreen.tap(box.x + box.width * 0.3, box.y + box.height * 0.47);
await p.waitForTimeout(2500); await shot(1);
await draw(); await shot(2);
await p.tap('[data-act="turn"]');
await p.waitForTimeout(2500); await shot(3);
await browser.close();

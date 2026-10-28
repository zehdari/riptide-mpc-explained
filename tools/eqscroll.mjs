// Lists displayed equations and tables that scroll sideways on a phone-width page.
import { chromium } from 'playwright';
const url = process.argv[2] || 'http://127.0.0.1:5199/riptide-mpc-explained/math.html';
const browser = await chromium.launch({ executablePath: process.env.HOME + '/.cache/ms-playwright/chromium-1248/chrome-linux64/chrome', args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
const p = await browser.newPage({ viewport: { width: 390, height: 844 } });
await p.goto(url, { waitUntil: 'networkidle' });
await p.waitForTimeout(1200);
console.log((await p.evaluate(() => [...document.querySelectorAll('.katex-display, .table-scroll')]
  .filter((e) => e.scrollWidth > e.clientWidth + 1)
  .map((e) => `${e.className} over by ${e.scrollWidth - e.clientWidth}px: ${(e.closest('.M') || e).textContent.slice(0, 50).replace(/\s+/g, ' ')}`))).join('\n') || 'nothing scrolls');
await p.screenshot({ path: '/tmp/math_phone.png', fullPage: true });
await browser.close();

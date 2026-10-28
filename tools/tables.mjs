// Lists tables that overflow their scroll wrapper at phone width.
import { chromium } from 'playwright';
const base = process.argv[2] || 'http://127.0.0.1:5199/riptide-mpc-explained/';
const pages = ['index.html', 'model.html', 'thrusters.html', 'matters.html', 'reference.html', 'mpc.html', 'solver.html', 'estimation.html', 'identify.html', 'build.html'];
const browser = await chromium.launch({ executablePath: process.env.HOME + '/.cache/ms-playwright/chromium-1248/chrome-linux64/chrome', args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
const p = await browser.newPage({ viewport: { width: 390, height: 844 } });
for (const pg of pages) {
  await p.goto(base + pg, { waitUntil: 'networkidle' });
  await p.waitForTimeout(1200);
  const r = await p.evaluate(() => [...document.querySelectorAll('.table-scroll')].map((w) => ({ head: w.querySelector('th, td')?.textContent.trim().slice(0, 30), over: w.scrollWidth - w.clientWidth })).filter((x) => x.over > 1));
  for (const x of r) console.log(pg, `"${x.head}"`, '+' + x.over + 'px');
}
await browser.close();

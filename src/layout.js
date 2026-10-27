import './style.css';
import katex from 'katex';

export const chapters = [
  { href: 'index.html', title: 'Overview' },
  { href: 'model.html', title: 'The Fossen model' },
  { href: 'thrusters.html', title: 'Thrusters and allocation' },
  { href: 'reference.html', title: 'Where to go: profiles and paths' },
  { href: 'mpc.html', title: 'The optimal control problem' },
  { href: 'solver.html', title: 'Solving it every 50 ms' },
  { href: 'estimation.html', title: 'Feedback and model error' },
  { href: 'matters.html', title: 'Which numbers matter' },
  { href: 'build.html', title: 'Build your own' },
];

export const SRC = 'https://github.com/osu-uwrt/riptide_control/blob/mpc/riptide_mpc';

function renderMath(root = document) {
  root.querySelectorAll('.m, .M').forEach((el) => {
    if (el.dataset.done) return;
    const tex = el.textContent;
    katex.render(tex, el, { displayMode: el.classList.contains('M'), throwOnError: false, trust: true, macros: MACROS });
    el.dataset.done = '1';
  });
}

export const MACROS = {
  '\\R': '\\mathbb{R}',
  '\\bnu': '\\boldsymbol{\\nu}',
  '\\bEta': '\\boldsymbol{\\eta}',
  '\\btau': '\\boldsymbol{\\tau}',
  '\\M': '\\mathbf{M}',
  '\\T': '\\mathbf{T}',
};

function buildRail() {
  const here = location.pathname.split('/').pop() || 'index.html';
  const rail = document.createElement('nav');
  rail.className = 'rail';
  rail.setAttribute('aria-label', 'Chapters');
  rail.innerHTML = `
    <a class="brand" href="index.html">How Talos flies<small>The riptide_mpc controller, explained</small></a>
    <ol>${chapters
      .map((c) => `<li><a href="${c.href}"${c.href === here ? ' aria-current="page"' : ''}>${c.title}</a>${c.href === here ? '<ul class="toc"></ul>' : ''}</li>`)
      .join('')}</ol>
    <div class="foot">Code: <a href="https://github.com/osu-uwrt/riptide_control/tree/mpc/riptide_mpc">riptide_control/riptide_mpc</a><br>Simulator: <a href="https://github.com/osu-uwrt/nereus">Nereus</a><br>Underwater Robotics Team, Ohio State</div>`;
  const toc = rail.querySelector('.toc');
  if (toc) {
    document.querySelectorAll('main h2[id]').forEach((h) => {
      const li = document.createElement('li');
      li.innerHTML = `<a href="#${h.id}">${h.dataset.short || h.textContent}</a>`;
      toc.appendChild(li);
    });
    const links = [...toc.querySelectorAll('a')];
    const heads = links.map((a) => document.getElementById(a.getAttribute('href').slice(1)));
    const spy = () => {
      let on = 0;
      heads.forEach((h, i) => { if (h.getBoundingClientRect().top < 140) on = i; });
      links.forEach((a, i) => a.classList.toggle('on', i === on && window.scrollY > 200));
    };
    addEventListener('scroll', spy, { passive: true });
    spy();
  }
  const btn = document.createElement('button');
  btn.className = 'btn menu-btn';
  btn.textContent = 'Chapters';
  btn.setAttribute('aria-expanded', 'false');
  btn.onclick = (e) => {
    e.stopPropagation();
    const open = rail.classList.toggle('open');
    btn.setAttribute('aria-expanded', String(open));
  };
  document.addEventListener('click', (e) => { if (!rail.contains(e.target)) rail.classList.remove('open'); });
  return { rail, btn };
}

function nextPrev() {
  const here = location.pathname.split('/').pop() || 'index.html';
  const i = chapters.findIndex((c) => c.href === here);
  const slot = document.querySelector('.next');
  if (!slot || i < 0) return;
  const prev = chapters[i - 1], next = chapters[i + 1];
  slot.innerHTML = `${prev ? `<a href="${prev.href}"><small>Previous</small><strong>${prev.title}</strong></a>` : '<span></span>'}${next ? `<a href="${next.href}" style="text-align:right"><small>Next</small><strong>${next.title}</strong></a>` : ''}`;
}

// Wraps tables so a wide one scrolls inside the text column on phones.
export function wrapTables(root = document) {
  root.querySelectorAll('table.params').forEach((t) => {
    if (t.parentElement.classList.contains('table-scroll')) return;
    const w = document.createElement('div');
    w.className = 'table-scroll';
    t.replaceWith(w);
    w.appendChild(t);
    // Let long names break after _ and / rather than mid-word.
    t.querySelectorAll('code').forEach((c) => {
      if (!c.children.length) c.innerHTML = c.textContent.replace(/[&<>]/g, (m) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' })[m]).replace(/([_/])/g, '$1<wbr>');
    });
  });
}

export function setup() {
  const main = document.querySelector('main');
  wrapTables();
  new MutationObserver(() => wrapTables()).observe(main, { childList: true, subtree: true });
  const shell = document.createElement('div');
  shell.className = 'shell';
  main.parentNode.insertBefore(shell, main);
  const { rail, btn } = buildRail();
  shell.appendChild(rail);
  shell.appendChild(main);
  document.body.appendChild(btn);
  renderMath();
  nextPrev();
}

export { renderMath };

// Navegación común a las tres páginas, como teclas programables del panel FANUC.
// Cada página pone <nav id="nav" data-actual="..."></nav> al pie del marco y esto la
// rellena. Son enlaces, no botones: no se usan las teclas 1-9 del kit, para que una
// pulsación suelta en el monitor no cambie de página.
(function () {
  const PAGINAS = [
    ['monitor', '/',        'MONITOR'],
    ['ensayos', '/ensayos', 'ENSAYOS'],
    ['sim',     '/sim',     'MECANISMO'],
  ];
  function montar() {
    const nav = document.getElementById('nav');
    if (!nav) return;
    nav.classList.add('fui-softkeys', 'tema-teclas');
    nav.style.setProperty('--fui-keys', PAGINAS.length);
    nav.setAttribute('aria-label', 'Páginas');
    const actual = nav.dataset.actual;
    nav.innerHTML = PAGINAS.map(([id, href, txt], i) =>
      `<a class="fui-softkey" href="${href}"${id === actual ? ' aria-current="page"' : ''}>`
      + `<span class="fui-legend">${i + 1}</span>${txt}</a>`).join('');
  }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', montar);
  else montar();
})();

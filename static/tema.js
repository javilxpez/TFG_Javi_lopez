// ── Tema FANUC para lo que no pasa por CSS ──────────────────────────────
// Chart.js y el canvas del mecanismo pintan con colores en JS. Se leen de los tokens de
// tema.css en vez de repetirlos aquí, para que haya un único sitio donde cambiarlos.

function coloresTema() {
  const cs = getComputedStyle(document.documentElement);
  const v = n => cs.getPropertyValue(n).trim();
  return {
    texto: v('--graf-texto'), tinta: v('--graf-tinta'), rejilla: v('--graf-rejilla'),
    azul: v('--serie-azul'), cian: v('--serie-cian'), morado: v('--serie-morado'),
    verde: v('--serie-verde'), ambar: v('--serie-ambar'), rojo: v('--serie-rojo'),
    tinta2: v('--serie-tinta2'), regla: v('--serie-regla'), reglaSuave: v('--serie-regla-suave'),
    fondo: v('--fui-field'), fuente: v('--fui-font-data')
  };
}

// '#rrggbb' → 'rgba(r,g,b,a)', para rellenos translúcidos con los colores del tema.
function conAlfa(hex, a) {
  const n = parseInt(hex.replace('#', ''), 16);
  return `rgba(${(n >> 16) & 255},${(n >> 8) & 255},${n & 255},${a})`;
}

// Tipografía y colores por defecto de Chart.js: lo que una gráfica no fije, sale del tema.
function temaGraficas() {
  if (!window.Chart) return;
  const t = coloresTema();
  Chart.defaults.font.family = t.fuente;
  Chart.defaults.font.size = 11;
  Chart.defaults.color = t.tinta;
  Chart.defaults.borderColor = t.rejilla;
}

// Reloj de la línea de estado: cualquier elemento con data-reloj.
document.addEventListener('DOMContentLoaded', () => {
  const relojes = document.querySelectorAll('[data-reloj]');
  if (!relojes.length) return;
  const dos = n => String(n).padStart(2, '0');
  const tic = () => {
    const d = new Date(), txt = `${dos(d.getHours())}:${dos(d.getMinutes())}:${dos(d.getSeconds())}`;
    relojes.forEach(r => { r.textContent = txt; });
  };
  tic();
  setInterval(tic, 1000);
});

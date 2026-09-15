// ── Modelo del mecanismo ─────────────────────────────────────────────────
// Una sola implementación de la geometría para /sim y /ensayos. Antes cada página
// llevaba su copia de las fórmulas y sus propios valores por defecto, y ya se habían
// separado: el tab teórico del monitor seguía con un cilindro y la barra horizontal
// mientras los ensayos usaban la caracola. La configuración vive en el servidor
// (mecanismo.json) y se edita sólo desde /sim.
//
// Convención, la misma que dibuja /sim:
//   O   articulación de la barra, en el origen.
//   D   salida del cable, en lo alto del mástil VERTICAL de altura L2: D = (0, L2).
//   θ   ángulo de la barra medido desde la HORIZONTAL; +90 apunta hacia arriba por el
//       mástil, −90 cuelga hacia abajo. α = 90 − θ es el ángulo en O entre mástil y barra.
//   A   punto donde tira el cable, a L3 de O:  A = L3·(cos θ, sen θ).
//   c   cable entre D y A:  c² = L2² + L3² − 2·L2·L3·sen θ.
//
// Equilibrio de momentos en O (el cable tira de A hacia D, la pesa cuelga en L3+L4):
//   T·d = m·g·(L3+L4)·cos θ,   d = L2·L3·cos θ / c   →   T = m·g·(L3+L4)·c / (L2·L3)
// y el par en el tambor es τ = T·r(φ) con el radio instantáneo de la caracola.

const MEC_DEFAULTS = {
  tipo: 'car',        // 'car' caracola de radio variable, 'cil' cilindro de radio fijo
  L1: 25,             // radio del cilindro (mm), sólo con tipo 'cil'
  r0: 25, r1: 45,     // radio de la caracola al principio y al final del barrido (mm)
  barr: 0.5,          // vueltas de tambor que dura la espiral
  L2: 290,            // mástil vertical, de O a la salida del cable (mm)
  L3: 95,             // de O al punto donde tira el cable (mm)
  L4: 110,            // resto de barra hasta la pesa (mm)
  m: 2, g: 9.81,
  th0: 90,            // ángulo de la barra en el home (°), en [−90, 90]
  red: 4.0,           // vueltas de motor por vuelta de tambor
  sentido: -1,        // −1: giro positivo desenrolla (cable más largo), +1: enrolla
  cicloRev: 1.78      // vueltas de motor entre A y B, para simular sin un ensayo cargado
};

const Mec = {
  q: { ...MEC_DEFAULTS },

  // ── Persistencia y propagación ──
  // El servidor es la fuente; BroadcastChannel avisa al instante a las otras pestañas
  // del mismo navegador, y al volver a una pestaña se relee por si se cambió desde
  // otro equipo.
  canal: ('BroadcastChannel' in self) ? new BroadcastChannel('mecanismo') : null,

  async cargar() {
    try {
      const r = await fetch('/api/mecanismo', { cache: 'no-store' });
      if (r.ok) this.q = { ...MEC_DEFAULTS, ...(await r.json()) };
    } catch (e) { /* sin servidor: se queda con lo que tenía */ }
    return this.q;
  },

  async guardar(q) {
    const r = await fetch('/api/mecanismo', {
      method: 'PUT', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(q)
    });
    const j = await r.json();
    if (!r.ok) throw new Error(j.error || ('HTTP ' + r.status));
    this.q = { ...MEC_DEFAULTS, ...j };
    if (this.canal) this.canal.postMessage(this.q);
    return this.q;
  },

  alCambiar(cb) {
    if (this.canal) this.canal.addEventListener('message', e => {
      this.q = { ...MEC_DEFAULTS, ...e.data }; cb(this.q);
    });
    window.addEventListener('focus', async () => {
      const antes = JSON.stringify(this.q);
      await this.cargar();
      if (JSON.stringify(this.q) !== antes) cb(this.q);
    });
  },

  // ── Geometría ──
  // Tambor: radio instantáneo y cable pagado para un giro φ (rad) contado desde el home.
  // El giro llega con signo: el radio se evalúa en |φ| y el cable hereda el signo.
  // Pasado el barrido el radio se queda en r1, que es lo que hace la pieza real.
  tambor(q, phi) {
    const sg = phi < 0 ? -1 : 1;
    phi = Math.abs(phi);
    if (q.tipo === 'cil') return { r: q.L1, s: sg * q.L1 * phi };
    const phitot = q.barr * 2 * Math.PI;
    if (phi <= phitot) {
      const r = q.r0 + (q.r1 - q.r0) * phi / phitot;
      return { r, s: sg * (q.r0 * phi + (q.r1 - q.r0) * phi * phi / (2 * phitot)) };
    }
    const sTot = phitot * (q.r0 + q.r1) / 2;
    return { r: q.r1, s: sg * (sTot + q.r1 * (phi - phitot)) };
  },

  cable(q, thDeg) {
    const s = Math.sin(thDeg * Math.PI / 180);
    return Math.sqrt(q.L2 * q.L2 + q.L3 * q.L3 - 2 * q.L2 * q.L3 * s);
  },
  cHome(q)   { return this.cable(q, q.th0); },
  limites(q) { return { cmin: Math.abs(q.L2 - q.L3), cmax: q.L2 + q.L3 }; },

  // sen θ es monótona en c y θ ∈ [−90, 90] es el recorrido físico: el ángulo es único.
  barraAng(q, c) {
    const sn = (q.L2 * q.L2 + q.L3 * q.L3 - c * c) / (2 * q.L2 * q.L3);
    if (sn < -1 || sn > 1) return null;
    return Math.asin(sn) * 180 / Math.PI;
  },

  // Estado completo del mecanismo con el motor a relRev vueltas del home.
  estado(q, relRev) {
    const phi = relRev * 2 * Math.PI / q.red;
    const t = this.tambor(q, phi);
    const c = this.cHome(q) - q.sentido * t.s;
    const { cmin, cmax } = this.limites(q);
    const base = { relRev, phi, r: t.r, s: t.s, c };
    if (!(c >= cmin && c <= cmax)) return { ...base, fuera: true };
    const th = this.barraAng(q, c);
    const T = q.m * q.g * (q.L3 + q.L4) * c / (q.L2 * q.L3);
    return { ...base, fuera: false, th, T, tau: T * t.r / 1000 };
  },

  // Inversa: vueltas de motor desde el home para llevar la barra a thDeg. El cable
  // pagado es monótono en φ, así que basta una bisección. null si no se llega.
  revParaAngulo(q, thDeg) {
    if (thDeg < -90 || thDeg > 90) return null;
    const sObj = (this.cHome(q) - this.cable(q, thDeg)) / q.sentido;
    const lim = 20 * 2 * Math.PI;                       // ±20 vueltas de tambor
    let a = -lim, b = lim;
    if ((this.tambor(q, a).s - sObj) * (this.tambor(q, b).s - sObj) > 0) return null;
    for (let i = 0; i < 80; i++) {
      const mitad = (a + b) / 2;
      if ((this.tambor(q, a).s - sObj) * (this.tambor(q, mitad).s - sObj) <= 0) b = mitad;
      else a = mitad;
    }
    return (a + b) / 2 * q.red / (2 * Math.PI);
  },

  // Lo que el servidor también rechaza, para avisar antes de enviar.
  validar(q) {
    const num = ['L1', 'r0', 'r1', 'barr', 'L2', 'L3', 'L4', 'm', 'g', 'th0', 'red', 'cicloRev'];
    for (const k of num) if (!Number.isFinite(q[k])) return `${k} no es un número`;
    if (q.th0 < -90 || q.th0 > 90)
      return 'el ángulo del home va de −90 a 90°: sen θ es simétrico, 100° daría el mismo cable que 80°';
    for (const k of ['L1', 'r0', 'r1', 'barr', 'L2', 'L3', 'red'])
      if (!(q[k] > 0)) return `${k} tiene que ser mayor que 0`;
    if (q.L4 < 0 || q.m < 0 || q.g < 0) return 'L4, masa y g no pueden ser negativos';
    return null;
  }
};

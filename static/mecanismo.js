// ── Modelo del mecanismo ─────────────────────────────────────────────────
// Una sola implementación de la geometría para /sim y /ensayos. Antes cada página
// llevaba su copia de las fórmulas y sus propios valores por defecto, y ya se habían
// separado: el tab teórico del monitor seguía con un cilindro y la barra horizontal
// mientras los ensayos usaban la caracola. La configuración vive en el servidor
// (mecanismo.json) y se edita sólo desde /sim.
//
// Convención, la misma que dibuja /sim:
//   O   articulación de la barra, en el origen. No está en el eje del carril vertical:
//       la escuadra la saca ob hacia la derecha (ob sólo pinta, no entra en las fórmulas).
//   C   eje de giro del tambor: sube L2 y queda xt a la izquierda, C = (−xt, L2).
//   T   salida del cable. El cable NO sale del eje: sale TANGENTE al tambor, a r(φ) del
//       eje, por el lado de O. Al crecer el radio la línea del cable se desplaza r1 − r0
//       (28 mm en la caracola medida), que es lo que antes se escondía en un "dx" fijo.
//   θ   barra medida desde la horizontal. Es lo que se configura (th0 en B), porque es
//       lo que se puede medir con un nivel sobre la barra. Va del punto muerto (ver
//       thMuerto) a +90°.
//   α   = incC − θ, el ángulo en O entre la línea O→C y la barra. Interno: es el que
//       entra en la ley del coseno. Ojo, llega hasta 180° + δ, NO hasta 180°; acotarlo
//       a 180° era lo que impedía representar la barra colgando.
//   A   punto del EJE de la barra a L3 de O. El cable no tira de ahí: tira de A' , que
//       está e1 por encima en perpendicular a la barra. Y la pesa no cuelga del eje,
//       sino de P' , e2 por debajo, a L3+L4 de O.
//         A' = L3·û + e1·n̂      P' = (L3+L4)·û − e2·n̂
//       con û = (cos θ, sen θ) la barra y n̂ = (−sen θ, cos θ) su perpendicular.
//       Eso mete dos cambios: el tiro se aleja de O —L3' = √(L3² + e1²), adelantado
//       δ = atan(e1/L3) respecto del eje— y el brazo del peso deja de ser (L3+L4)·cos θ.
//   d   distancia del eje del tambor a A' (ley del coseno):
//         β = α − δ,   d² = |OC|² + L3'² − 2·|OC|·L3'·cos β,   |OC| = √(L2² + xt²)
//   c   cable libre, de T a A'. Es el cateto del triángulo rectángulo C–T–A' (el radio
//       es perpendicular a la tangente):   c² = d² − r²
//
// El desplazamiento xt cambia la física, no sólo el dibujo: con C encima de O la línea
// O→C era vertical, θ = 90 − α, y el cos θ del peso se cancelaba con el sen α del cable,
// dejando T proporcional al cable que queda. Inclinada esa línea ya no se cancelan, y
// cuando la línea del cable pasa por O el brazo es cero con la pesa aún haciendo momento:
// ahí la tensión se va a infinito y el mecanismo no puede sostener la carga.
//
// Equilibrio de momentos en O (el cable tira de A' hacia T, la pesa cuelga en L3+L4):
//   bc = distancia de O a la línea T–A'   brazo del cable (producto vectorial A' × û)
//   b  = (L3+L4)·cos θ + e2·sen θ          brazo del peso
//   T·bc = m·g·b
// Con r = 0 (cable saliendo del eje) bc = |OC|·L3'·sen β / c: la fórmula anterior. Y con
// e1 = e2 = 0 queda L3' = L3, δ = 0, β = α y b = (L3+L4)·cos θ.
// El par en el tambor es τ = T·r(φ) con el radio instantáneo de la caracola.
//
// Perfil de la caracola, medido sobre la sección del CAD (foto del 30-09): una espiral
// LINEAL de 56 a ≈28 mm en unos 131° (0.364 rev), con un arco a radio constante en cada
// extremo (≈50° a 56 y ≈115° a 28) y una rampa corta de cierre (≈60°). Es exactamente
// la ley de tambor(): rampa lineal entre r0 y r1 en 'barr' vueltas y radio fijo fuera.

const MEC_DEFAULTS = {
  tipo: 'car',        // 'car' caracola de radio variable, 'cil' cilindro, 'leva' par constante
  L1: 25,             // radio del cilindro (mm), sólo con tipo 'cil'
  // Medidos sobre el CAD de la caracola: el radio CRECE al subir. El ajuste anterior los
  // daba al revés (45 → 25) porque el modelo no podía representar la barra colgando y
  // compensaba con la caracola. Ver la nota de validación al final del fichero.
  r0: 28.18, r1: 56,  // radio de la caracola en B y al final del barrido (mm)
  barr: 0.367,        // vueltas de tambor del barrido = 1.835 rev de motor medidas / 5
  L2: 290,            // altura de D sobre O (mm)
  // Eje del tambor: está EN el eje del carril (comprobado en el banco), y el carril queda
  // 30 mm a la izquierda de la articulación, así que xt = ob. El 48 que hubo aquí salía
  // de leer el CAD de perfil con el eje en el centro de la placa del motor, 18 mm más
  // allá del carril: no era así. El "dx = 11" anterior era la línea del cable medida en
  // una foto a medio recorrido: con el cable saliendo tangente por la derecha del tambor
  // la línea queda en −xt + r, y se desplaza r1 − r0 a lo largo del recorrido. Un punto
  // fijo no puede representar ese desplazamiento.
  xt: 30,             // eje del tambor a la izquierda de la vertical de O (mm)
  ob: 30,             // carril vertical a la izquierda de O (mm). Sólo dibujo.
  L3: 95,             // de O al punto del eje donde tira el cable (mm)
  L4: 110,            // resto de barra hasta la pesa (mm)
  e1: 20,             // el cable tira e1 por encima del eje de la barra (mm)
  e2: 20,             // la pesa cuelga e2 por debajo del eje (mm)
  m: 2, g: 9.81,
  th0: -87,           // barra desde la horizontal al empezar el ensayo, en B (°). Medido
                      // con inclinómetro: el recorrido va de −87° a +3°. Ojo, queda 2.6°
                      // por debajo de thLibre(): ver la nota de ahí.
  // Leva de par constante (tipo 'leva'): par objetivo en el tambor, tope del radio donde
  // la tensión se anula y hasta qué θ se diseña. El barrido sale del perfil.
  tau0: 1.5,          // N·m en el tambor
  rmax: 70,           // mm
  thfin: 3,           // ° hasta dónde se diseña: el recorrido medido del banco acaba en +3°.
                      // Ojo: en esta geometría la tensión NO baja al pasar la horizontal
                      // (41 N a 0°, 45 a 40°, 60 a 60°, y se dispara hacia 70°, donde el
                      // cable se alinea con la barra), así que pasado 0° el radio sigue
                      // bajando, no vuelve a crecer. Con el cable vertical sería constante.
  red: 5,             // vueltas de motor por vuelta de tambor: la reducción 3:15
  sentido: 1          // +1: avanzar el ciclo acorta el cable (B → A), −1: lo alarga
};

const Mec = {
  q: { ...MEC_DEFAULTS },
  // Claves que el servidor no conoce. Si el puente es anterior a un parámetro nuevo, sus
  // valores por defecto pisan los de aquí y el modelo calcula con geometría vieja sin que
  // se note: eso dibujaba una teórica invertida y parecía un error de signo.
  faltan: [],

  // ── Persistencia y propagación ──
  // El servidor es la fuente; BroadcastChannel avisa al instante a las otras pestañas
  // del mismo navegador, y al volver a una pestaña se relee por si se cambió desde
  // otro equipo.
  canal: ('BroadcastChannel' in self) ? new BroadcastChannel('mecanismo') : null,

  async cargar() {
    try {
      const r = await fetch('/api/mecanismo', { cache: 'no-store' });
      if (r.ok) {
        const dado = await r.json();
        this.faltan = Object.keys(MEC_DEFAULTS).filter(k => !(k in dado));
        this.q = { ...MEC_DEFAULTS, ...dado };
      }
    } catch (e) { /* sin servidor: se queda con lo que tenía */ }
    return this.q;
  },

  // Aviso listo para pintar, o cadena vacía si el servidor está al día.
  aviso() {
    return this.faltan.length
      ? `El servidor no conoce ${this.faltan.join(', ')}: es anterior a esos parámetros y está`
        + ` imponiendo su geometría. Reinicia web_monitor.py.`
      : '';
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
  // Tambor: radio instantáneo y cable pagado para un giro φ (rad) del tambor, contado
  // desde el arranque del ensayo (B) y CON SIGNO (positivo = el ciclo avanza hacia A).
  //
  // La espiral tiene una sola ley: al avanzar el ciclo el punto de trabajo va del radio
  // r0 (el de B) al r1, a lo largo de 'barr' vueltas de tambor. Fuera de ese tramo el
  // cable trabaja sobre el extremo que corresponda y el radio se queda en él: en r0 si se
  // gira hacia atrás desde el home, en r1 pasado el final de la espiral.
  //
  // r0 y r1 pueden estar en cualquier orden: r0 > r1 es simplemente una caracola montada
  // al revés, con el radio decreciendo al avanzar. Antes el radio se evaluaba en |φ|,
  // que equivalía a suponer una espiral simétrica a cada lado del arranque: retroceder
  // volvía a crecer hacia r1 en vez de quedarse en r0.
  //
  //   r(φ) = r0 + (r1−r0)·u/φtot,   u = clamp(φ, 0, φtot)   (φ = avance del tambor)
  //   s(φ) = ∫₀^φ r(ψ) dψ           (con signo)
  tambor(q, phi) {
    if (q.tipo === 'cil') return { r: q.L1, s: q.L1 * phi };
    if (q.tipo === 'leva') return this.tamborLeva(q, phi);
    const phitot = q.barr * 2 * Math.PI;
    if (phi <= 0) return { r: q.r0, s: q.r0 * phi };
    if (phi <= phitot) {
      const r = q.r0 + (q.r1 - q.r0) * phi / phitot;
      return { r, s: q.r0 * phi + (q.r1 - q.r0) * phi * phi / (2 * phitot) };
    }
    const sTot = phitot * (q.r0 + q.r1) / 2;
    return { r: q.r1, s: sTot + q.r1 * (phi - phitot) };
  },

  // Eje del tambor C: distancia O→C y hacia dónde apunta (grados desde el eje +x, con
  // la barra tendida hacia +x). Con xt = 0 la inclinación es 90°, la vertical de siempre.
  oc(q)   { return Math.hypot(q.xt, q.L2); },
  incC(q) { return Math.atan2(q.L2, -q.xt) * 180 / Math.PI; },
  // Lado del tambor por el que baja el cable: el de O. Con el eje a la izquierda de O
  // (xt > 0) sale por la derecha del tambor; +1 derecha, −1 izquierda.
  lado(q) { return q.xt >= 0 ? 1 : -1; },
  // Radio con el que trabaja el cable en B, antes de girar nada.
  rHome(q) { return q.tipo === 'car' ? q.r0 : q.tipo === 'leva' ? q.rmax : q.L1; },
  // Barrido del tambor en el recorrido, en vueltas. En la caracola y el cilindro es el
  // parámetro; en la leva sale del perfil.
  barrido(q) { return q.tipo === 'leva' ? this.perfilLeva(q).barr : q.barr; },

  // ── Leva de par constante ──
  // Perfil r(φ) que mantiene el par en el tambor en τ0 a lo largo del recorrido: en cada
  // punto r = τ0/T, con T la tensión que pide la barra ahí. Donde la tensión es casi nula
  // (el arranque, con el cable flojo) el radio se acota a rmax. Como T depende de θ, θ del
  // cable recogido y el cable del propio radio, el perfil sale integrando paso a paso con
  // la misma cinemática que estado(). Se calcula una vez por configuración y se cachea.
  _leva: { clave: null, perfil: null },
  _levaPorQ: new WeakMap(),
  // Ángulo de presión máximo que se le permite al perfil, atan(|dr/dψ|/r). Un escalón más
  // brusco no lo sigue el cable: lo puentea como una cuerda y el modelo deja de valer.
  PRESION_MAX: 25,
  perfilLeva(q) {
    // Por identidad del objeto primero: tamborLeva() llama aquí miles de veces por
    // trayectoria y montar la clave JSON cada vez lo hacía 40 veces más lento.
    const porQ = this._levaPorQ.get(q);
    if (porQ) return porQ;
    const clave = JSON.stringify([q.tau0, q.rmax, q.thfin, q.th0, q.xt, q.L2, q.L3, q.L4, q.e1, q.e2, q.m, q.g, !!q._sinIter, this.PRESION_MAX]);
    if (this._leva.clave === clave) { this._levaPorQ.set(q, this._leva.perfil); return this._leva.perfil; }
    const dphi = 0.5 * Math.PI / 180, tanG = Math.tan(this.PRESION_MAX * Math.PI / 180);
    // Integra el perfil arrancando en rIni. En cada paso el radio que pide el par es
    // τ0/T, pero no se le deja bajar (ni subir) más deprisa que tanG·r por radián. Si la
    // bajada no alcanza a la curva τ0/T, el par se pasa de τ0: eso es lo que devuelve
    // 'exceso', y lo que decide el radio de arranque.
    // Además el perfil tiene que ser CONVEXO: un cable tenso sólo se asienta sobre un
    // perfil convexo, en un tramo cóncavo lo puentea como una cuerda y el radio que
    // trabaja es el de la cuerda, no el del perfil. En polares la curvatura es
    // proporcional a r² + 2r'² − r·r'', así que r'' ≤ (r² + 2r'²)/r. Eso acota lo rápido
    // que la bajada puede aplanarse al llegar al radio final: la curva τ0/T se aplana
    // más deprisa de lo que permite la convexidad, y ahí el perfil se queda un poco por
    // debajo (par algo menor que τ0, nunca mayor).
    const integra = (rIni, corr) => {
      const phi = [], r = [], s = [], th = [], T = [];
      let t = q.th0, rr = rIni, ss = 0, exceso = 0, rAnt = null;
      for (let i = 0; i < 3 * 360 * 4; i++) {
        let Ti = this.tensionDe(q, t, rr);
        // Corrección del modelo exacto: donde el par simulado salió τ_sim, el radio que
        // hace falta es r·τ0/τ_sim, o sea como si la tensión fuera T·τ_sim/τ0.
        if (corr && Ti != null) Ti = Ti * corr(i * dphi);
        // El brazo real del cable en el eje no es r sino p = r·cos γ, con γ el ángulo de
        // presión local (tan γ = r'/r): donde el perfil tiene pendiente hace falta algo
        // más de radio para el mismo par. Se usa la pendiente del paso anterior.
        const pend = rAnt == null ? 0 : (rr - rAnt) / dphi / rr;
        const fc = Math.sqrt(1 + pend * pend);
        const pide = (Ti != null && Ti > 1) ? Math.min(rIni, fc * q.tau0 * 1000 / Ti) : rIni;
        if (i > 0) {
          let techo = rr * (1 + tanG * dphi);
          if (rAnt != null) {
            const rp = (rr - rAnt) / dphi, kcap = (rr * rr + 2 * rp * rp) / rr;
            techo = Math.min(techo, 2 * rr - rAnt + 0.85 * kcap * dphi * dphi);   // 15 % de margen
          }
          const nuevo = Math.max(rr * (1 - tanG * dphi), Math.min(techo, pide));
          rAnt = rr; rr = nuevo;
        } else rr = pide;
        Ti = this.tensionDe(q, t, rr);
        if (Ti == null) break;
        // El exceso sólo cuenta en la entrada (hasta θ = 20°): es lo que decide el radio
        // de arranque. Cerca del punto muerto de arriba la tensión se dispara y el par se
        // pasa de τ0 haga lo que haga el arranque; eso se informa, no se "resuelve".
        // Se mide con la tensión corregida, que es la que da el par real.
        const Tc = corr ? Ti * corr(i * dphi) : Ti;
        if (t <= 20) exceso = Math.max(exceso, Tc * rr / 1000 - q.tau0);
        phi.push(i * dphi); r.push(rr); s.push(ss); th.push(t); T.push(Ti);
        if (t >= q.thfin) break;
        const tn = this.pasoTheta(q, t, rr, rr, -rr * dphi);
        if (tn == null) break;
        t = tn; ss += rr * dphi;
      }
      return { phi, r, s, th, T, exceso, barr: phi.length ? phi[phi.length - 1] / (2 * Math.PI) : 0 };
    };
    // Radio de arranque: el mayor, hasta rmax, con el que la bajada limitada no hace que
    // el par se pase de τ0 (más de un 1 %). Monótono: más arranque, más exceso.
    const disena = corr => {
      let lo = 5, hi = q.rmax, mejor = integra(lo, corr);
      if (integra(hi, corr).exceso <= 0.01 * q.tau0) mejor = integra(hi, corr);
      else {
        for (let i = 0; i < 16; i++) {
          const m = (lo + hi) / 2, p = integra(m, corr);
          if (p.exceso <= 0.01 * q.tau0) { lo = m; mejor = p; } else hi = m;
        }
      }
      return { ...mejor, rIni: mejor.r.length ? mejor.r[0] : q.rmax };
    };
    // Diseño con la tangencia a un círculo y luego dos correcciones con el contacto
    // exacto: se simula el perfil, y donde el contacto cae en ψ* con par τ_sim se escala
    // el radio del perfil EN ψ* por τ0/τ_sim. (Indexar por el giro φ en vez de por ψ*
    // era lo que hacía oscilar la corrección: el contacto va hasta 30° por detrás.)
    let perfil = disena(null);
    if (!q._sinIter) for (let it = 0; it < 2 && perfil.phi.length > 10; it++) {
      const q2 = { ...q, perfilForzado: perfil };
      const n = perfil.phi.length, fac = new Array(n).fill(1), cnt = new Array(n).fill(0);
      const rec = perfil.phi[n - 1];
      for (let j = 0; j <= 200; j++) {
        const e = this.estado(q2, rec * j / 200 * q.red / (2 * Math.PI));
        if (!e || e.fuera || e.psi == null || e.tau < 0.3 * q.tau0) continue;
        const i = Math.max(0, Math.min(n - 1, Math.round(e.psi / dphi)));
        fac[i] = (fac[i] * cnt[i] + e.tau / q.tau0) / (cnt[i] + 1); cnt[i]++;
      }
      // rellena huecos y suaviza
      let ult = 1; for (let i = 0; i < n; i++) { if (cnt[i]) ult = fac[i]; else fac[i] = ult; }
      const sua = fac.map((_, i) => { let a = 0, c = 0; for (let k = -8; k <= 8; k++) { const j = i + k; if (j >= 0 && j < n) { a += fac[j]; c++; } } return a / c; });
      const corr = psi => { const i = Math.max(0, Math.min(n - 1, Math.round(psi / dphi))); return Math.max(0.7, Math.min(1.4, sua[i])); };
      const prev = perfil; perfil = disena(corr);
      if (!perfil.phi.length) { perfil = prev; break; }
    }
    this._leva = { clave, perfil };
    this._levaPorQ.set(q, perfil);
    return perfil;
  },
  // Ángulo de presión de la leva: atan(|dr/dψ| / r), lo que el perfil se aparta de un
  // círculo en cada punto. El modelo supone que el cable sale tangente a un círculo de
  // radio r, y el cable real sólo se asienta en un perfil convexo y suave: por encima de
  // ~30° el cable puentea el escalón como una cuerda y ni el radio ni el par son los que
  // dice el modelo. Con rmax alto el arranque sale a 60-80°: el radio máximo es lo que
  // hay que bajar hasta que esto quede en 20-25°.
  presionLeva(q) {
    const p = this.perfilLeva(q);
    let max = 0, donde = 0;
    for (let i = 1; i < p.r.length; i++) {
      const dr = (p.r[i] - p.r[i - 1]) / (p.phi[i] - p.phi[i - 1]);
      const a = Math.atan(Math.abs(dr) / p.r[i]) * 180 / Math.PI;
      if (a > max) { max = a; donde = p.phi[i] * 180 / Math.PI; }
    }
    // Curvatura mínima normalizada: (r² + 2r'² − r·r'')/r², negativa = tramo cóncavo.
    let conv = Infinity;
    for (let i = 1; i < p.r.length - 1; i++) {
      const h = p.phi[i] - p.phi[i - 1], rp = (p.r[i + 1] - p.r[i - 1]) / (2 * h), rpp = (p.r[i + 1] - 2 * p.r[i] + p.r[i - 1]) / (h * h);
      conv = Math.min(conv, (p.r[i] ** 2 + 2 * rp * rp - p.r[i] * rpp) / p.r[i] ** 2);
    }
    return { max, donde, convexo: conv >= -1e-3, conv };
  },
  // r y s de la leva en un giro φ cualquiera: antes del arranque el radio se queda en el
  // inicial, pasado el final en el último, igual que la caracola fuera de su espiral.
  tamborLeva(q, phi) {
    const p = q.perfilForzado || this.perfilLeva(q), n = p.phi.length;
    if (!n) return { r: q.rmax, s: q.rmax * phi };
    if (phi <= 0) return { r: p.r[0], s: p.r[0] * phi };
    const fin = p.phi[n - 1];
    if (phi >= fin) return { r: p.r[n - 1], s: p.s[n - 1] + p.r[n - 1] * (phi - fin) };
    let lo = 0, hi = n - 1;
    while (hi - lo > 1) { const m = (lo + hi) >> 1; if (p.phi[m] <= phi) lo = m; else hi = m; }
    const f = (phi - p.phi[lo]) / (p.phi[hi] - p.phi[lo]);
    return { r: p.r[lo] + (p.r[hi] - p.r[lo]) * f, s: p.s[lo] + (p.s[hi] - p.s[lo]) * f };
  },

  // Tiro del cable: distancia real O→A' y cuánto se adelanta del eje de la barra.
  L3ef(q) { return Math.hypot(q.L3, q.e1 || 0); },
  delta(q) { return Math.atan2(q.e1 || 0, q.L3) * 180 / Math.PI; },

  // θ (barra desde la horizontal) ↔ α (ángulo en O entre O→C y la barra). Se configura θ
  // porque es lo que se mide, con un nivel sobre la barra; α es interno a la ley del coseno.
  alfaDe(q, th) { return this.incC(q) - th; },
  thDe(q, a)    { return this.incC(q) - a; },

  // Punto muerto: la línea del cable pasa por O. El brazo del cable se anula y la barra
  // no se puede levantar. No es un parámetro, es una propiedad de la geometría, y marca
  // el θ mínimo del mecanismo. Pasa cuando A' cae sobre la tangente trazada desde O al
  // tambor, por el otro lado de O: esa tangente va a γ = asin(r/|OC|) de la línea O→C,
  // hacia el lado del cable. Depende del radio; se da en B (r0), que es donde manda.
  //
  // Antes esto se configuraba con α acotado a [0°, 180°], y ahí estaba el fallo de fondo:
  // con la línea O→D inclinada 102.6°, α = 180° es θ = −77.4°, o sea que el modelo no
  // llegaba a representar la barra colgando y el ajuste tenía que sacar α a 114° (θ = −11°).
  // De ahí salía una curva de par decreciente, justo al revés que la medida.
  thMuerto(q) {
    const gam = Math.asin(Math.min(1, this.rHome(q) / this.oc(q))) * 180 / Math.PI;
    return this.incC(q) - this.lado(q) * gam - this.delta(q) - 180;
  },

  // Reposo de la barra SIN cable: la pesa queda justo bajo el pivote y su brazo se anula.
  // Es el sobre-centro del mecanismo, y como el cable sólo puede tirar marca el θ MÍNIMO
  // al que la barra puede quedarse: por debajo de aquí el brazo de la pesa cambia de
  // signo y la gravedad sube la barra sola.
  //
  // Se mide sin modelo de por medio: inclinómetro sobre la barra con el cable flojo. Y de
  // esa lectura sale e2, que es el parámetro más sensible del banco:
  //     e2 = (L3 + L4) / tan|θ_libre|
  // Con L3+L4 = 205 y e2 = 20 esto cae en −84.4°, y explica las lecturas minúsculas y
  // erráticas de la célula en B: ahí el mecanismo de verdad no necesita fuerza.
  thLibre(q) { return -Math.atan2(q.L3 + q.L4, q.e2 || 1e-9) * 180 / Math.PI; },

  // Distancia del eje del tambor a A' con la barra a α (ley del coseno en O, con β = α − δ).
  dCA(q, aDeg) {
    const L = this.oc(q), Le = this.L3ef(q);
    const co = Math.cos((aDeg - this.delta(q)) * Math.PI / 180);
    return Math.sqrt(L * L + Le * Le - 2 * L * Le * co);
  },
  // Cable libre, del punto de tangencia T al tiro A': cateto del triángulo rectángulo
  // C–T–A'. Si A' se mete dentro del tambor no hay tangente y se devuelve 0.
  cable(q, aDeg, r) {
    const d = this.dCA(q, aDeg), rr = r == null ? this.rHome(q) : r;
    return d > rr ? Math.sqrt(d * d - rr * rr) : 0;
  },
  // CABLE EQUIVALENTE g = c − lado·r·aT. Al moverse la barra el punto de tangencia T
  // migra por el tambor, y el cable que se enrolla o desenrolla por esa migración
  // (r por el ángulo que recorre T) NO lo paga el tambor. Lo que el tambor paga al girar
  // φ es Δg, no Δc: g(θ) = g(θ_B) − s(φ). Se comprueba por trabajos virtuales: −dg/dθ es
  // exactamente el brazo del cable en O, mientras que −dc/dθ se queda un 10-30 % corto.
  // Sin este término el modelo recogía ~10 mm de más por recorrido.
  gDe(q, th, r) { const t = this.tangente(q, th, r); return t ? t.g : null; },
  tensionDe(q, th, r) {
    const t = this.tangente(q, th, r);
    if (!t || t.brazo < 1e-6) return null;
    const thr = th * Math.PI / 180;
    const bp = (q.L3 + q.L4) * Math.cos(thr) + (q.e2 || 0) * Math.sin(thr);
    return bp > 0 ? q.m * q.g * bp / t.brazo : 0;
  },
  cHome(q) { return this.gDe(q, q.th0, this.rHome(q)); },   // cable equivalente en B
  // Topes del cable equivalente con el tambor a radio r: barra vertical (θ = 90°, el
  // mínimo, porque g baja al subir la barra) y punto muerto (el máximo).
  limites(q, r) {
    const rr = r == null ? this.rHome(q) : r, tm = this.thMuerto(q) + 0.05;
    return { cmin: this.gDe(q, 90, rr), cmax: this.gDe(q, Math.max(tm, -89.95), rr) };
  },

  // θ a partir del cable equivalente con el tambor a radio r. g es monótono decreciente
  // en θ entre el punto muerto y 90° (su pendiente es −brazo, y el brazo es positivo
  // ahí), así que basta una bisección. null si no hay solución.
  thetaDeG(q, g, r) {
    let lo = Math.max(this.thMuerto(q) + 0.05, -89.95), hi = 90;
    const gLo = this.gDe(q, lo, r), gHi = this.gDe(q, hi, r);
    if (gLo == null || gHi == null) return null;
    const tol = 1e-6 * Math.abs(gLo - gHi) + 1e-6;
    if (g > gLo + tol || g < gHi - tol) return null;
    for (let i = 0; i < 50; i++) {
      const m = (lo + hi) / 2;
      if (this.gDe(q, m, r) > g) lo = m; else hi = m;
    }
    return (lo + hi) / 2;
  },
  // Lo mismo devolviendo α, que es lo que esperaban los que llamaban aquí antes.
  barraAng(q, c, r) {
    const th = this.thetaDeG(q, c, r == null ? this.rHome(q) : r);
    return th == null ? null : this.incC(q) - th;
  },

  // Un paso de la cinemática: la barra está a θ con el cable sobre radio r, el tambor
  // cambia el cable equivalente en dg (negativo = recoge) y pasa a trabajar a radio rNew.
  // Se evalúa g con el MISMO radio rNew a los dos lados, para que el término r·aT no
  // meta un salto espurio aT·Δr cuando el radio cambia por el camino (en la caracola Δr
  // es 28 mm y aT ~0.3 rad: 8 mm de cable, un 7 % del recorrido, que es lo que faltaba
  // en el balance de energía con la fórmula cerrada).
  pasoTheta(q, th, r, rNew, dg) {
    const g0 = this.gDe(q, th, rNew);
    return g0 == null ? null : this.thetaDeG(q, g0 + dg, rNew);
  },

  // Trayectoria θ(φ) del mecanismo con el tambor configurado, integrada paso a paso desde
  // B hacia delante (hasta la barra vertical o 3 vueltas) y hacia atrás (hasta el punto
  // muerto o una vuelta). Es lo que consulta estado(): con el radio variando no hay
  // relación cerrada entre giro y ángulo. Se cachea por configuración.
  // Varias configuraciones a la vez (el simulador compara la leva con sus cilindros
  // equivalentes), así que la caché guarda unas cuantas, no sólo la última.
  // Punto de tangencia del cable sobre un CÍRCULO de radio r en C con la barra a θ, y el
  // brazo que hace en O. Es la aproximación con la que se diseña la leva paso a paso y
  // la semilla del contacto exacto; el estado del mecanismo usa contacto().
  tangente(q, th, r) {
    const Le = this.L3ef(q), ang = (th + this.delta(q)) * Math.PI / 180;
    const Ap = { x: Le * Math.cos(ang), y: Le * Math.sin(ang) };
    const C = { x: -q.xt, y: q.L2 };
    const vx = C.x - Ap.x, vy = C.y - Ap.y, d = Math.hypot(vx, vy);
    if (!(d > r)) return null;
    const c = Math.sqrt(d * d - r * r);
    const dir = Math.atan2(vy, vx) - this.lado(q) * Math.asin(r / d);
    const u = { x: Math.cos(dir), y: Math.sin(dir) };            // de A' hacia T
    const tg = { x: Ap.x + c * u.x, y: Ap.y + c * u.y };
    const aT = Math.atan2(tg.y - C.y, tg.x - C.x);
    return { Ap, tg, u, c, aT, brazo: Ap.x * u.y - Ap.y * u.x, g: c - this.lado(q) * r * aT };
  },

  // ── Contacto exacto del cable con el perfil ──
  // El cable es una recta que sale de A' y toca el perfil donde es TANGENTE a él, no a un
  // círculo del radio local: en una leva o una espiral el perfil tiene pendiente, y la
  // recta tangente a un círculo de radio r cortaría el lóbulo que viene detrás. El brazo
  // del par en el tambor es la distancia del eje a esa recta (menor que r donde hay
  // pendiente), y el cable enrollado es la longitud de ARCO del perfil, no r·ángulo.
  //
  // Perfil en el sistema del tambor: r(ψ), con ψ el mismo parámetro de tambor(). El
  // punto ψ queda en el mundo, con el tambor girado φ, en el ángulo
  //     a(ψ, φ) = aRef + sl·(φ − ψ),   sl = sentido·lado
  // de modo que al girar φ el punto ψ = φ pasa por el ángulo aRef, y aRef se fija para
  // que en B (φ = 0, θ = θ0) el cable toque justo en ψ = 0.
  sl(q) { return q.sentido * this.lado(q); },
  rPerfil(q, psi) { return this.tambor(q, psi).r; },
  rPerfilD(q, psi) { const h = 2e-3; return (this.tambor(q, psi + h).r - this.tambor(q, psi - h).r) / (2 * h); },
  // Longitud de arco desde ψ = 0, tabulada por configuración (de −2 a +6 vueltas).
  _arco: new Map(),
  _arcoPorQ: new WeakMap(),
  arco(q, psi) {
    let t = this._arcoPorQ.get(q);
    if (!t) {
      const clave = JSON.stringify([q.tipo, q.L1, q.r0, q.r1, q.barr, q.tau0, q.rmax, q.thfin, q.th0, q.xt, q.L2, q.L3, q.L4, q.e1, q.e2, q.m, q.g]);
      t = this._arco.get(clave);
      if (t) this._arcoPorQ.set(q, t);
      else {
      const h = 0.5 * Math.PI / 180, ini = -2 * 2 * Math.PI, n = Math.round(8 * 2 * Math.PI / h);
      const L = new Array(n + 1); L[0] = 0;
      for (let i = 1; i <= n; i++) {
        const a = ini + (i - 0.5) * h, r = this.rPerfil(q, a), rp = this.rPerfilD(q, a);
        L[i] = L[i - 1] + Math.hypot(r, rp) * h;
      }
      const i0 = Math.round(-ini / h), L0 = L[i0];
      t = { ini, h, L: L.map(v => v - L0) };
      if (this._arco.size >= 8) this._arco.delete(this._arco.keys().next().value);
      this._arco.set(clave, t); this._arcoPorQ.set(q, t);
      }
    }
    const x = (psi - t.ini) / t.h, i = Math.max(0, Math.min(t.L.length - 2, Math.floor(x))), f = x - i;
    return t.L[i] + (t.L[i + 1] - t.L[i]) * f;
  },
  // Punto del perfil y su tangente en el mundo.
  puntoPerfil(q, psi, phi, aRef) {
    const sl = this.sl(q), a = aRef + sl * (phi - psi), r = this.rPerfil(q, psi), rp = this.rPerfilD(q, psi);
    const ca = Math.cos(a), sa = Math.sin(a);
    return { x: -q.xt + r * ca, y: q.L2 + r * sa, r, a,
             tx: rp * ca + sl * r * sa, ty: rp * sa - sl * r * ca };   // dP/dψ
  },
  ApDe(q, th) {
    const Le = this.L3ef(q), ang = (th + this.delta(q)) * Math.PI / 180;
    return { x: Le * Math.cos(ang), y: Le * Math.sin(ang) };
  },
  // Punto de contacto: ψ* con cross(P − A', dP/dψ) = 0. Se busca el cambio de signo más
  // cercano a 'cerca' (el contacto del paso anterior, o φ) y se biseca.
  contacto(q, th, phi, aRef, cerca) {
    const Ap = this.ApDe(q, th);
    const f = psi => { const P = this.puntoPerfil(q, psi, phi, aRef); return (P.x - Ap.x) * P.ty - (P.y - Ap.y) * P.tx; };
    let lo = null, hi = null;
    for (const anchura of [0.06, 0.25, 1.0, 2.5]) {
      const n = 12; let prev = cerca - anchura, fp = f(prev);
      let mejor = null;
      for (let i = 1; i <= 2 * n; i++) {
        const x = cerca - anchura + anchura * i / n, fx = f(x);
        if (fp * fx <= 0) { const d = Math.abs((prev + x) / 2 - cerca); if (mejor == null || d < mejor.d) mejor = { lo: prev, hi: x, d }; }
        prev = x; fp = fx;
      }
      if (mejor) { lo = mejor.lo; hi = mejor.hi; break; }
    }
    if (lo == null) return null;
    let flo = f(lo);
    for (let i = 0; i < 30; i++) { const m = (lo + hi) / 2, fm = f(m); if (flo * fm <= 0) hi = m; else { lo = m; flo = fm; } }
    const psi = (lo + hi) / 2, P = this.puntoPerfil(q, psi, phi, aRef);
    const dx = P.x - Ap.x, dy = P.y - Ap.y, c = Math.hypot(dx, dy);
    if (!(c > 1e-9)) return null;
    const u = { x: dx / c, y: dy / c };
    // brazo en O (signo + = tira hacia arriba) y brazo en el eje del tambor (la "r" eficaz)
    const brazo = Ap.x * u.y - Ap.y * u.x;
    const p = Math.abs((-q.xt - Ap.x) * u.y - (q.L2 - Ap.y) * u.x);
    return { psi, P, Ap, u, c, brazo, p, rc: P.r };
  },
  // aRef: orientación del tambor tal que en B el contacto cae en ψ = 0.
  aRefDe(q) {
    const t0 = this.tangente(q, q.th0, this.rPerfil(q, 0));
    const a0 = t0 ? t0.aT : (this.lado(q) > 0 ? 0 : Math.PI);
    const Ap = this.ApDe(q, q.th0);
    const f = aRef => { const P = this.puntoPerfil(q, 0, 0, aRef); return (P.x - Ap.x) * P.ty - (P.y - Ap.y) * P.tx; };
    let lo = a0 - 0.7, hi = a0 + 0.7, flo = f(lo);
    if (flo * f(hi) > 0) return a0;
    for (let i = 0; i < 40; i++) { const m = (lo + hi) / 2, fm = f(m); if (flo * fm <= 0) hi = m; else { lo = m; flo = fm; } }
    return (lo + hi) / 2;
  },
  // Cable total desde el anclaje: libre + arco enrollado hasta el contacto. Es lo que se
  // conserva al girar el tambor.
  G(q, th, phi, aRef, cerca) {
    const k = this.contacto(q, th, phi, aRef, cerca);
    return k ? { G: k.c + this.arco(q, k.psi), k } : null;
  },

  // Trayectoria θ(φ): para cada giro φ, el θ que conserva el cable total. Integrada
  // desde B hacia delante (hasta la barra vertical o 3 vueltas) y hacia atrás (hasta
  // que deja de haber contacto o una vuelta). Se cachea por configuración; varias a la
  // vez, porque el simulador compara la leva con sus cilindros equivalentes.
  _tray: new Map(),
  trayectoria(q) {
    const clave = JSON.stringify(q);
    if (this._tray.has(clave)) return this._tray.get(clave);
    const dphi = 0.5 * Math.PI / 180, aRef = this.aRefDe(q);
    const g0 = this.G(q, q.th0, 0, aRef, 0);
    const vacio = { phi: [0], th: [q.th0], psi: [0], aRef, G0: g0 ? g0.G : 0 };
    if (!g0) { this._tray.set(clave, vacio); return vacio; }
    const avanza = signo => {
      const phi = [], th = [], psi = [];
      let t = q.th0, ps = 0;
      for (let i = 1; i <= (signo > 0 ? 3 * 360 * 2 : 360 * 2); i++) {
        const p1 = signo * i * dphi;
        // θ que conserva G: G baja al subir la barra, así que se biseca en un entorno de
        // la θ anterior (ensanchando si hace falta).
        let lo = t - 1.5, hi = t + 1.5, gl = null, gh = null, ok = false;
        for (let k = 0; k < 6 && !ok; k++) {
          gl = this.G(q, lo, p1, aRef, ps); gh = this.G(q, hi, p1, aRef, ps);
          if (gl && gh && (gl.G - g0.G) * (gh.G - g0.G) <= 0) ok = true;
          else { lo -= 3 * (k + 1); hi += 3 * (k + 1); }
          if (lo < -90 || hi > 90) break;
        }
        if (!ok) break;
        for (let k = 0; k < 22; k++) {
          const m = (lo + hi) / 2, gm = this.G(q, m, p1, aRef, ps);
          if (!gm) break;
          if ((gl.G - g0.G) * (gm.G - g0.G) <= 0) hi = m; else { lo = m; gl = gm; }
        }
        t = (lo + hi) / 2;
        const gk = this.G(q, t, p1, aRef, ps);
        if (!gk || t < -90 || t > 90) break;
        ps = gk.k.psi;
        phi.push(p1); th.push(t); psi.push(ps);
        if (t >= 90 - 1e-6) break;
      }
      return { phi, th, psi };
    };
    const ade = avanza(1), atr = avanza(-1);
    const tray = { phi: atr.phi.reverse().concat([0], ade.phi), th: atr.th.reverse().concat([q.th0], ade.th),
                   psi: atr.psi.reverse().concat([0], ade.psi), aRef, G0: g0.G };
    if (this._tray.size >= 8) this._tray.delete(this._tray.keys().next().value);
    this._tray.set(clave, tray);
    return tray;
  },
  // θ y ψ de contacto en un giro φ cualquiera, interpolando en la trayectoria.
  thetaEn(q, phi) {
    const t = this.trayectoria(q), n = t.phi.length;
    if (!n || phi < t.phi[0] - 1e-9 || phi > t.phi[n - 1] + 1e-9) return null;
    let lo = 0, hi = n - 1;
    while (hi - lo > 1) { const m = (lo + hi) >> 1; if (t.phi[m] <= phi) lo = m; else hi = m; }
    if (hi === lo) return { th: t.th[lo], psi: t.psi[lo] };
    const f = Math.max(0, Math.min(1, (phi - t.phi[lo]) / (t.phi[hi] - t.phi[lo])));
    return { th: t.th[lo] + (t.th[hi] - t.th[lo]) * f, psi: t.psi[lo] + (t.psi[hi] - t.psi[lo]) * f };
  },

  // Estado completo del mecanismo con el ciclo avanzado avRev vueltas de motor desde B
  // (el arranque del ensayo). El avance es positivo hacia A; negativo sólo si el eje se
  // pasa de B. Quien llama lo saca de pos_rev: avance = (pos_rev − pos_rev_B) · sentido
  // del barrido, porque el ciclo B → A va hacia posiciones menores.
  estado(q, avRev) {
    const phi = avRev * 2 * Math.PI / q.red;
    const t = this.tambor(q, phi), tray = this.trayectoria(q);
    const base = { avRev, phi, r: t.r, s: t.s, g: 0, c: 0, aRef: tray.aRef };
    const e = this.thetaEn(q, phi);
    // Sin ángulo no hay estado: se sale marcando fuera, pero con a y th a null explícito
    // para que quien pinte sepa que no hay número en vez de recibir un NaN camuflado.
    if (e == null) return { ...base, fuera: true, a: null, th: null };
    const th = e.th, a = this.incC(q) - th;
    if (!(th >= -90 && th <= 90)) return { ...base, fuera: true, a, th };
    // Contacto exacto: dónde toca el cable, cuánto cable libre, brazo en O y brazo en el
    // eje del tambor (la r eficaz, que es la que multiplica a la tensión).
    const k = this.contacto(q, th, phi, tray.aRef, e.psi);
    if (!k || k.brazo < 1e-6) return { ...base, fuera: true, a, th };
    base.tg = k.P; base.Ap = k.Ap; base.c = k.c; base.psi = k.psi; base.rc = k.rc; base.r = k.p;
    base.s = this.arco(q, k.psi);                       // cable enrollado desde B
    base.g = tray.G0 - k.c;
    const thr = th * Math.PI / 180;
    // Brazo del peso: colgar por debajo del eje añade componente horizontal al inclinarse.
    const brazoPeso = (q.L3 + q.L4) * Math.cos(thr) + (q.e2 || 0) * Math.sin(thr);
    const brazoCable = k.brazo;
    // Pasado el sobre-centro (thLibre) el brazo de la pesa cambia de signo: para sostenerla
    // el cable tendría que EMPUJAR. No puede: ahí queda flojo y la barra se apoya en el
    // tope. Sin esto la fórmula devolvía una tensión negativa y se pintaba tal cual.
    if (brazoPeso <= 0)
      return { ...base, fuera: true, flojo: true, a, th, T: 0, tau: 0, brazoCable, brazoPeso };
    const T = q.m * q.g * brazoPeso / brazoCable;
    return { ...base, fuera: false, a, th, T, tau: T * k.p / 1000, brazoCable, brazoPeso };
  },

  // Inversa: avance en vueltas de motor desde B para llevar la barra a α = aDeg. Se busca
  // el giro φ en el que el cable que deja el tambor coincide con el cable libre que pide
  // esa barra (con el radio de ese mismo giro). El cable pagado crece con φ mucho más
  // deprisa de lo que el radio cambia el cable libre, así que basta una bisección. null
  // si no se llega.
  revParaAngulo(q, aDeg) {
    const obj = this.thDe(q, aDeg), t = this.trayectoria(q);
    for (let i = 1; i < t.phi.length; i++) {
      const a = t.th[i - 1], b = t.th[i];
      if ((a - obj) * (b - obj) <= 0 && a !== b) {
        const phi = t.phi[i - 1] + (t.phi[i] - t.phi[i - 1]) * (obj - a) / (b - a);
        return phi * q.red / (2 * Math.PI);
      }
    }
    return null;
  },

  // Lo mismo pidiendo θ, que es como se habla del mecanismo fuera de aquí.
  revParaTheta(q, thDeg) {
    if (!(thDeg >= this.thMuerto(q) && thDeg <= 90)) return null;
    return this.revParaAngulo(q, this.alfaDe(q, thDeg));
  },

  // Recorrido entre B y A en vueltas de motor. No es un parámetro: lo fija la mecánica,
  // el barrido del tambor por la reducción entre el eje del motor y el tambor.
  recorridoMotor(q) { return this.barrido(q) * q.red; },

  // Lo que el servidor también rechaza, para avisar antes de enviar.
  validar(q) {
    const num = ['L1', 'r0', 'r1', 'barr', 'L2', 'xt', 'ob', 'L3', 'L4', 'e1', 'e2', 'm', 'g', 'th0', 'red', 'tau0', 'rmax', 'thfin'];
    for (const k of num) if (!Number.isFinite(q[k])) return `${k} no es un número`;
    if (!['car', 'cil', 'leva'].includes(q.tipo)) return 'tipo tiene que ser car, cil o leva';
    const tm = this.thMuerto(q);
    if (!(q.th0 > tm && q.th0 <= 90))
      return `θ en B va de ${tm.toFixed(1)}° (punto muerto: la línea del cable pasa por O`
           + ` y el brazo se anula) a 90° (barra vertical hacia arriba)`;
    for (const k of ['L1', 'r0', 'r1', 'barr', 'L2', 'L3', 'red'])
      if (!(q[k] > 0)) return `${k} tiene que ser mayor que 0`;
    if (q.L4 < 0 || q.m < 0 || q.g < 0) return 'L4, masa y g no pueden ser negativos';
    if (q.tipo === 'leva') {
      if (!(q.tau0 > 0 && q.rmax > 0)) return 'τ0 y rmax tienen que ser mayores que 0';
      if (!(q.thfin > q.th0 && q.thfin <= 90)) return 'θ final de la leva tiene que estar entre θ en B y 90°';
      if (!this.perfilLeva(q).phi.length) return 'la leva no arranca: con ese τ0 y rmax no hay solución en B';
    }
    return null;
  }
};

// ── Validación contra el ensayo del 30-09 ────────────────────────────────
// Dos comprobaciones independientes, sin ajustar ningún parámetro:
//
//  1. Cable. La caracola medida (28.18 → 56 mm sobre 0.367 vueltas de tambor) recoge
//     97.1 mm. La barra subiendo de −80° a +10° pide 95.9 mm. Difieren un 1.2%.
//
//  2. Par. Separando el par medido en gravedad (subida+bajada)/2 y fricción
//     (subida−bajada)/2, la fricción sale plana en ~6.2 unidades de drive —rozamiento
//     seco, como debe ser— y la gravedad queda con un error rms del 10% frente a este
//     modelo en la zona de velocidad constante, con la escala de unidades del drive como
//     único grado de libertad (1 unidad ≈ 0.055 N·m).
//
// La tensión del cable sale casi constante (≈ 43-48 N) en todo el recorrido: el brazo de
// la pesa y el brazo del cable crecen a la vez y se cancelan. O sea que la forma de la
// curva de par la pone la caracola, no la barra. Que es para lo que sirve una caracola.

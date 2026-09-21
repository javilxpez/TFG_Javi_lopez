// ── Modelo del mecanismo ─────────────────────────────────────────────────
// Una sola implementación de la geometría para /sim y /ensayos. Antes cada página
// llevaba su copia de las fórmulas y sus propios valores por defecto, y ya se habían
// separado: el tab teórico del monitor seguía con un cilindro y la barra horizontal
// mientras los ensayos usaban la caracola. La configuración vive en el servidor
// (mecanismo.json) y se edita sólo desde /sim.
//
// Convención, la misma que dibuja /sim:
//   O   articulación de la barra, en el origen.
//   D   salida del cable: NO está encima de O. Sube L2 y queda dx a la izquierda,
//       D = (−dx, L2). Con dx = 0 se recupera el mástil vertical de antes.
//   α   ángulo en O entre la línea O→D y la barra, de 0° a 180°. Es el ángulo con
//       el que se configura el home: 0° barra plegada sobre el mástil, 90° perpendicular,
//       180° barra en prolongación del mástil. En ese rango cada α da un cable distinto:
//       no hay dos ángulos con el mismo cable, así que el ángulo nunca es ambiguo.
//   θ   = 90° − α, la misma barra medida desde la horizontal. Sólo se usa para dibujar.
//   A   punto del EJE de la barra a L3 de O. El cable no tira de ahí: tira de A' , que
//       está e1 por encima en perpendicular a la barra. Y la pesa no cuelga del eje,
//       sino de P' , e2 por debajo, a L3+L4 de O.
//         A' = L3·û + e1·n̂      P' = (L3+L4)·û − e2·n̂
//       con û = (cos θ, sen θ) la barra y n̂ = (−sen θ, cos θ) su perpendicular.
//       Eso mete dos cambios: el tiro se aleja de O —L3' = √(L3² + e1²), adelantado
//       δ = atan(e1/L3) respecto del eje— y el brazo del peso deja de ser (L3+L4)·cos θ.
//   c   cable entre D y A (ley del coseno):  c² = |OD|² + L3² − 2·|OD|·L3·cos α,
//       con |OD| = √(L2² + dx²).
//
// El desplazamiento dx cambia la física, no sólo el dibujo: con D encima de O la línea
// O→D era vertical, θ = 90 − α, y el cos θ del peso se cancelaba con el sen α del cable,
// dejando T proporcional al cable que queda. Inclinada esa línea ya no se cancelan, y en
// α = 180° el cable queda alineado con la barra —brazo cero— con la pesa aún haciendo
// momento: ahí la tensión se va a infinito y el mecanismo no puede sostener la carga.
//
// Equilibrio de momentos en O (el cable tira de A hacia D, la pesa cuelga en L3+L4):
//   β = α − δ                       ángulo en O entre O→D y O→A'
//   c² = |OD|² + L3'² − 2·|OD|·L3'·cos β
//   d = |OD|·L3'·sen β / c           brazo del cable
//   b = (L3+L4)·cos θ + e2·sen θ     brazo del peso
//   T·d = m·g·b   →   T = m·g·b·c / (|OD|·L3'·sen β)
// Con e1 = e2 = 0 queda L3' = L3, δ = 0, β = α y b = (L3+L4)·cos θ: la fórmula anterior.
// y el par en el tambor es τ = T·r(φ) con el radio instantáneo de la caracola.

const MEC_DEFAULTS = {
  tipo: 'car',        // 'car' caracola de radio variable, 'cil' cilindro de radio fijo
  L1: 25,             // radio del cilindro (mm), sólo con tipo 'cil'
  // Valores del ajuste sobre los ensayos limpios del 21-09 (R² 0.93 en los dos, con
  // dx = 65): radio grande en B y decreciendo al subir. Conviene confirmarlos midiendo.
  r0: 45, r1: 25,     // radio de la caracola en B y al final del barrido (mm)
  barr: 0.3,          // vueltas de tambor del barrido (en la caracola, lo que dura la espiral)
  L2: 290,            // altura de D sobre O (mm)
  dx: 65,             // D desplazada a la izquierda de O (mm): 25 de D + 40 de O
  L3: 95,             // de O al punto del eje donde tira el cable (mm)
  L4: 110,            // resto de barra hasta la pesa (mm)
  e1: 20,             // el cable tira e1 por encima del eje de la barra (mm)
  e2: 20,             // la pesa cuelga e2 por debajo del eje (mm)
  m: 2, g: 9.81,
  a0: 114,            // α al empezar el ensayo, en B (°); del ajuste. 180 es singular con dx ≠ 0
  red: 5.5,           // vueltas de motor por vuelta de tambor (del ajuste; la 3:15 daría 5)
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
    const phitot = q.barr * 2 * Math.PI;
    if (phi <= 0) return { r: q.r0, s: q.r0 * phi };
    if (phi <= phitot) {
      const r = q.r0 + (q.r1 - q.r0) * phi / phitot;
      return { r, s: q.r0 * phi + (q.r1 - q.r0) * phi * phi / (2 * phitot) };
    }
    const sTot = phitot * (q.r0 + q.r1) / 2;
    return { r: q.r1, s: sTot + q.r1 * (phi - phitot) };
  },

  // Salida del cable: distancia O→D y hacia dónde apunta (grados desde el eje +x, con
  // la barra tendida hacia +x). Con dx = 0 la inclinación es 90°, la vertical de siempre.
  od(q)   { return Math.hypot(q.dx, q.L2); },
  incD(q) { return Math.atan2(q.L2, -q.dx) * 180 / Math.PI; },

  // Tiro del cable: distancia real O→A' y cuánto se adelanta del eje de la barra.
  L3ef(q) { return Math.hypot(q.L3, q.e1 || 0); },
  delta(q) { return Math.atan2(q.e1 || 0, q.L3) * 180 / Math.PI; },

  cable(q, aDeg) {
    const L = this.od(q), Le = this.L3ef(q);
    const co = Math.cos((aDeg - this.delta(q)) * Math.PI / 180);
    return Math.sqrt(L * L + Le * Le - 2 * L * Le * co);
  },
  cHome(q)   { return this.cable(q, q.a0); },   // cable en B, donde empieza el ensayo
  limites(q) { const L = this.od(q), Le = this.L3ef(q); return { cmin: Math.abs(L - Le), cmax: L + Le }; },

  // α a partir del cable. cos α es monótono en [0°, 180°]: un cable, un ángulo.
  barraAng(q, c) {
    const L = this.od(q), Le = this.L3ef(q);
    let co = (L * L + Le * Le - c * c) / (2 * L * Le);
    // En los extremos exactos (c = |OD| ± L3) el redondeo saca el coseno de [−1, 1] por
    // 1e-16 y esto devolvía null. Un null se cuela hasta quien lo pinta y le estalla,
    // así que se recorta aquí: a esa distancia el ángulo es 0° o 180° y ya está.
    if (co > 1 && co < 1 + 1e-9) co = 1;
    if (co < -1 && co > -1 - 1e-9) co = -1;
    if (co < -1 || co > 1) return null;
    // acos da β, el ángulo hasta O→A'. α, que es lo que se configura, va al eje de la barra.
    return Math.acos(co) * 180 / Math.PI + this.delta(q);
  },

  // Estado completo del mecanismo con el ciclo avanzado avRev vueltas de motor desde B
  // (el arranque del ensayo). El avance es positivo hacia A; negativo sólo si el eje se
  // pasa de B. Quien llama lo saca de pos_rev: avance = (pos_rev − pos_rev_B) · sentido
  // del barrido, porque el ciclo B → A va hacia posiciones menores.
  estado(q, avRev) {
    const phi = avRev * 2 * Math.PI / q.red;
    const t = this.tambor(q, phi);
    const c = this.cHome(q) - q.sentido * t.s;
    const { cmin, cmax } = this.limites(q);
    const base = { avRev, phi, r: t.r, s: t.s, c };
    if (!(c >= cmin && c <= cmax)) return { ...base, fuera: true };
    const a = this.barraAng(q, c);
    // Sin ángulo no hay estado: se sale marcando fuera, pero con a y th a null explícito
    // para que quien pinte sepa que no hay número en vez de recibir un NaN camuflado.
    if (a == null) return { ...base, fuera: true, a: null, th: null };
    // θ es la barra desde la horizontal; fuera de ±90° no hay barra que valga.
    const th = this.incD(q) - a;
    if (!(th >= -90 && th <= 90)) return { ...base, fuera: true, a, th };
    // Con la línea O→D inclinada, cos θ y sen α ya no se cancelan. En α → 180° el brazo
    // del cable se anula con la pesa aún cargando: la tensión no tiene solución finita.
    const sb = Math.sin((a - this.delta(q)) * Math.PI / 180);
    if (sb < 1e-6) return { ...base, fuera: true, a, th };
    const thr = th * Math.PI / 180;
    // Brazo del peso: colgar por debajo del eje añade componente horizontal al inclinarse.
    const brazoPeso = (q.L3 + q.L4) * Math.cos(thr) + (q.e2 || 0) * Math.sin(thr);
    const brazoCable = this.od(q) * this.L3ef(q) * sb / c;
    const T = q.m * q.g * brazoPeso / brazoCable;
    return { ...base, fuera: false, a, th, T, tau: T * t.r / 1000, brazoCable, brazoPeso };
  },

  // Inversa: avance en vueltas de motor desde B para llevar la barra a α = aDeg. El cable
  // pagado es monótono en φ, así que basta una bisección. null si no se llega.
  revParaAngulo(q, aDeg) {
    if (aDeg < 0 || aDeg > 180) return null;
    const sObj = (this.cHome(q) - this.cable(q, aDeg)) / q.sentido;
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

  // Recorrido entre B y A en vueltas de motor. No es un parámetro: lo fija la mecánica,
  // el barrido del tambor por la reducción entre el eje del motor y el tambor.
  recorridoMotor(q) { return q.barr * q.red; },

  // Lo que el servidor también rechaza, para avisar antes de enviar.
  validar(q) {
    const num = ['L1', 'r0', 'r1', 'barr', 'L2', 'dx', 'L3', 'L4', 'e1', 'e2', 'm', 'g', 'a0', 'red'];
    for (const k of num) if (!Number.isFinite(q[k])) return `${k} no es un número`;
    if (q.a0 < 0 || q.a0 > 180)
      return 'α en el home va de 0° (barra plegada sobre el mástil) a 180° (barra en prolongación del mástil)';
    for (const k of ['L1', 'r0', 'r1', 'barr', 'L2', 'L3', 'red'])
      if (!(q[k] > 0)) return `${k} tiene que ser mayor que 0`;
    if (q.L4 < 0 || q.m < 0 || q.g < 0) return 'L4, masa y g no pueden ser negativos';
    return null;
  }
};

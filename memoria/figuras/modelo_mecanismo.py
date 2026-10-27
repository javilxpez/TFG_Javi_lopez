"""Modelo del mecanismo: port fiel de static/mecanismo.js (modelo TANGENTE, 07-10-2026).

Sustituye al modelo anterior, que situaba la salida del cable en un punto fijo D
desplazado 'dx' de la vertical de O. Aquel modelo no podia representar la barra
colgando y el ajuste lo compensaba invirtiendo la caracola, de donde salian las
conclusiones del 30-09 (incluida la del tambor excentrico), hoy superadas.

Diferencias de fondo con el modelo viejo:
  - El cable sale TANGENTE al tambor, cuyo eje C = (-xt, L2). Al cambiar el radio la
    linea del cable se desplaza r1 - r0: un punto fijo no puede representarlo.
  - El cable que el tambor paga no es el cable libre c, sino el cable equivalente
    g = c - lado*r*aT: al moverse la barra el punto de tangencia migra por el tambor
    y enrolla cable que el tambor no ha pagado.
  - CONTACTO EXACTO: el cable toca el perfil donde es tangente AL PERFIL, no a un
    circulo del radio local. El brazo del par es la distancia del eje a esa recta
    (menor que r donde el perfil tiene pendiente) y el cable enrollado es longitud
    de ARCO del perfil.
  - La barra lleva desfases: el cable tira de A', e1 por encima del eje, y la pesa
    cuelga de P', e2 por debajo. Eso adelanta el tiro y cambia el brazo del peso.

No se porta el diseno de la leva de par constante (perfilLeva): sigue solo en el JS,
y la conclusion del 07-10 es que no se recomienda.

Validacion: ejecutar este fichero (`python modelo_mecanismo.py`) comprueba las tres
cifras que documenta mecanismo.js.
"""
import csv, json, math, os
import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(os.path.dirname(AQUI))
D = math.pi / 180

MEC_DEFAULTS = dict(
    tipo='car', L1=25.0, r0=28.18, r1=56.0, barr=0.367,
    L2=290.0, xt=30.0, ob=30.0, L3=95.0, L4=110.0, e1=20.0, e2=20.0,
    m=2.0, g=9.81, th0=-87.0, tau0=1.5, rmax=70.0, thfin=3.0, mes0=0.0,
    red=5.0, sentido=1)


def carga_q(ruta=None):
    """Configuracion del servidor (mecanismo.json) sobre los valores por defecto."""
    q = dict(MEC_DEFAULTS)
    p = ruta or os.path.join(RAIZ, 'mecanismo.json')
    if os.path.exists(p):
        q.update(json.load(open(p, encoding='utf-8')))
    return q


# ── Tambor ────────────────────────────────────────────────────────────────
def tambor(q, phi):
    """Radio y cable pagado para un giro phi (rad) del tambor desde B, con signo.

    'mes0' (vueltas de tambor) es una EXTENSION del modelo del JS: el perfil real
    no empieza a estrecharse en el punto de arranque, sino que lleva un arco de
    radio constante r0 antes de la espiral. Con mes0 = 0 esto se reduce exactamente
    a la ley de mecanismo.js.
    """
    if q['tipo'] == 'cil':
        return q['L1'], q['L1'] * phi
    if q['tipo'] == 'leva':
        raise NotImplementedError("el perfil de leva solo esta en mecanismo.js")
    phitot = q['barr'] * 2 * math.pi
    mes = q.get('mes0', 0.0) * 2 * math.pi
    if phi <= 0:
        return q['r0'], q['r0'] * phi
    if phi <= mes:
        return q['r0'], q['r0'] * phi
    u = phi - mes
    sMes = q['r0'] * mes
    if u <= phitot:
        r = q['r0'] + (q['r1'] - q['r0']) * u / phitot
        return r, sMes + q['r0'] * u + (q['r1'] - q['r0']) * u * u / (2 * phitot)
    sTot = sMes + phitot * (q['r0'] + q['r1']) / 2
    return q['r1'], sTot + q['r1'] * (u - phitot)


def oc(q):    return math.hypot(q['xt'], q['L2'])
def incC(q):  return math.atan2(q['L2'], -q['xt']) / D
def lado(q):  return 1 if q['xt'] >= 0 else -1
def rHome(q): return q['r0'] if q['tipo'] == 'car' else (q['rmax'] if q['tipo'] == 'leva' else q['L1'])
def barrido(q): return q['barr']
def L3ef(q):  return math.hypot(q['L3'], q.get('e1', 0.0))
def delta(q): return math.atan2(q.get('e1', 0.0), q['L3']) / D
def alfaDe(q, th): return incC(q) - th
def thDe(q, a):    return incC(q) - a


def thMuerto(q):
    """Punto muerto: la linea del cable pasa por O y el brazo se anula."""
    gam = math.asin(min(1.0, rHome(q) / oc(q))) / D
    return incC(q) - lado(q) * gam - delta(q) - 180


def thLibre(q):
    """Reposo sin cable: la pesa queda bajo el pivote. theta minimo del mecanismo."""
    return -math.atan2(q['L3'] + q['L4'], q.get('e2', 0.0) or 1e-9) / D


def dCA(q, aDeg):
    L, Le = oc(q), L3ef(q)
    co = math.cos((aDeg - delta(q)) * D)
    return math.sqrt(L * L + Le * Le - 2 * L * Le * co)


def cable(q, aDeg, r=None):
    d = dCA(q, aDeg)
    rr = rHome(q) if r is None else r
    return math.sqrt(d * d - rr * rr) if d > rr else 0.0


def ApDe(q, th):
    Le, ang = L3ef(q), (th + delta(q)) * D
    return Le * math.cos(ang), Le * math.sin(ang)


def tangente(q, th, r):
    """Tangencia a un CIRCULO de radio r: semilla del contacto exacto."""
    Apx, Apy = ApDe(q, th)
    Cx, Cy = -q['xt'], q['L2']
    vx, vy = Cx - Apx, Cy - Apy
    d = math.hypot(vx, vy)
    if not (d > r):
        return None
    c = math.sqrt(d * d - r * r)
    dir_ = math.atan2(vy, vx) - lado(q) * math.asin(r / d)
    ux, uy = math.cos(dir_), math.sin(dir_)
    tgx, tgy = Apx + c * ux, Apy + c * uy
    aT = math.atan2(tgy - Cy, tgx - Cx)
    return dict(Ap=(Apx, Apy), tg=(tgx, tgy), u=(ux, uy), c=c, aT=aT,
                brazo=Apx * uy - Apy * ux, g=c - lado(q) * r * aT)


def tensionDe(q, th, r):
    t = tangente(q, th, r)
    if not t or t['brazo'] < 1e-6:
        return None
    thr = th * D
    bp = (q['L3'] + q['L4']) * math.cos(thr) + q.get('e2', 0.0) * math.sin(thr)
    return q['m'] * q['g'] * bp / t['brazo'] if bp > 0 else 0.0


# ── Contacto exacto del cable con el perfil ───────────────────────────────
def sl(q): return q['sentido'] * lado(q)
def rPerfil(q, psi): return tambor(q, psi)[0]


def rPerfilD(q, psi):
    h = 2e-3
    return (tambor(q, psi + h)[0] - tambor(q, psi - h)[0]) / (2 * h)


_ARCO = {}


def arco(q, psi):
    """Longitud de arco del perfil desde psi = 0, tabulada (de -2 a +6 vueltas)."""
    clave = (q['tipo'], q['L1'], q['r0'], q['r1'], q['barr'], q.get('mes0', 0.0))
    t = _ARCO.get(clave)
    if t is None:
        h = 0.5 * D
        ini = -2 * 2 * math.pi
        n = round(8 * 2 * math.pi / h)
        L = np.empty(n + 1)
        L[0] = 0.0
        for i in range(1, n + 1):
            a = ini + (i - 0.5) * h
            L[i] = L[i - 1] + math.hypot(rPerfil(q, a), rPerfilD(q, a)) * h
        i0 = round(-ini / h)
        t = (ini, h, L - L[i0])
        _ARCO[clave] = t
    ini, h, L = t
    x = (psi - ini) / h
    i = max(0, min(len(L) - 2, int(math.floor(x))))
    f = x - i
    return L[i] + (L[i + 1] - L[i]) * f


def puntoPerfil(q, psi, phi, aRef):
    s = sl(q)
    a = aRef + s * (phi - psi)
    r, rp = rPerfil(q, psi), rPerfilD(q, psi)
    ca, sa = math.cos(a), math.sin(a)
    return dict(x=-q['xt'] + r * ca, y=q['L2'] + r * sa, r=r, a=a,
                tx=rp * ca + s * r * sa, ty=rp * sa - s * r * ca)


def contacto(q, th, phi, aRef, cerca):
    """psi* donde el cable desde A' es tangente al perfil. Brazo en O y en el eje."""
    Apx, Apy = ApDe(q, th)

    def f(psi):
        P = puntoPerfil(q, psi, phi, aRef)
        return (P['x'] - Apx) * P['ty'] - (P['y'] - Apy) * P['tx']

    lo = hi = None
    for anchura in (0.06, 0.25, 1.0, 2.5):
        n = 12
        prev = cerca - anchura
        fp = f(prev)
        mejor = None
        for i in range(1, 2 * n + 1):
            x = cerca - anchura + anchura * i / n
            fx = f(x)
            if fp * fx <= 0:
                d = abs((prev + x) / 2 - cerca)
                if mejor is None or d < mejor[2]:
                    mejor = (prev, x, d)
            prev, fp = x, fx
        if mejor:
            lo, hi = mejor[0], mejor[1]
            break
    if lo is None:
        return None
    flo = f(lo)
    for _ in range(30):
        m = (lo + hi) / 2
        fm = f(m)
        if flo * fm <= 0:
            hi = m
        else:
            lo, flo = m, fm
    psi = (lo + hi) / 2
    P = puntoPerfil(q, psi, phi, aRef)
    dx, dy = P['x'] - Apx, P['y'] - Apy
    c = math.hypot(dx, dy)
    if not (c > 1e-9):
        return None
    ux, uy = dx / c, dy / c
    brazo = Apx * uy - Apy * ux
    p = abs((-q['xt'] - Apx) * uy - (q['L2'] - Apy) * ux)
    return dict(psi=psi, P=P, Ap=(Apx, Apy), u=(ux, uy), c=c, brazo=brazo, p=p, rc=P['r'])


def aRefDe(q):
    """Orientacion del tambor tal que en B el contacto cae en psi = 0."""
    t0 = tangente(q, q['th0'], rPerfil(q, 0))
    a0 = t0['aT'] if t0 else (0.0 if lado(q) > 0 else math.pi)
    Apx, Apy = ApDe(q, q['th0'])

    def f(aRef):
        P = puntoPerfil(q, 0, 0, aRef)
        return (P['x'] - Apx) * P['ty'] - (P['y'] - Apy) * P['tx']

    lo, hi = a0 - 0.7, a0 + 0.7
    flo = f(lo)
    if flo * f(hi) > 0:
        return a0
    for _ in range(40):
        m = (lo + hi) / 2
        fm = f(m)
        if flo * fm <= 0:
            hi = m
        else:
            lo, flo = m, fm
    return (lo + hi) / 2


def G(q, th, phi, aRef, cerca):
    """Cable total desde el anclaje: libre + arco enrollado. Se conserva al girar."""
    k = contacto(q, th, phi, aRef, cerca)
    return (k['c'] + arco(q, k['psi']), k) if k else None


_TRAY = {}


def trayectoria(q):
    """theta(phi) integrada paso a paso desde B. Cacheada por configuracion."""
    clave = json.dumps(q, sort_keys=True)
    if clave in _TRAY:
        return _TRAY[clave]
    dphi = 0.5 * D
    aRef = aRefDe(q)
    g0 = G(q, q['th0'], 0, aRef, 0)
    if not g0:
        t = dict(phi=[0.0], th=[q['th0']], psi=[0.0], aRef=aRef, G0=0.0)
        _TRAY[clave] = t
        return t
    G0 = g0[0]

    def avanza(signo):
        phi, th, psi = [], [], []
        t, ps = q['th0'], 0.0
        tope = 3 * 360 * 2 if signo > 0 else 360 * 2
        for i in range(1, tope + 1):
            p1 = signo * i * dphi
            lo, hi, gl, ok = t - 1.5, t + 1.5, None, False
            for k in range(6):
                gl = G(q, lo, p1, aRef, ps)
                gh = G(q, hi, p1, aRef, ps)
                if gl and gh and (gl[0] - G0) * (gh[0] - G0) <= 0:
                    ok = True
                    break
                lo -= 3 * (k + 1)
                hi += 3 * (k + 1)
                if lo < -90 or hi > 90:
                    break
            if not ok:
                break
            for _ in range(22):
                m = (lo + hi) / 2
                gm = G(q, m, p1, aRef, ps)
                if not gm:
                    break
                if (gl[0] - G0) * (gm[0] - G0) <= 0:
                    hi = m
                else:
                    lo, gl = m, gm
            t = (lo + hi) / 2
            gk = G(q, t, p1, aRef, ps)
            if not gk or t < -90 or t > 90:
                break
            ps = gk[1]['psi']
            phi.append(p1); th.append(t); psi.append(ps)
            if t >= 90 - 1e-6:
                break
        return phi, th, psi

    af, at_ = avanza(1), avanza(-1)
    tray = dict(phi=list(reversed(at_[0])) + [0.0] + af[0],
                th=list(reversed(at_[1])) + [q['th0']] + af[1],
                psi=list(reversed(at_[2])) + [0.0] + af[2],
                aRef=aRef, G0=G0)
    _TRAY[clave] = tray
    return tray


def thetaEn(q, phi):
    t = trayectoria(q)
    n = len(t['phi'])
    if not n or phi < t['phi'][0] - 1e-9 or phi > t['phi'][n - 1] + 1e-9:
        return None
    lo, hi = 0, n - 1
    while hi - lo > 1:
        m = (lo + hi) >> 1
        if t['phi'][m] <= phi:
            lo = m
        else:
            hi = m
    if hi == lo:
        return t['th'][lo], t['psi'][lo]
    f = max(0.0, min(1.0, (phi - t['phi'][lo]) / (t['phi'][hi] - t['phi'][lo])))
    return (t['th'][lo] + (t['th'][hi] - t['th'][lo]) * f,
            t['psi'][lo] + (t['psi'][hi] - t['psi'][lo]) * f)


def estado(q, avRev):
    """Estado con el ciclo avanzado avRev vueltas de MOTOR desde B."""
    phi = avRev * 2 * math.pi / q['red']
    r0_, s0_ = tambor(q, phi)
    tray = trayectoria(q)
    base = dict(avRev=avRev, phi=phi, r=r0_, s=s0_, g=0.0, c=0.0, aRef=tray['aRef'])
    e = thetaEn(q, phi)
    if e is None:
        return dict(base, fuera=True, a=None, th=None)
    th, psi0 = e
    a = incC(q) - th
    if not (-90 <= th <= 90):
        return dict(base, fuera=True, a=a, th=th)
    k = contacto(q, th, phi, tray['aRef'], psi0)
    if not k or k['brazo'] < 1e-6:
        return dict(base, fuera=True, a=a, th=th)
    base.update(tg=k['P'], Ap=k['Ap'], c=k['c'], psi=k['psi'], rc=k['rc'], r=k['p'],
                s=arco(q, k['psi']), g=tray['G0'] - k['c'])
    thr = th * D
    brazoPeso = (q['L3'] + q['L4']) * math.cos(thr) + q.get('e2', 0.0) * math.sin(thr)
    if brazoPeso <= 0:
        return dict(base, fuera=True, flojo=True, a=a, th=th, T=0.0, tau=0.0,
                    brazoCable=k['brazo'], brazoPeso=brazoPeso)
    T = q['m'] * q['g'] * brazoPeso / k['brazo']
    return dict(base, fuera=False, a=a, th=th, T=T, tau=T * k['p'] / 1000,
                brazoCable=k['brazo'], brazoPeso=brazoPeso)


def serie(q, avs):
    """T (N), tau (N.m), theta (deg) y brazo r (mm) en una lista de avances de motor."""
    n = len(avs)
    T = np.full(n, np.nan); tau = np.full(n, np.nan)
    th = np.full(n, np.nan); r = np.full(n, np.nan)
    for i, av in enumerate(avs):
        e = estado(q, float(av))
        if not e.get('fuera'):
            T[i], tau[i], th[i], r[i] = e['T'], e['tau'], e['th'], e['r']
    return T, tau, th, r


def recorridoMotor(q): return barrido(q) * q['red']


# ── Ensayos ───────────────────────────────────────────────────────────────
def load(p):
    consts, lines = {}, []
    for l in open(p, encoding='utf-8'):
        if l.startswith('#'):
            for tok in l.lstrip('#').split():
                if '=' in tok:
                    k, v = tok.split('=', 1)
                    consts[k] = v
        else:
            lines.append(l)
    R = list(csv.DictReader(lines))
    cols = ['t_rel_s', 'rep', 'pos_rev', 'lc1_raw', 'lc1_base', 'rpm',
            'par_x10', 'ref_rpm', 'lim_a', 'lim_b', 'work_us']
    d = {k: np.array([float(r[k]) for r in R]) for k in cols}
    d['fase'] = np.array([r['fase'] for r in R])
    d['name'] = os.path.basename(p)[:15]
    cal = float(consts.get('lc1_cuentas_por_N', 0) or 0)
    off = float(consts.get('lc1_cero_N', 0) or 0)
    d['cal'], d['off'] = cal, off
    d['F'] = d['lc1_base'] / cal + off if cal else None
    d['tau_ua'] = np.abs(d['par_x10']) / 10.0
    rel = d['pos_rev'] - d['pos_rev'][0]
    i = int(np.argmax(np.abs(rel)))
    d['dir'] = -1.0 if rel[i] < 0 else 1.0
    d['av'] = rel * d['dir']
    return d


def mov(d):
    """Muestras en regimen: consigna no nula y velocidad establecida en su sentido."""
    return ((d['ref_rpm'] != 0) & (np.sign(d['rpm']) == np.sign(d['ref_rpm']))
            & (np.abs(d['rpm']) >= 0.6 * np.abs(d['ref_rpm'])))


def r2(y, yh):
    m = ~np.isnan(yh) & ~np.isnan(y)
    if m.sum() < 5:
        return float('nan'), 0
    yy = y[m]
    ss = ((yy - yy.mean()) ** 2).sum()
    return (1 - ((yy - yh[m]) ** 2).sum() / ss if ss > 0 else float('nan')), int(m.sum())


# ── Validacion contra las cifras que documenta mecanismo.js ───────────────
if __name__ == '__main__':
    q = dict(carga_q(), tipo='car', r0=56.0, r1=28.18, barr=0.367, L2=330.0)
    print('Configuracion: %s  r0=%.2f r1=%.2f barr=%.3f L2=%.0f xt=%.0f red=%.1f'
          % (q['tipo'], q['r0'], q['r1'], q['barr'], q['L2'], q['xt'], q['red']))
    print('  incC = %.2f deg, |OC| = %.1f mm, punto muerto %.1f deg, theta libre %.1f deg'
          % (incC(q), oc(q), thMuerto(q), thLibre(q)))

    # 1. recorrido
    t = trayectoria(q)
    print('\n1. RECORRIDO')
    print('   theta de %.1f a %.1f deg en %.3f rev de tambor (%.3f rev de motor)'
          % (t['th'][0], t['th'][-1], t['phi'][-1] / (2 * math.pi),
             t['phi'][-1] / (2 * math.pi) * q['red']))
    rec = recorridoMotor(q)
    e_fin = estado(q, rec)
    print('   al final del barrido (%.3f rev de motor): theta = %s'
          % (rec, 'fuera' if e_fin.get('fuera') else '%.1f deg' % e_fin['th']))

    # 2. balance de energia
    print('\n2. BALANCE DE ENERGIA  (int tau dphi  vs  m g dh)')
    n = 400
    phis = np.linspace(0, barrido(q) * 2 * math.pi, n + 1)
    taus, ths = [], []
    for p in phis:
        e = estado(q, p * q['red'] / (2 * math.pi))
        taus.append(np.nan if e.get('fuera') else e['tau'])
        ths.append(np.nan if e.get('th') is None else e['th'])
    taus, ths = np.array(taus), np.array(ths)
    ok = ~np.isnan(taus)
    Wtau = float(np.trapezoid(taus[ok], phis[ok]))
    hP = lambda th: ((q['L3'] + q['L4']) * math.sin(th * D) - q.get('e2', 0) * math.cos(th * D))
    th_i, th_f = ths[ok][0], ths[ok][-1]
    Wgrav = q['m'] * q['g'] * (hP(th_f) - hP(th_i)) / 1000
    print('   int tau dphi = %.4f J     m g dh = %.4f J     error %.2f %%'
          % (Wtau, Wgrav, 100 * abs(Wtau - Wgrav) / Wgrav))

    # 3. cable
    print('\n3. CABLE')
    print('   arco del perfil en el barrido: %.1f mm' % arco(q, barrido(q) * 2 * math.pi))
    print('   tension de %.1f a %.1f N ; brazo del cable de %.1f a %.1f mm'
          % (estado(q, 0.05)['T'], estado(q, rec * 0.98)['T'],
             estado(q, 0.05)['r'], estado(q, rec * 0.98)['r']))

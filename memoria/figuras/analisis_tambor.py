"""Tambor: geometria confirmada, ganancia frente a un cilindro y perfiles candidatos.

Uso:  python analisis_tambor.py [--prefijo n14]
Necesita datosXX.json (generar_campana.py). Escribe nXX_tambor.pdf y tamborXX.json.
El contexto y las conclusiones estan en ANALISIS_TAMBOR.md. Tarda un par de minutos.

Geometria confirmada (14-10-2026): el tambor es SOLO una espiral lineal de 56 a 28,18 mm
en 180 grados; el cable arranca en el extremo de 56 mm y el radio baja al subir la barra.
No hay meseta de radio constante (la 'espiral con meseta' de generar_campana.py queda
superada) y la holgura del cable es, como mucho, de 10 grados de tambor.

Metrica: a IGUAL ALCANCE (mismo recorrido de barra en las mismas vueltas de motor) el
trabajo y el par medio son los mismos con cualquier tambor; lo unico que un perfil puede
mejorar es el par PICO. La referencia es el cilindro que hace ese recorrido en esas vueltas.
Equivale a decir cuanto alcance de mas da el perfil a igual par pico.
"""
import json, math, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter
import modelo_mecanismo as mm

OUT = os.path.dirname(os.path.abspath(__file__))
PREF = sys.argv[sys.argv.index('--prefijo') + 1] if '--prefijo' in sys.argv else 'n14'
plt.rcParams.update({'font.family': 'serif', 'font.size': 9, 'axes.grid': True,
                     'grid.alpha': .3, 'savefig.bbox': 'tight', 'legend.fontsize': 8,
                     'axes.titlesize': 9, 'lines.linewidth': 1.1})
coma = FuncFormatter(lambda x, p: ('%g' % round(x, 6)).replace('.', ','))
D = math.pi / 180


def save(fig, n):
    for ax in fig.axes:
        ax.xaxis.set_major_formatter(coma); ax.yaxis.set_major_formatter(coma)
    fig.tight_layout(); fig.savefig(f'{OUT}/{n}.pdf'); plt.close(fig)


# ── Perfil 'valle': bajada lo mas rapida posible y luego plano (o subida) ─
_tambor = mm.tambor


def tambor(q, phi):
    """Extension del modelo, tipo 'valle': espiral logaritmica de r0 a r1 con
    tan(angulo de presion) = barr, que es la bajada mas rapida que el cable sigue sin
    puentear el perfil; despues lineal hasta mes0 = (r_fin, phi_fin) y plano. Con
    r_fin = r1 queda 'bajada y plano'. La union no esta suavizada."""
    if q['tipo'] != 'valle':
        return _tambor(q, phi)
    R0, Rm, k = q['r0'], q['r1'], q['barr']
    Rf, fin = q['mes0']
    fd = math.log(R0 / Rm) / k
    if phi <= 0: return R0, R0 * phi
    if phi <= fd: return R0 * math.exp(-k * phi), R0 / k * (1 - math.exp(-k * phi))
    s1 = (R0 - Rm) / k
    if phi <= fin:
        u = phi - fd; g = (Rf - Rm) / (fin - fd)
        return Rm + g * u, s1 + Rm * u + g * u * u / 2
    return Rf, s1 + (Rm + Rf) / 2 * (fin - fd) + Rf * (phi - fin)


mm.tambor = tambor          # el resto del modelo lo busca por nombre en el modulo

N = json.load(open(f'{OUT}/datos{PREF[1:]}.json', encoding='utf-8'))
Q = dict(mm.carga_q(), tipo='car', mes0=0.0, **N['geom'])
R_MAX, R_MIN, GIRO = 56.0, 28.18, 0.5          # la pieza; GIRO en vueltas de tambor
HOLGURA_MAX = 10 / 360 * Q['red']              # 10 grados de tambor, en vueltas de motor
PRESION = 25                                    # grados; el limite de mecanismo.js
TH_B = -3.0                                     # final del recorrido del banco
AVS = np.arange(0.0, 4.0, 0.02)


def espiral(o=0.0):
    """La pieza, con el cable arrancando 'o' vueltas de tambor dentro de la espiral."""
    return dict(Q, r0=R_MAX - (R_MAX - R_MIN) * o / GIRO, r1=R_MIN, barr=GIRO - o)


def valle(r0, rm, rf=None, fin=125):
    rf = rm if rf is None else rf
    return dict(Q, tipo='valle', r0=float(r0), r1=float(rm), L1=float(r0),
                barr=math.tan(PRESION * D), mes0=(float(rf), fin * D))


def curva(q):
    _, tau, th, _ = mm.serie(q, AVS)
    ok = ~np.isnan(th)
    return AVS[ok], th[ok], tau[ok]


# ── Datos medidos: par del accionamiento y celula, como en generar_campana ─
DS = [mm.load(os.path.join(mm.RAIZ, 'cycles', n + '_ciclo.csv')) for n in N['ensayos']]
edges = np.arange(0, 1.90, 0.05); cent = (edges[:-1] + edges[1:]) / 2


def binned(col):
    U = [[] for _ in cent]; Dn = [[] for _ in cent]
    for d in DS:
        m0 = mm.mov(d); up = d['fase'] == 'hacia_A'
        v_ = d[col] / d['cal'] if col == 'lc1_base' else d[col]
        for msk, A in ((m0 & up, U), (m0 & ~up, Dn)):
            for a, v in zip(d['av'][msk], v_[msk]):
                i = np.searchsorted(edges, a) - 1
                if 0 <= i < len(cent): A[i].append(v)
    f = lambda A: np.array([np.mean(x) if len(x) >= 6 else np.nan for x in A])
    return f(U), f(Dn)


TU, TD = binned('tau_ua'); FU, FD = binned('lc1_base')
car = (cent >= N['carga_desde']) & ~np.isnan((TU + TD) / 2)
AV, TAUG, FG = cent[car], ((TU + TD) / 2)[car], ((FU + FD) / 2)[car]
assert abs(TAUG.mean() - N['taug_med']) < 1e-6, 'el promediado no coincide con la campana'
R = {}

# ── 1. Geometria: donde arranca el cable y cuanta holgura hay ────────────
def ajuste(tau):
    k = float(np.dot(TAUG, tau) / np.dot(tau, tau))
    return k, 1 - float(((TAUG - k * tau) ** 2).sum()) / float(((TAUG - TAUG.mean()) ** 2).sum())


mejor = None
for o in np.arange(0.0, 0.16, 0.01):
    a, th, tau = curva(espiral(o))
    for s in np.arange(0.0, HOLGURA_MAX + 1e-9, 0.01):
        x = AV - s
        if x.min() < a.min() or x.max() > a.max(): continue
        k, R2 = ajuste(np.interp(x, a, tau))
        if k > 0 and (mejor is None or R2 > mejor[0]):
            mejor = (R2, float(o), float(s), k)
R2_G, O, S, K = mejor
UA = 1 / K                                      # u. del accionamiento -> N.m en el tambor
PIEZA = espiral(O)
a_p, th_p, tau_p = curva(PIEZA)
R['geometria'] = dict(arranque_grados=360 * O, holgura_vueltas_motor=S,
                      holgura_grados_tambor=S / Q['red'] * 360, R2=R2_G, ua_a_Nm=UA)

# ── 2. Celula: la tension sigue subiendo; lo que baja es el brazo ────────
TAU_NM = TAUG * UA
BRAZO = 1000 * TAU_NM / FG                      # brazo efectivo medido (mm), sin modelo
GT = (AV - S) / Q['red'] * 360                  # grados de tambor desde el arranque
ip = int(np.argmax(TAU_NM))
R['medido'] = dict(
    tambor_grados=GT.tolist(), par_Nm=TAU_NM.tolist(), celula_N=FG.tolist(), brazo_mm=BRAZO.tolist(),
    pico=dict(tambor_grados=float(GT[ip]), par_Nm=float(TAU_NM[ip]), celula_N=float(FG[ip]),
              brazo_mm=float(BRAZO[ip])),
    inicio=dict(par_Nm=float(TAU_NM[0]), celula_N=float(FG[0]), brazo_mm=float(BRAZO[0])),
    final=dict(par_Nm=float(TAU_NM[-1]), celula_N=float(FG[-1]), brazo_mm=float(BRAZO[-1])),
    caida_final_pct=float(100 * (1 - TAU_NM[-1] / TAU_NM[ip])),
    celula_monotona=bool(np.all(np.diff(FG) > -0.3)),
    ancho_98_grados=float(np.ptp(GT[TAU_NM >= 0.98 * TAU_NM[ip]])))

# ── 3. Cilindros: la linea de referencia alcance-par ─────────────────────
CIL = {r: curva(dict(Q, tipo='cil', L1=float(r))) for r in np.arange(30.0, 81.0, 2.0)}
TH_A = max(c[1].min() for c in CIL.values()) + 0.5


def ventana(av, th, tau):
    a0, a1 = np.interp(TH_A, th, av), np.interp(TH_B, th, av)
    g = np.linspace(a0, a1, 300); t = np.interp(g, av, tau)
    return a1 - a0, float(t.max()), float(t.mean()), float(np.interp(g[t.argmax()], av, th))


LINEA = [(r, *ventana(*c)[:2]) for r, c in CIL.items()]
RC, NC, PC = map(np.array, zip(*LINEA))


def comparar(q):
    n, pk, mn, thp = ventana(*curva(q))
    pk_c = float(np.interp(n, NC[::-1], PC[::-1]))      # cilindro con las mismas vueltas
    n_c = float(np.interp(pk, PC, NC))                  # cilindro con el mismo pico
    return dict(vueltas=n, alcance_grados_por_vuelta=(TH_B - TH_A) / n, pico_Nm=pk, medio_Nm=mn,
                pico_en_brazo=thp, r_cilindro_igual_alcance=float(np.interp(n, NC[::-1], RC[::-1])),
                pico_frente_cilindro_pct=100 * (pk / pk_c - 1),
                alcance_frente_cilindro_pct=100 * (n_c / n - 1))


R['recorrido'] = dict(th_ini=TH_A, th_fin=TH_B)
ACTUAL = comparar(espiral(0.0))

# ── 4. Techo teorico: par constante, r = tau0/T (no construible) ─────────
hP = lambda th: ((Q['L3'] + Q['L4']) * math.sin(th * D) - Q['e2'] * math.cos(th * D)) / 1000
TAU0 = Q['m'] * Q['g'] * (hP(TH_B) - hP(TH_A)) / (ACTUAL['vueltas'] / Q['red'] * 2 * math.pi)
ths = np.arange(TH_A, TH_B + 0.01, 0.5); phi, rid, pid = 0.0, [], []
for i, th in enumerate(ths):
    r = 50.0
    for _ in range(30):
        T = mm.tensionDe(Q, th, r); r = min(TAU0 * 1000 / T, 1e4) if T else 1e4
    if i: phi += Q['m'] * Q['g'] * (hP(th) - hP(ths[i - 1])) / TAU0
    rid.append(r); pid.append(phi)
rid, pid = np.array(rid), np.array(pid)
pres = np.degrees(np.arctan(np.abs(np.gradient(rid, pid)) / rid))
pk_c_act = float(np.interp(ACTUAL['vueltas'], NC[::-1], PC[::-1]))
R['ideal'] = dict(tau0_Nm=TAU0, techo_pct=-100 * (1 - TAU0 / pk_c_act),
                  tabla=[dict(brazo=float(t), r_mm=float(rid[np.argmin(abs(ths - t))]),
                              tambor_grados=float(pid[np.argmin(abs(ths - t))] / D),
                              presion_grados=float(pres[np.argmin(abs(ths - t))]))
                         for t in (-75, -70, -60, -50, -40, -30, -20, -10, TH_B)])

# ── 5. Perfiles: el actual y los candidatos ──────────────────────────────
PERFILES = {
    'actual 56→28 (180°)':     espiral(0.0),
    '65→45 plano':             valle(65, 45),
    '70→48 plano':             valle(70, 48),
    '75→53 plano':             valle(75, 53),
    '70→45 plano (corto)':     valle(70, 45),
    '65→40→45 (valle)':        valle(65, 40, 45),
}
R['perfiles'] = {nom: (ACTUAL if nom.startswith('actual') else comparar(q)) for nom, q in PERFILES.items()}
R['cilindros'] = [dict(r=float(r), vueltas=float(n), pico_Nm=float(p)) for r, n, p in LINEA]
json.dump(R, open(f'{OUT}/tambor{PREF[1:]}.json', 'w'), indent=1, ensure_ascii=False)

# ── FIGURA ───────────────────────────────────────────────────────────────
fig, ax = plt.subplots(1, 2, figsize=(6.3, 3.0))
g = np.linspace(0, 130, 200)
ax[0].plot(GT, BRAZO, 'o', ms=3, color='#2ca02c', label='medido: par / célula')
for nom, q, c, e in (('actual 56→28 (180°)', PIEZA, '#222222', '-'),
                     ('65→45 plano', PERFILES['65→45 plano'], '#d62728', '--'),
                     ('70→48 plano', PERFILES['70→48 plano'], '#1f77b4', '-.')):
    ax[0].plot(g, [mm.tambor(q, x * D)[0] for x in g], e, color=c, label=nom)
ok = rid < 80
ax[0].plot(pid[ok] / D, rid[ok], ':', color='#9467bd', label='par constante (ideal)')
ax[0].set_xlabel('giro del tambor (°)'); ax[0].set_ylabel('radio / brazo (mm)')
ax[0].set_title('(a) perfil del tambor', fontsize=8.5); ax[0].legend(fontsize=6)
ax[1].plot((TH_B - TH_A) / NC, PC, '-', color='gray', label='cilindros')
for (nom, v), mk in zip(R['perfiles'].items(), 'osD^vP'):
    ax[1].plot(v['alcance_grados_por_vuelta'], v['pico_Nm'], mk, ms=5, label=nom)
ax[1].set_xlim(46, 60); ax[1].set_ylim(1.75, 2.45)
ax[1].set_xlabel('alcance (° de barra por vuelta de motor)'); ax[1].set_ylabel('par pico en el tambor (N·m)')
ax[1].set_title('(b) alcance frente a par pico', fontsize=8.5); ax[1].legend(fontsize=6)
save(fig, PREF + '_tambor')

# ── Resumen ──────────────────────────────────────────────────────────────
G, M = R['geometria'], R['medido']
print('GEOMETRIA  espiral 56 -> 28,18 mm en 180 deg; cable a %.0f deg del extremo de 56 mm,'
      ' holgura %.1f deg de tambor, R2 %.3f' % (G['arranque_grados'], G['holgura_grados_tambor'], G['R2']))
print('MEDIDO     pico %.2f N.m a %.0f deg de tambor (98%% del pico durante %.0f deg), cae %.0f%% al final'
      % (M['pico']['par_Nm'], M['pico']['tambor_grados'], M['ancho_98_grados'], M['caida_final_pct']))
print('           celula %.1f -> %.1f N (monotona: %s); brazo par/celula %.0f -> %.0f mm'
      % (M['inicio']['celula_N'], M['final']['celula_N'], M['celula_monotona'],
         M['inicio']['brazo_mm'], M['final']['brazo_mm']))
print('IDEAL      par constante %.2f N.m -> techo %.1f%% (no construible: ver tabla en el json)'
      % (TAU0, R['ideal']['techo_pct']))
print('\n%-22s %7s %9s %7s %8s | %10s %10s' % ('perfil (brazo %.0f -> %.0f deg)' % (TH_A, TH_B),
      'vueltas', 'alcance', 'pico', 'en', 'pico/cil', 'alc/cil'))
for nom, v in R['perfiles'].items():
    print('%-22s %7.2f %6.1f°/v %7.2f %7.0f° | %+9.1f%% %+9.1f%%'
          % (nom, v['vueltas'], v['alcance_grados_por_vuelta'], v['pico_Nm'], v['pico_en_brazo'],
             v['pico_frente_cilindro_pct'], v['alcance_frente_cilindro_pct']))

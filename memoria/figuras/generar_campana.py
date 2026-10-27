"""Analisis de una campana de ensayos con el modelo tangente.

Uso:  python generar_figuras07.py [--ultimos N] [--prefijo nXX] [--patron GLOB]
Por defecto analiza los 10 ensayos mas recientes de cycles/ y escribe n14_*.pdf y
datos14.json. Las figuras y cifras que cita la memoria salen de aqui.
"""
import glob, json, math, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter
import modelo_mecanismo as mm

OUT = os.path.dirname(os.path.abspath(__file__))
RAIZ = mm.RAIZ

def _arg(nombre, defecto=None):
    return sys.argv[sys.argv.index(nombre) + 1] if nombre in sys.argv else defecto

ULTIMOS = int(_arg('--ultimos', '10'))
PREF    = _arg('--prefijo', 'n14')
PATRON  = _arg('--patron')
plt.rcParams.update({'font.family': 'serif', 'font.size': 9, 'axes.grid': True,
                     'grid.alpha': .3, 'savefig.bbox': 'tight', 'legend.fontsize': 8,
                     'axes.titlesize': 9, 'lines.linewidth': 1.1})
C_UP, C_DN, C_M, C_F, C_E = '#d62728', '#1f77b4', '#222222', '#2ca02c', '#9467bd'
coma = FuncFormatter(lambda x, p: ('%g' % round(x, 6)).replace('.', ','))


def save(fig, n):
    for ax in fig.axes:
        ax.xaxis.set_major_formatter(coma); ax.yaxis.set_major_formatter(coma)
    fig.tight_layout(); fig.savefig(f'{OUT}/{n}.pdf'); plt.close(fig)


N = {}

# ── Geometria del banco tal y como esta montado ──────────────────────────
# El perfil fisico es la caracola medida sobre el CAD: espiral lineal de 56 a 28,18 mm
# en 0,367 vueltas de tambor. mecanismo.json guarda ahora un perfil de leva que es una
# propuesta de diseno, no la pieza montada, asi que aqui se fija la pieza real.
Q = dict(mm.carga_q(), tipo='car', r0=56.0, r1=28.18, barr=0.367, L2=330.0, mes0=0.0)
N['geom'] = {k: Q[k] for k in ('r0', 'r1', 'barr', 'L2', 'xt', 'L3', 'L4', 'e1', 'e2',
                               'm', 'th0', 'red')}

# ── Ensayos ──────────────────────────────────────────────────────────────
TODOS = sorted(glob.glob(os.path.join(RAIZ, 'cycles', PATRON or '*.csv')))
if not PATRON:
    TODOS = TODOS[-ULTIMOS:]          # los mas recientes: el nombre es la marca de tiempo
DS, DESCART = {}, []
for p in TODOS:
    d = mm.load(p)
    if d['av'].max() < 1.55:
        DESCART.append((d['name'], 'recorrido corto (%.2f rev)' % d['av'].max())); continue
    if (d['fase'] == 'volviendo_a_B').sum() < 50:
        DESCART.append((d['name'], 'sin bajada util')); continue
    DS[d['name']] = d
N['ensayos'] = sorted(DS)
N['descartados'] = DESCART
N['n_ensayos'] = len(DS)

# ── Promediado por tramos de avance, separando subida y bajada ───────────
edges = np.arange(0, 1.90, 0.05); cent = (edges[:-1] + edges[1:]) / 2


def binned(par, umbral=6):
    U = [[] for _ in cent]; Dn = [[] for _ in cent]
    for d in DS.values():
        m0 = mm.mov(d); up = d['fase'] == 'hacia_A'
        for msk, A in ((m0 & up, U), (m0 & ~up, Dn)):
            for a, v in zip(d['av'][msk], d[par][msk]):
                i = np.searchsorted(edges, a) - 1
                if 0 <= i < len(cent): A[i].append(v)
    f = lambda A: np.array([np.mean(x) if len(x) >= umbral else np.nan for x in A])
    return f(U), f(Dn)


TU, TD = binned('tau_ua')          # par del accionamiento, unidades internas
CU, CD = binned('lc1_base')        # celula, cuentas
TG = (TU + TD) / 2                 # gravedad: limpio de perdidas
TF = (TU - TD) / 2                 # friccion
CN = (CU + CD) / 2

CARGA = 0.60                       # antes de este avance el cable aun no tira
car = (cent >= CARGA) & ~np.isnan(TG)
AV, TAUG, TAUF, CEL = cent[car], TG[car], TF[car], CN[car]
N['carga_desde'] = CARGA
N['av_ini'], N['av_fin'] = float(AV[0]), float(AV[-1])
N['taug_ini'], N['taug_fin'] = float(TAUG[0]), float(TAUG[-1])
N['taug_var'] = float(100 * (TAUG[-1] / TAUG[0] - 1))
N['taug_med'] = float(TAUG.mean())
N['taug_riz'] = float(100 * (TAUG.max() - TAUG.min()) / TAUG.mean())
N['tauf_med'] = float(np.nanmean(TAUF))
N['tauf_pct'] = float(100 * np.nanmean(TAUF) / TAUG.mean())
N['tauf_rango'] = [float(np.nanmin(TAUF)), float(np.nanmax(TAUF))]


# ── Hipotesis de tambor ──────────────────────────────────────────────────
def metricas(mod):
    """Escala k > 0 unica (unidades de drive -> N.m) por minimos cuadrados."""
    m = ~np.isnan(mod)
    if m.sum() < 5: return float('nan'), float('nan'), float('nan')
    k = float(np.dot(TAUG[m], mod[m]) / np.dot(mod[m], mod[m]))
    ss = float(((TAUG[m] - TAUG[m].mean()) ** 2).sum())
    R2 = 1 - float(((TAUG[m] - k * mod[m]) ** 2).sum()) / ss
    return k, R2, float(np.corrcoef(TAUG[m], mod[m])[0, 1] ** 2)


def cilindro_equivalente(q):
    """Radio del cilindro que recorre lo mismo en las mismas vueltas de motor."""
    obj = mm.serie(q, [N['av_fin']])[2][0]
    lo, hi = 10.0, 80.0
    for _ in range(40):
        mid = (lo + hi) / 2
        th = mm.serie(dict(q, tipo='cil', L1=mid), [N['av_fin']])[2][0]
        if np.isnan(th) or th > obj: hi = mid
        else: lo = mid
    return (lo + hi) / 2


HIP = {}
HIP['espiral configurada'] = dict(Q)
HIP['espiral invertida'] = dict(Q, r0=Q['r1'], r1=Q['r0'])
RCIL = cilindro_equivalente(Q)
HIP['cilindro'] = dict(Q, tipo='cil', L1=RCIL)
N['r_cilindro'] = float(RCIL)

# meseta: el perfil real lleva un arco de radio constante antes de la espiral
mejor = None
for mes in np.arange(0.0, 0.26, 0.01):
    q2 = dict(Q, mes0=float(mes))
    _, tau, _, _ = mm.serie(q2, AV)
    k, R2, _ = metricas(tau)
    if np.isfinite(R2) and (mejor is None or R2 > mejor[1]): mejor = (float(mes), R2)
N['mes0'] = mejor[0]
HIP['espiral con meseta'] = dict(Q, mes0=mejor[0])

RES = {}
for nom, q in HIP.items():
    T, tau, th, r = mm.serie(q, AV)
    k, R2, rho2 = metricas(tau)
    RES[nom] = dict(T=T, tau=tau, th=th, r=r, k=k, R2=R2, rho2=rho2)
    N['R2_' + nom] = R2; N['rho2_' + nom] = rho2; N['k_' + nom] = k
    ok = ~np.isnan(tau)
    N['tauvar_' + nom] = float(100 * (tau[ok][-1] / tau[ok][0] - 1))
    N['rvar_' + nom] = float(100 * (r[ok][-1] / r[ok][0] - 1))
    N['riz_' + nom] = float(100 * (np.nanmax(tau) - np.nanmin(tau)) / np.nanmean(tau))
    N['taumax_' + nom] = float(np.nanmax(tau)); N['taumin_' + nom] = float(np.nanmin(tau))
    N['taumed_' + nom] = float(np.nanmean(tau))

BASE = RES['espiral configurada']
N['T_ini'], N['T_fin'] = float(BASE['T'][0]), float(BASE['T'][-1])
N['T_var'] = float(100 * (BASE['T'][-1] / BASE['T'][0] - 1))
N['th_ini'], N['th_fin'] = float(BASE['th'][0]), float(BASE['th'][-1])
N['r_ini'], N['r_fin'] = float(BASE['r'][0]), float(BASE['r'][-1])
MEJOR = max(RES, key=lambda n: RES[n]['R2'])
N['mejor_hipotesis'] = MEJOR
N['ua_a_Nm'] = float(1.0 / RES[MEJOR]['k'])

# par medido en N.m con la escala de la mejor hipotesis
N['taug_Nm_med'] = float(TAUG.mean() * N['ua_a_Nm'])
N['taug_Nm_max'] = float(TAUG.max() * N['ua_a_Nm'])
N['taug_Nm_min'] = float(TAUG.min() * N['ua_a_Nm'])
N['tauf_Nm'] = float(np.nanmean(TAUF) * N['ua_a_Nm'])
N['taum_motor'] = float(TAUG.max() * N['ua_a_Nm'] / Q['red'])
N['taum_motor_pct'] = float(100 * N['taum_motor'] / 1.27)

# radio efectivo medido: tau_g / T, normalizado
REF = TAUG / BASE['T']
N['ref_var'] = float(100 * (REF[-1] / REF[0] - 1))

# ── Celula de carga frente al modelo ─────────────────────────────────────
cal = float(np.mean([d['cal'] for d in DS.values()]))
N['cal_cuentas_por_N'] = cal
F_med = CEL / cal
ok = ~np.isnan(F_med) & ~np.isnan(BASE['T'])
esc = float(np.dot(BASE['T'][ok], F_med[ok]) / np.dot(F_med[ok], F_med[ok]))
N['celula_escala'] = esc
N['celula_R2'] = float(mm.r2(BASE['T'][ok], esc * F_med[ok])[0])
N['F_ini'], N['F_fin'] = float(F_med[0]), float(F_med[-1])
N['F_var'] = float(100 * (F_med[-1] / F_med[0] - 1))
# el par reconstruido con la celula y el brazo del modelo
tau_cel = esc * F_med * BASE['r'] / 1000
N['R2_celula_x_brazo'] = float(mm.r2(TAUG, metricas(tau_cel)[0] * tau_cel)[0])

# acuerdo de la celula ensayo a ensayo: entre medias hubo un cambio en el montaje
# (se quito un pellizco del cable), asi que promediarlos todos juntos lo oculta.
N['celula_por_ensayo'] = {}
for nom, d in sorted(DS.items()):
    # SOLO la subida y ORDENADO POR AVANCE: el ciclo es B->A->B, asi que la ultima
    # muestra en el tiempo vuelve a estar en B y comparar primera con ultima en orden
    # temporal daba una tendencia falsa (plana).
    m = mm.mov(d) & (d['av'] >= CARGA) & (d['fase'] == 'hacia_A')
    if m.sum() < 20: continue
    orden = np.argsort(d['av'][m])
    av_ = d['av'][m][orden]; F_ = d['lc1_base'][m][orden] / d['cal']
    Tm_, _, _, _ = mm.serie(Q, av_)
    o = ~np.isnan(Tm_)
    if o.sum() < 20: continue
    e_ = float(np.dot(Tm_[o], F_[o]) / np.dot(F_[o], F_[o]))
    N['celula_por_ensayo'][nom] = dict(
        escala=e_, R2=float(mm.r2(Tm_[o], e_ * F_[o])[0]),
        F_ini=float(np.mean(F_[o][:5])), F_fin=float(np.mean(F_[o][-5:])),
        var=float(100 * (np.mean(F_[o][-5:]) / np.mean(F_[o][:5]) - 1)))

# ── Temporizacion ────────────────────────────────────────────────────────
dts, work = [], []
for d in DS.values():
    dt = np.diff(d['t_rel_s']) * 1000
    dts.append(dt[(dt > 0) & (dt < 200)]); work.append(d['work_us'] / 1000)
dts = np.concatenate(dts); work = np.concatenate(work)
N.update(dt_media=float(dts.mean()), dt_sd=float(dts.std()),
         dt_frac50=float((np.abs(dts - 50) < 0.5).mean()),
         work_media=float(work.mean()), work_max=float(work.max()))
N['rep_avmax'] = [float(d['av'].max()) for d in DS.values()]
N['rep_sd'] = float(np.std(N['rep_avmax'], ddof=1))

# ── FIGURAS ──────────────────────────────────────────────────────────────
d = DS[sorted(DS)[-1]]
ts = d['t_rel_s']; sube = d['fase'] == 'hacia_A'
Tm, taum, thm, rm = mm.serie(Q, d['av'])
fig, ax = plt.subplots(4, 1, figsize=(6.3, 7.4), sharex=True)
tc = ts[(d['av'] >= CARGA) & sube]
for a in ax:
    if len(tc): a.axvspan(ts[0], tc[0], color='gray', alpha=.16, lw=0)
ax[0].plot(ts, d['av'], color=C_M); ax[0].axhline(CARGA, ls=':', color='gray')
ax[0].set_ylabel('avance desde B\n(rev de motor)')
a0 = ax[0].twinx(); a0.plot(ts, thm, '--', color=C_E); a0.set_ylabel(r'$\theta$ del modelo (°)'); a0.grid(False)
ax[1].step(ts, d['ref_rpm'], where='post', color='gray', label='consigna')
ax[1].plot(ts, d['rpm'], color=C_M, label='medida')
ax[1].set_ylabel('velocidad (rpm)'); ax[1].legend(loc='lower right', ncol=2)
ax[2].plot(ts, d['lc1_base'] / cal, color=C_F, label='célula (N)')
a2 = ax[2].twinx(); a2.plot(ts, Tm, '--', color=C_DN); a2.set_ylabel('T del modelo (N)'); a2.grid(False)
ax[2].set_ylabel('célula (N)'); ax[2].legend(loc='center left')
ax[3].plot(ts, d['tau_ua'], color=C_M); ax[3].set_ylabel('|par| (u. accionam.)')
ax[3].set_xlabel('tiempo (s)')
ax[0].set_title('Ensayo %s · ciclo B→A→B (gris: tramo sin carga)' % d['name'][9:].replace('-', ':'))
save(fig, PREF + '_ensayo_tipo')

fig, ax = plt.subplots(3, 1, figsize=(6.3, 6.6), sharex=True)
for a in ax: a.axvspan(0, CARGA, color='gray', alpha=.16, lw=0)
sel = ~np.isnan(TG)
ax[0].plot(cent[sel], CU[sel] / cal, '-o', ms=3, color=C_UP, label='subida B→A')
ax[0].plot(cent[sel], CD[sel] / cal, '-o', ms=3, color=C_DN, label='bajada A→B')
Tall = mm.serie(Q, cent)[0]
ax[0].plot(cent, esc and Tall / esc, '--', lw=2, color=C_M, label='T del modelo / escala')
ax[0].set_ylabel('célula (N)'); ax[0].legend(loc='upper left', fontsize=7)
ax[1].plot(cent[sel], TU[sel], '-o', ms=3, color=C_UP, label='subida')
ax[1].plot(cent[sel], TD[sel], '-o', ms=3, color=C_DN, label='bajada')
ax[1].plot(cent[sel], TG[sel], '-', lw=2, color=C_M, label=r'$\tau_g$')
ax[1].plot(cent[sel], TF[sel], '--', color=C_F, label=r'$\tau_f$')
ax[1].set_ylabel('|par| (u. accionam.)'); ax[1].legend(ncol=2, loc='upper left', fontsize=7)
ax[2].plot(AV, REF / REF.max(), '-o', ms=3, color=C_M, label=r'medido $\propto\tau_g/T$')
for nom, est, col in (('espiral configurada', '--', C_UP), ('espiral con meseta', '-', C_E),
                      ('espiral invertida', ':', C_DN)):
    r = RES[nom]['r']; ax[2].plot(AV, r / np.nanmax(r), est, color=col, label=nom)
ax[2].set_ylabel('brazo normalizado'); ax[2].set_xlabel('avance desde B (rev de motor)')
ax[2].legend(fontsize=7)
save(fig, PREF + '_descomposicion')

fig, ax = plt.subplots(1, 2, figsize=(6.3, 2.8))
ax[0].plot(AV, TAUG * N['ua_a_Nm'], '-o', ms=3, color=C_M, label='medido')
for nom, est, col in (('espiral configurada', '-', C_E), ('espiral con meseta', '-', C_UP),
                      ('cilindro', '--', C_DN), ('espiral invertida', ':', '#8c564b')):
    ax[0].plot(AV, RES[nom]['tau'], est, color=col,
               label='%s (R²=%.2f)' % (nom.replace('espiral ', 'esp. '), RES[nom]['R2']))
ax[0].set_xlabel('avance desde B (rev de motor)'); ax[0].set_ylabel(r'$\tau$ en el tambor (N·m)')
ax[0].legend(fontsize=6.2)
ax[0].set_title('(a) par: medido frente a hipótesis', fontsize=8.5)
ax[1].plot(AV, F_med * esc, '-o', ms=3, color=C_F, label='célula × escala')
ax[1].plot(AV, BASE['T'], '--', lw=2, color=C_DN, label='T del modelo')
ax[1].set_xlabel('avance desde B (rev de motor)'); ax[1].set_ylabel('tensión del cable (N)')
ax[1].legend(fontsize=7); ax[1].set_title('(b) tensión: célula frente a modelo', fontsize=8.5)
save(fig, PREF + '_modelo')

DATOS = f'{OUT}/datos{PREF[1:]}.json'
json.dump(N, open(DATOS, 'w'), indent=1, ensure_ascii=False, default=float)

print('  ensayos usados: %d  %s' % (len(DS), ', '.join(sorted(DS))))
for n_, por in DESCART: print('    descartado %s: %s' % (n_, por))
print('  recorrido: %.2f a %.2f rev de motor; theta del modelo %.1f -> %.1f deg'
      % (N['av_ini'], N['av_fin'], N['th_ini'], N['th_fin']))
print('  tension del modelo: %.1f -> %.1f N (%+.0f %%); brazo %.1f -> %.1f mm'
      % (N['T_ini'], N['T_fin'], N['T_var'], N['r_ini'], N['r_fin']))
print('  PAR MEDIDO: %.1f -> %.1f ud (%+.1f %%), rizado %.1f %%, media %.1f ud'
      % (N['taug_ini'], N['taug_fin'], N['taug_var'], N['taug_riz'], N['taug_med']))
print('  friccion: %.1f ud de media (%.0f %% del par), rango %.1f-%.1f'
      % (N['tauf_med'], N['tauf_pct'], *N['tauf_rango']))
print('  mejor hipotesis: %s; escala 1 ud = %.4f N.m' % (N['mejor_hipotesis'], N['ua_a_Nm']))
print('  meseta ajustada: %.2f rev de tambor' % N['mes0'])
print('\n  %-22s %8s %8s %9s %9s %8s' % ('hipotesis', 'R2', 'rho2', 'var tau', 'var r', 'rizado'))
for nom in ('espiral configurada', 'espiral con meseta', 'cilindro', 'espiral invertida'):
    print('  %-22s %8.3f %8.3f %+8.1f %% %+8.1f %% %7.1f %%'
          % (nom, N['R2_' + nom], N['rho2_' + nom], N['tauvar_' + nom],
             N['rvar_' + nom], N['riz_' + nom]))
print('  MEDIDO                                   %+8.1f %% %+8.1f %% %7.1f %%'
      % (N['taug_var'], N['ref_var'], N['taug_riz']))
print('\n  celula: escala %.3f sobre la calibracion provisional (%.1f cuentas/N), R2 %.3f'
      % (N['celula_escala'], N['cal_cuentas_por_N'], N['celula_R2']))
print('  celula x brazo del modelo reproduce el par del drive con R2 %.3f' % N['R2_celula_x_brazo'])
print('  par medido: media %.2f N.m, pico %.2f N.m; friccion %.2f N.m; en el motor %.2f N.m (%.0f %% del nominal)'
      % (N['taug_Nm_med'], N['taug_Nm_max'], N['tauf_Nm'], N['taum_motor'], N['taum_motor_pct']))
for nom in ('espiral configurada', 'espiral con meseta', 'cilindro', 'espiral invertida'):
    print('    %-22s pico %.2f N.m, medio %.2f' % (nom, N['taumax_' + nom], N['taumed_' + nom]))
print('\n  celula ensayo a ensayo (T del modelo sube %+.0f %%):' % N['T_var'])
for nom, v in N['celula_por_ensayo'].items():
    print('    %s  escala %5.2f  R2 %7.3f  F %5.1f -> %5.1f N (%+.0f %%)'
          % (nom[9:], v['escala'], v['R2'], v['F_ini'], v['F_fin'], v['var']))
print('  periodo %.2f +- %.2f ms (%.1f %% dentro de 0,5 ms); trabajo %.2f ms'
      % (N['dt_media'], N['dt_sd'], 100 * N['dt_frac50'], N['work_media']))


# ── Comparacion de diseno A IGUAL RECORRIDO ──────────────────────────────
# Cada tambor se escala para que la barra llegue al MISMO theta final en las mismas
# vueltas de motor. Sin esa normalizacion se comparan recorridos distintos y el
# tambor mas grande parece peor solo por mover mas carga por vuelta.
TH_OBJ = N['th_fin']
AVD = np.linspace(N['av_ini'], N['av_fin'], 60)

def escala_para_llegar(hacer):
    lo, hi = 0.4, 2.5
    for _ in range(36):
        mid = (lo + hi) / 2
        th = mm.serie(hacer(mid), [N['av_fin']])[2][0]
        if np.isnan(th) or th > TH_OBJ: hi = mid
        else: lo = mid
    return (lo + hi) / 2

CAND = {
    'cilindro':              lambda s: dict(Q, tipo='cil', L1=44.2 * s),
    'espiral 56→28 (pieza)': lambda s: dict(Q, r0=56 * s, r1=28.18 * s),
    'espiral con meseta':    lambda s: dict(Q, r0=56 * s, r1=28.18 * s, mes0=N['mes0']),
    'espiral suave 52→44':   lambda s: dict(Q, r0=52 * s, r1=44 * s),
    'espiral invertida':     lambda s: dict(Q, r0=28.18 * s, r1=56 * s),
}
N['diseno'] = {}
print('\n  DISENO A IGUAL RECORRIDO (barra hasta %.1f deg en %.2f rev de motor):' % (TH_OBJ, N['av_fin']))
print('    %-24s escala  r ini→fin (mm)   tau pico  medio  rizado' % 'tambor')
for nom, hacer in CAND.items():
    s = escala_para_llegar(hacer)
    q = hacer(s)
    T, tau, th, r = mm.serie(q, AVD)
    ok = ~np.isnan(tau)
    if ok.sum() < 5: continue
    d_ = dict(escala=float(s), r_ini=float(r[ok][0]), r_fin=float(r[ok][-1]),
              tau_max=float(np.nanmax(tau)), tau_med=float(np.nanmean(tau)),
              riz=float(100 * (np.nanmax(tau) - np.nanmin(tau)) / np.nanmean(tau)))
    N['diseno'][nom] = d_
    print('    %-24s %5.2f   %5.1f → %5.1f      %5.2f   %5.2f   %5.1f %%'
          % (nom, s, d_['r_ini'], d_['r_fin'], d_['tau_max'], d_['tau_med'], d_['riz']))
json.dump(N, open(DATOS, 'w'), indent=1, ensure_ascii=False, default=float)

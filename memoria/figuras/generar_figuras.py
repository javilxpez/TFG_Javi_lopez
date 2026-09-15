"""Figuras y cifras de la memoria a partir de los ensayos reales (cycles/*.csv),
del modelo analítico del mecanismo y del historial de git."""
import csv, glob, os, json, math, subprocess, datetime, collections
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter

ROOT = '/home/uni/Documents/TFG_Javi_lopez'
OUT = ROOT + '/memoria/figuras'
os.makedirs(OUT, exist_ok=True)

plt.rcParams.update({
    'font.family': 'serif', 'font.size': 9, 'axes.grid': True, 'grid.alpha': 0.3,
    'savefig.bbox': 'tight', 'legend.fontsize': 8, 'axes.titlesize': 9,
    'lines.linewidth': 1.1,
})
C_B, C_A, C_M, C_F = '#1f77b4', '#d62728', '#222222', '#2ca02c'
coma = FuncFormatter(lambda x, p: ('%g' % round(x, 6)).replace('.', ','))


def fmt(fig):
    for ax in fig.axes:
        ax.xaxis.set_major_formatter(coma)
        ax.yaxis.set_major_formatter(coma)


def save(fig, name):
    fmt(fig)
    fig.tight_layout()
    fig.savefig(f'{OUT}/{name}.pdf')
    plt.close(fig)


# Calibración provisional de LC1 que figura en la interfaz (index.html / web_monitor.py)
CAL, OFF = 14.41, 51.78   # cuentas/N y fuerza (N) en el punto de tara

COLS = ['t_ms', 't_rel_s', 'rep', 'pos_rev', 'lc1_raw', 'lc1_base', 'rpm', 'par_x10',
        'ref_rpm', 'lim_a', 'lim_b', 'estado', 'work_us']


def load(p):
    lines = [l for l in open(p, encoding='utf-8') if not l.startswith('#')]
    R = list(csv.DictReader(lines))
    d = {k: np.array([float(r[k]) for r in R]) for k in COLS}
    d['fase'] = np.array([r['fase'] for r in R])
    d['name'] = os.path.basename(p).replace('_ciclo.csv', '')
    d['n'] = len(R)
    if len(R):
        d['vref'] = int(np.max(np.abs(d['ref_rpm'])))
        d['zero'] = float(np.median(d['lc1_raw'] - d['lc1_base']))
        d['F'] = d['lc1_base'] / CAL + OFF           # N (calibración provisional)
        d['Fraw'] = (d['lc1_raw'] - d['zero']) / CAL + OFF
        d['tau'] = np.abs(d['par_x10']) / 10.0       # unidades del accionamiento
    return d


tests = [load(p) for p in sorted(glob.glob(ROOT + '/cycles/*.csv'))]
tests = [t for t in tests if t['n'] > 30]
T30 = [t for t in tests if t['vref'] in (30, 31)]
num = {}
num['ensayos_total'] = len(glob.glob(ROOT + '/cycles/*.csv'))
num['ensayos_validos'] = len(tests)
num['ensayos_30rpm'] = len(T30)
num['muestras_total'] = int(sum(t['n'] for t in tests))

# ───────────────────────── 1. Ensayo tipo ─────────────────────────
t = [x for x in tests if x['name'] == '20260915-181346'][0]
fig, ax = plt.subplots(4, 1, figsize=(6.3, 7.2), sharex=True)
ts = t['t_rel_s']
for a in ax:
    for key, col in (('lim_a', C_A), ('lim_b', C_B)):
        on = t[key] > 0.5
        i = 0
        while i < len(on):
            if on[i]:
                j = i
                while j + 1 < len(on) and on[j + 1]:
                    j += 1
                a.axvspan(ts[i] - 0.025, ts[j] + 0.025, color=col, alpha=0.12, lw=0)
                i = j + 1
            else:
                i += 1
ax[0].plot(ts, t['pos_rev'], color=C_M)
ax[0].axhline(1.0, ls=':', color='gray')
ax[0].set_ylabel('posición\n(rev motor)')
ax[1].step(ts, t['ref_rpm'], where='post', color='gray', label='consigna')
ax[1].plot(ts, t['rpm'], color=C_M, label='medida')
ax[1].set_ylabel('velocidad\n(rpm)')
ax[1].legend(loc='lower left', ncol=2)
ax[2].step(ts, t['lc1_raw'], where='post', color='gray', label='bruta (cabecera de lote)')
ax[2].plot(ts, t['lc1_base'] + t['zero'], color=C_F, label='filtrada (media móvil 8)')
ax[2].set_ylabel('LC1\n(cuentas)')
ax[2].legend(loc='upper left')
a2 = ax[2].twinx()
lo, hi = ax[2].get_ylim()
a2.set_ylim((lo - t['zero']) / CAL + OFF, (hi - t['zero']) / CAL + OFF)
a2.set_ylabel('F (N, prov.)')
a2.grid(False)
ax[3].plot(ts, t['tau'], color=C_M)
ax[3].set_ylabel('|par|\n(u. accionam.)')
ax[3].set_xlabel('tiempo (s)')
ax[0].set_title('Ensayo 2026-09-15 18:13:46 · 30 rpm · B = final de carrera '
                '(sombreado: FC A rojo, FC B azul)')
save(fig, 'ensayo_tipo')

# ───────────────────────── 2. Temporización ─────────────────────────
dts = np.concatenate([np.diff(x['t_ms']) for x in tests])
work = np.concatenate([x['work_us'] for x in tests]) / 1000.0
fig, ax = plt.subplots(1, 2, figsize=(6.3, 2.4))
vals, cnts = np.unique(dts, return_counts=True)
ax[0].bar(vals, cnts, width=0.6, color=C_M)
ax[0].set_yscale('log')
ax[0].set_xlabel('periodo entre muestras (ms)')
ax[0].set_ylabel('nº de muestras')
ax[1].hist(work, bins=40, color=C_M)
ax[1].set_xlabel('tiempo de trabajo del lazo (ms)')
ax[1].set_ylabel('nº de muestras')
save(fig, 'temporizacion')
num['dt_media_ms'] = float(dts.mean())
num['dt_sd_ms'] = float(dts.std())
num['dt_min_ms'] = int(dts.min())
num['dt_max_ms'] = int(dts.max())
num['dt_frac_50'] = float((dts == 50).mean())
num['work_media_ms'] = float(work.mean())
num['work_max_ms'] = float(work.max())
num['work_min_ms'] = float(work.min())
num['frec_muestreo_hz'] = 1000.0 / float(dts.mean())

# Periodo real de refresco de la lectura bruta (cambios del valor de cabecera)
chg = []
for x in tests:
    idx = np.where(np.diff(x['lc1_raw']) != 0)[0]
    if len(idx) > 2:
        chg.extend(np.diff(x['t_ms'][idx + 1]))
chg = np.array(chg)
num['raw_refresco_mediana_ms'] = float(np.median(chg))

# ───────────────────────── 3. Repetibilidad ─────────────────────────
rows = []
for x in T30:
    for r in sorted(set(x['rep'])):
        m = x['rep'] == r
        rows.append({'ensayo': x['name'], 'rep': int(r), 'pos_B': float(x['pos_rev'][m].max()),
                     'pos_fin': float(x['pos_rev'][m][-1]), 'pos_ini': float(x['pos_rev'][m][0]),
                     'dur': float(x['t_rel_s'][m][-1] - x['t_rel_s'][m][0])})
pB = np.array([r['pos_B'] for r in rows])
pF = np.array([r['pos_fin'] for r in rows])
fig, ax = plt.subplots(1, 2, figsize=(6.3, 2.5))
k = np.arange(len(rows)) + 1
ax[0].plot(k, pB, 'o', color=C_B)
ax[0].axhline(pB.mean(), color='gray', ls='--')
ax[0].set_xlabel('ciclo')
ax[0].set_ylabel('posición en B (rev)')
ax[1].plot(k, pF * 1000, 's', color=C_A)
ax[1].axhspan(-20, 20, color='gray', alpha=0.15, lw=0)
ax[1].set_xlabel('ciclo')
ax[1].set_ylabel('posición final (mrev)')
save(fig, 'repetibilidad')
num['rep_ciclos'] = len(rows)
num['rep_posB_media'] = float(pB.mean())
num['rep_posB_sd'] = float(pB.std(ddof=1))
num['rep_posB_min'] = float(pB.min())
num['rep_posB_max'] = float(pB.max())
num['rep_posfin_media_mrev'] = float(pF.mean() * 1000)
num['rep_posfin_sd_mrev'] = float(pF.std(ddof=1) * 1000)
num['rep_posfin_absmax_mrev'] = float(np.abs(pF).max() * 1000)
num['rep_filas'] = rows


# ───────────────────────── 4-6. Fuerza y par frente a posición ─────────────────────────
def moving(x):
    ref, rpm = x['ref_rpm'], x['rpm']
    return (ref != 0) & (np.sign(rpm) == np.sign(ref)) & (np.abs(rpm) >= 0.6 * np.abs(ref))


fig, ax = plt.subplots(2, 1, figsize=(6.3, 5.0), sharex=True)
for x in T30:
    mv = moving(x)
    b = mv & (x['ref_rpm'] > 0)
    a = mv & (x['ref_rpm'] < 0)
    ax[0].plot(x['pos_rev'][b], x['F'][b], '.', ms=2, color=C_B, alpha=0.5)
    ax[0].plot(x['pos_rev'][a], x['F'][a], '.', ms=2, color=C_A, alpha=0.5)
    ax[1].plot(x['pos_rev'][b], x['tau'][b], '.', ms=2, color=C_B, alpha=0.5)
    ax[1].plot(x['pos_rev'][a], x['tau'][a], '.', ms=2, color=C_A, alpha=0.5)
ax[0].plot([], [], 'o', color=C_B, label='hacia B (baja la pesa)')
ax[0].plot([], [], 'o', color=C_A, label='hacia A (sube la pesa)')
ax[0].legend(loc='upper left')
ax[0].set_ylabel('tensión del cable\nF (N, cal. provisional)')
ax[1].set_ylabel('|par| del motor\n(u. del accionamiento)')
ax[1].set_xlabel('posición del eje del motor (rev)')
save(fig, 'fuerza_par_posicion')

edges = np.arange(0.0, 1.85, 0.05)
cent = (edges[:-1] + edges[1:]) / 2


def binned(key, sign):
    out = np.full(len(cent), np.nan)
    cnt = np.zeros(len(cent))
    acc = [[] for _ in cent]
    for x in T30:
        mv = moving(x) & (np.sign(x['ref_rpm']) == sign)
        for p, v in zip(x['pos_rev'][mv], x[key][mv]):
            i = np.searchsorted(edges, p) - 1
            if 0 <= i < len(cent):
                acc[i].append(v)
    for i, l in enumerate(acc):
        cnt[i] = len(l)
        if len(l) >= 8:
            out[i] = np.mean(l)
    return out, cnt


FB, nB = binned('F', +1)
FA, nA = binned('F', -1)
TB, _ = binned('tau', +1)
TA, _ = binned('tau', -1)
ok = ~np.isnan(FB) & ~np.isnan(FA) & ~np.isnan(TB) & ~np.isnan(TA)
Fg = (FA + FB) / 2
Tg = (TA + TB) / 2
Tf = (TA - TB) / 2
Fh = (FA - FB) / 2
rrel = Tg / Fg
rrel_n = rrel / np.nanmax(rrel[ok])
fig, ax = plt.subplots(3, 1, figsize=(6.3, 6.4), sharex=True)
ax[0].plot(cent[ok], FB[ok], '-o', ms=3, color=C_B, label='hacia B')
ax[0].plot(cent[ok], FA[ok], '-o', ms=3, color=C_A, label='hacia A')
ax[0].plot(cent[ok], Fg[ok], '-', lw=2, color=C_M, label='media (sin retardo de filtro)')
ax[0].set_ylabel('F (N, prov.)')
ax[0].legend(loc='upper left')
ax[1].plot(cent[ok], TB[ok], '-o', ms=3, color=C_B, label='hacia B')
ax[1].plot(cent[ok], TA[ok], '-o', ms=3, color=C_A, label='hacia A')
ax[1].plot(cent[ok], Tg[ok], '-', lw=2, color=C_M, label=r'$\tau_g$ (gravitatorio)')
ax[1].plot(cent[ok], Tf[ok], '--', color=C_F, label=r'$\tau_f$ (pérdidas)')
ax[1].set_ylabel('|par| (u. accionam.)')
ax[1].legend(loc='center left', ncol=2)
ax[2].plot(cent[ok], rrel_n[ok], '-o', ms=3, color=C_M)
ax[2].set_ylabel(r'$r_{ef}$ normalizado' + '\n' + r'$\propto \tau_g / F$')
ax[2].set_xlabel('posición del eje del motor (rev)')
save(fig, 'descomposicion_par')
sel = ok
num['bins_validos'] = int(sel.sum())
num['bins_pos_min'] = float(cent[sel].min())
num['bins_pos_max'] = float(cent[sel].max())
num['Fg_min'] = float(np.nanmin(Fg[sel]))
num['Fg_max'] = float(np.nanmax(Fg[sel]))
num['Fg_var_pct'] = float(100 * (num['Fg_max'] - num['Fg_min']) / np.nanmean(Fg[sel]))
num['Tg_min'] = float(np.nanmin(Tg[sel]))
num['Tg_max'] = float(np.nanmax(Tg[sel]))
num['Tg_var_pct'] = float(100 * (num['Tg_max'] - num['Tg_min']) / np.nanmean(Tg[sel]))
num['Tf_media'] = float(np.nanmean(Tf[sel]))
num['Tf_sobre_Tg_pct'] = float(100 * np.nanmean(Tf[sel]) / np.nanmean(Tg[sel]))
num['Fh_media_N'] = float(np.nanmean(Fh[sel]))
num['rrel_min_norm'] = float(np.nanmin(rrel_n[sel]))
num['tabla_bins'] = [{'pos': round(float(c), 3), 'F_B': round(float(a), 2), 'F_A': round(float(b), 2),
                      'tau_B': round(float(cc), 1), 'tau_A': round(float(dd), 1)}
                     for c, a, b, cc, dd, o in zip(cent, FB, FA, TB, TA, ok) if o]
# Variación en el tramo central (sin los extremos de aceleración y del tope B)
cen = sel & (cent >= 0.3) & (cent <= 1.2)
num['central_Fg_var_pct'] = float(100 * (np.nanmax(Fg[cen]) - np.nanmin(Fg[cen])) / np.nanmean(Fg[cen]))
num['central_Tg_var_pct'] = float(100 * (np.nanmax(Tg[cen]) - np.nanmin(Tg[cen])) / np.nanmean(Tg[cen]))
num['central_Fg_min'] = float(np.nanmin(Fg[cen]))
num['central_Fg_max'] = float(np.nanmax(Fg[cen]))
num['central_Tg_min'] = float(np.nanmin(Tg[cen]))
num['central_Tg_max'] = float(np.nanmax(Tg[cen]))

# ───────────────────────── 7. Efecto de la velocidad ─────────────────────────
fig, ax = plt.subplots(1, 1, figsize=(6.3, 2.8))
cols = {20: '#9467bd', 30: C_M, 40: '#ff7f0e'}
names = {20: '20260914-183121', 30: '20260915-181346', 40: '20260914-194105'}
vel = {}
for v, nm in names.items():
    x = [y for y in tests if y['name'] == nm][0]
    mv = moving(x)
    for s, ls in ((+1, '-'), (-1, '--')):
        m = mv & (np.sign(x['ref_rpm']) == s)
        o = np.argsort(x['pos_rev'][m])
        ax.plot(x['pos_rev'][m][o], x['tau'][m][o], ls, color=cols[v],
                label=f'{v} rpm ' + ('hacia B' if s > 0 else 'hacia A'))
    vel[v] = {'tauA_medio': float(x['tau'][mv & (x['ref_rpm'] < 0)].mean()),
              'tauB_medio': float(x['tau'][mv & (x['ref_rpm'] > 0)].mean()),
              'posB': float(x['pos_rev'].max()), 'dur_s': float(x['t_rel_s'][-1])}
ax.set_xlabel('posición del eje del motor (rev)')
ax.set_ylabel('|par| (u. accionam.)')
ax.legend(ncol=3, fontsize=7)
save(fig, 'efecto_velocidad')
num['velocidades'] = vel

# ───────────────────────── 8. Modelo analítico ─────────────────────────
L2, L3, L4, m, g = 290.0, 95.0, 110.0, 2.0, 9.81     # mm, kg, m/s² (valores de la interfaz)
K = m * g * (L3 + L4) / (L2 * L3)                    # N/mm
cmin, cmax = abs(L2 - L3), L2 + L3
dc = cmax - cmin
phitot = 0.5 * 2 * math.pi                           # barrido del tambor (rad)
phi = np.linspace(0, phitot, 2001)


def simulate(rfun):
    r = rfun(phi)
    c = cmin + np.concatenate([[0], np.cumsum((r[1:] + r[:-1]) / 2 * np.diff(phi))])
    T = K * c
    tau = T * r / 1000.0                             # N·m en el tambor
    return r, c, T, tau


def ripple(tau):
    return 100 * (tau.max() - tau.min()) / tau.mean()


r_cil = dc / phitot
Cid = (cmax ** 2 - cmin ** 2) / (2 * phitot)
prof = {}
prof['cilindro'] = simulate(lambda p: np.full_like(p, r_cil))
prof['ideal'] = simulate(lambda p: Cid / np.sqrt(cmin ** 2 + 2 * Cid * p))
best = None
for q in np.linspace(0.2, 1.0, 801):
    a0 = 2 * dc / (phitot * (1 + q))
    rr, cc, TT, tt = simulate(lambda p, a0=a0, q=q: a0 + (a0 * q - a0) * p / phitot)
    if best is None or tt.max() < best[0]:
        best = (tt.max(), q, a0)
qb, a0b = best[1], best[2]
prof['espiral lineal'] = simulate(lambda p: a0b + (a0b * qb - a0b) * p / phitot)
theta = np.degrees(np.arcsin(np.clip((L2 ** 2 + L3 ** 2 - prof['cilindro'][1] ** 2) / (2 * L2 * L3), -1, 1)))

fig, ax = plt.subplots(1, 3, figsize=(6.5, 2.6))
thg = np.linspace(90, -90, 400)
cg = np.sqrt(L2 ** 2 + L3 ** 2 - 2 * L2 * L3 * np.sin(np.radians(thg)))
ax[0].plot(thg, K * cg, color=C_M)
ax[0].set_xlabel(r'ángulo de la barra $\theta$ (°)')
ax[0].set_ylabel('tensión T (N)')
ax[0].invert_xaxis()
sty = {'cilindro': ('-', C_A), 'espiral lineal': ('--', C_B), 'ideal': ('-', C_F)}
for k_, (r, c, T, tau) in prof.items():
    ls, col = sty[k_]
    ax[1].plot(np.degrees(phi), r, ls, color=col, label=k_)
    ax[2].plot(np.degrees(phi), tau, ls, color=col, label=k_)
ax[1].set_xlabel(r'giro del tambor $\varphi$ (°)')
ax[1].set_ylabel('radio r (mm)')
ax[2].set_xlabel(r'giro del tambor $\varphi$ (°)')
ax[2].set_ylabel(r'par en el tambor $\tau$ (N·m)')
ax[1].legend(fontsize=7)
for a in (ax[1], ax[2]):
    a.set_xticks([0, 45, 90, 135, 180])
ax[0].set_xticks([90, 45, 0, -45, -90])
fig.tight_layout()
save(fig, 'modelo_perfiles')
num['modelo'] = {
    'K_N_por_mm': K, 'cmin_mm': cmin, 'cmax_mm': cmax, 'T_min_N': K * cmin, 'T_max_N': K * cmax,
    'phitot_rev': 0.5, 'r_cilindro_mm': r_cil,
    'ideal_r_ini_mm': float(prof['ideal'][0][0]), 'ideal_r_fin_mm': float(prof['ideal'][0][-1]),
    'ideal_tau_Nm': float(prof['ideal'][3].mean()), 'ideal_C_mm2_rad': Cid,
    'cil_tau_min': float(prof['cilindro'][3].min()), 'cil_tau_max': float(prof['cilindro'][3].max()),
    'cil_rizado_pct': ripple(prof['cilindro'][3]),
    'lin_r_ini_mm': float(prof['espiral lineal'][0][0]), 'lin_r_fin_mm': float(prof['espiral lineal'][0][-1]),
    'lin_tau_min': float(prof['espiral lineal'][3].min()), 'lin_tau_max': float(prof['espiral lineal'][3].max()),
    'lin_rizado_pct': ripple(prof['espiral lineal'][3]),
    'ideal_rizado_pct': ripple(prof['ideal'][3]),
    'reduccion_pico_ideal_vs_cil_pct': 100 * (1 - prof['ideal'][3].max() / prof['cilindro'][3].max()),
    'reduccion_pico_lin_vs_cil_pct': 100 * (1 - prof['espiral lineal'][3].max() / prof['cilindro'][3].max()),
    'cable_lin_mm': float(prof['espiral lineal'][1][-1] - cmin),
    'cable_ideal_mm': float(prof['ideal'][1][-1] - cmin),
    'motor_tau_ideal_Nm_red4': float(prof['ideal'][3].mean() / 4),
    'motor_tau_cilmax_Nm_red4': float(prof['cilindro'][3].max() / 4),
    'energia_J': float(m * g * (L3 + L4) * 2 / 1000),
}
# comprobación energética: ∫τ dφ = m·g·ΔH del centro de la pesa
for k_, (r, c, T, tau) in prof.items():
    num['modelo']['trabajo_' + k_.replace(' ', '_') + '_J'] = float(np.trapezoid(tau, phi))

# ───────────────────────── 9. Actividad en git ─────────────────────────
log = subprocess.run(['git', '-C', ROOT, 'log', '--date=short', '--format=@%ad', '--numstat'],
                     capture_output=True, text=True).stdout
commits = collections.Counter()
lines = collections.Counter()
cur = None
for l in log.splitlines():
    if l.startswith('@'):
        cur = datetime.date.fromisoformat(l[1:])
        wk = cur - datetime.timedelta(days=cur.weekday())
        commits[wk] += 1
    elif l.strip() and cur:
        a, b, f = l.split('\t', 2)
        if a.isdigit() and not any(s in f for s in ('fp-info-cache', '.kicad_', '.zip', '.log')):
            lines[cur - datetime.timedelta(days=cur.weekday())] += int(a)
w0 = min(commits)
w1 = max(commits)
weeks = []
w = w0
while w <= w1:
    weeks.append(w)
    w += datetime.timedelta(days=7)
fig, ax = plt.subplots(figsize=(6.3, 2.2))
ax.bar(weeks, [commits.get(w, 0) for w in weeks], width=5, color=C_M)
ax.set_ylabel('commits / semana')
fig.autofmt_xdate()
ax.xaxis.set_major_formatter(matplotlib.dates.DateFormatter('%d-%m-%y'))
fig.savefig(f'{OUT}/actividad_git.pdf')
plt.close(fig)
num['git_commits'] = sum(commits.values())
num['git_semanas_con_actividad'] = len(commits)
num['git_primera'] = str(w0)
num['git_ultima'] = str(w1)

json.dump(num, open(f'{OUT}/datos.json', 'w'), indent=1, ensure_ascii=False, default=float)
print(json.dumps({k: v for k, v in num.items() if k not in ('rep_filas', 'tabla_bins')}, indent=1,
                 ensure_ascii=False, default=float))

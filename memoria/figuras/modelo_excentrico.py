"""Tambor excentrico (cilindro con el eje desplazado) frente a la caracola.

Brazo efectivo de un cilindro de radio R cuyo eje de giro esta a distancia e de su
centro geometrico:            r(phi) = R - e*cos(phi + phi0)
El cable sale tangente, luego esta a esa distancia del eje de giro; y como para
cualquier tambor ds/dphi = r(phi), el cable pagado vale
                              s(phi) = R*phi - e*(sen(phi+phi0) - sen(phi0))

RESTRICCION COMUN A TODAS LAS HIPOTESIS: el tambor tiene que pagar el mismo cable
S = c1 - c2 en el mismo giro dphi, porque la carga recorre lo mismo. Eso fija R a
partir de (e, phi0) y deja solo dos parametros de forma:
                              R = (S + e*(sen(dphi+phi0) - sen(phi0))) / dphi
Sin esta restriccion el ajuste degenera en R = e = 0 (un tambor que no mueve la
carga tiene par constante nulo, y por tanto rizado cero).

Dos analisis:
  (a) AJUSTE   : que perfil explica el par medido, con escala k > 0 libre.
  (b) DISENO   : excentrico que minimiza el rizado de par. Responde a si basta un
                 excentrico o hace falta mecanizar una espiral.
"""
import sys, os, json, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from scipy.optimize import minimize, brentq
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter
from modelo_mecanismo import Q, od, incD, cHome, limites, tambor, load

OUT = os.path.dirname(os.path.abspath(__file__))
plt.rcParams.update({'font.family':'serif','font.size':9,'axes.grid':True,'grid.alpha':.3,
                     'savefig.bbox':'tight','legend.fontsize':8,'axes.titlesize':9,
                     'lines.linewidth':1.1})
C_UP,C_DN,C_M,C_F,C_E = '#d62728','#1f77b4','#222222','#2ca02c','#9467bd'
coma = FuncFormatter(lambda x,p: ('%g'%round(x,6)).replace('.',','))
def save(fig,n):
    for ax in fig.axes:
        ax.xaxis.set_major_formatter(coma); ax.yaxis.set_major_formatter(coma)
    fig.tight_layout(); fig.savefig(f'{OUT}/{n}.pdf'); plt.close(fig)

L_OD, INC, CB = od(Q), incD(Q), cHome(Q)
CMIN, CMAX = limites(Q)

def T_de_c(c):
    """Tension del cable para una longitud libre c (mm), segun el cap. 4."""
    co = (L_OD**2 + Q['L3']**2 - c*c) / (2*L_OD*Q['L3'])
    if not -1.0 <= co <= 1.0: return np.nan
    a  = math.degrees(math.acos(co)); th = INC - a
    sa = math.sin(math.radians(a))
    if sa < 1e-6 or not -90 <= th <= 90: return np.nan
    return Q['m']*Q['g']*(Q['L3']+Q['L4'])*c*math.cos(math.radians(th))/(L_OD*Q['L3']*sa)

# ── datos: los tres ensayos limpios, mismo promediado que la campana ──────
LIM = ['20260921-180637','20260921-180709','20260921-180846']
DS = {k: load('cycles/%s_ciclo.csv'%k) for k in LIM}
def mov(d):
    return ((d['ref_rpm']!=0) & (np.sign(d['rpm'])==np.sign(d['ref_rpm']))
            & (np.abs(d['rpm']) >= 0.6*np.abs(d['ref_rpm'])))
edges = np.arange(0,1.85,0.05); cent = (edges[:-1]+edges[1:])/2
def binned(par, umbral=6):
    U=[[] for _ in cent]; D=[[] for _ in cent]
    for d in DS.values():
        m0 = mov(d); up = d['fase']=='hacia_A'
        for msk,A in ((m0&up,U),(m0&~up,D)):
            for a,v in zip(d['av'][msk], d[par][msk]):
                i = np.searchsorted(edges,a)-1
                if 0 <= i < len(cent): A[i].append(v)
    f = lambda A: np.array([np.mean(x) if len(x)>=umbral else np.nan for x in A])
    return f(U), f(D)
TU,TD = binned('tau_ua')
TG = (TU+TD)/2                       # par gravitatorio, limpio de perdidas
CARGA = 0.65                         # antes de este avance el cable no tira

# ── recorrido con carga: mismo que la tabla de perfiles del cap. 4 ────────
avmax = float(np.mean([d['av'].max() for d in DS.values()]))
phi1 = CARGA*2*math.pi/Q['red']; phi2 = avmax*2*math.pi/Q['red']
DPHI = phi2 - phi1
C1 = CB - tambor(Q,phi1)[1]; C2 = CB - tambor(Q,phi2)[1]
S  = C1 - C2
cs = np.linspace(C2,C1,400)
W  = float(abs(np.trapezoid([T_de_c(c) for c in cs], cs))/1000)
TAU0 = W/DPHI
PH = np.linspace(0, DPHI, 300); H = PH[1]-PH[0]

def R_de(e, phi0):
    """Radio medio que cumple la restriccion de cable pagado."""
    return (S + e*(math.sin(DPHI+phi0) - math.sin(phi0))) / DPHI

def recorre(rf):
    """Integra el recorrido desde C1. Devuelve (r, c, tau) o None si no es fisico."""
    c = C1; o = []
    for p in PH:
        r = rf(p)
        if r <= 0: return None
        T = T_de_c(c)
        if np.isnan(T): return None
        o.append((r, c, T*r/1000.0)); c -= r*H
    return np.array(o)

def rizado(cv): return 100*(cv[:,2].max()-cv[:,2].min())/cv[:,2].mean()

def cv_exc(e, phi0):
    R = R_de(e, phi0)
    if R <= 0 or e > 0.9*R: return None
    return recorre(lambda p: R - e*math.cos(p+phi0))

N = {}

# ── curvas de referencia ─────────────────────────────────────────────────
CV_CIL = recorre(lambda p: S/DPHI)
CV_CAR = recorre(lambda p: tambor(Q, phi1+p)[0])
c_i = C1; ide = []
for p in PH:
    r = TAU0/T_de_c(c_i)*1000.0
    ide.append((r, c_i, TAU0)); c_i -= r*H
CV_IDE = np.array(ide)

# ── (b) DISENO: excentrico de rizado minimo ──────────────────────────────
def coste(x):
    cv = cv_exc(abs(x[0]), x[1])
    return 1e6 if cv is None else rizado(cv)
best = None
for e0 in (2,6,12,20,30):
    for f0 in np.linspace(-math.pi, math.pi, 13):
        r_ = minimize(coste, [e0,f0], method='Nelder-Mead',
                      options=dict(xatol=1e-6, fatol=1e-9, maxiter=6000))
        if r_.fun < 1e6 and (best is None or r_.fun < best.fun): best = r_
eo = abs(best.x[0]); p0o = (best.x[1] + math.pi) % (2*math.pi) - math.pi
Ro = R_de(eo, p0o); CV_EXC = cv_exc(eo, p0o)

N.update(W_J=W, dphi_rev=float(DPHI/(2*math.pi)), tau_ideal=TAU0, c1=float(C1), c2=float(C2),
         S_mm=float(S), dis_R=float(Ro), dis_e=float(eo), dis_phi0=float(math.degrees(p0o)),
         dis_rel=float(eo/Ro), dis_rmin=float(Ro-eo), dis_rmax=float(Ro+eo),
         dis_r_ini=float(CV_EXC[0,0]), dis_r_fin=float(CV_EXC[-1,0]),
         dis_r_var=float(100*(CV_EXC[-1,0]/CV_EXC[0,0]-1)),
         riz_cil=rizado(CV_CIL), riz_car=rizado(CV_CAR),
         riz_exc=rizado(CV_EXC), riz_ide=rizado(CV_IDE),
         tau_cil=[float(CV_CIL[:,2].min()),float(CV_CIL[:,2].max())],
         tau_car=[float(CV_CAR[:,2].min()),float(CV_CAR[:,2].max())],
         tau_exc=[float(CV_EXC[:,2].min()),float(CV_EXC[:,2].max())],
         r_cilindro=float(S/DPHI), r_ideal=[float(CV_IDE[0,0]),float(CV_IDE[-1,0])])

# ── (a) AJUSTE al par medido ─────────────────────────────────────────────
car_m = (cent>=CARGA) & ~np.isnan(TG)
AV, TAUG = cent[car_m], TG[car_m]
PHM = (AV-CARGA)*2*math.pi/Q['red']          # giro de tambor desde el inicio de carga

def serie_en(rf):
    """r y tau en los puntos medidos, partiendo de C1 con paso fino."""
    fino = np.linspace(0, PHM[-1], 2000); h = fino[1]-fino[0]
    c = C1; rs=[]; cc=[]
    for p in fino:
        r = rf(p)
        if r <= 0: return None, None
        rs.append(r); cc.append(c); c -= r*h
    rs = np.interp(PHM, fino, rs); cc = np.interp(PHM, fino, cc)
    tau = np.array([T_de_c(c)*r/1000.0 for r,c in zip(rs,cc)])
    return (None,None) if np.isnan(tau).any() else (rs, tau)

def metricas(tau):
    """Escala k > 0 por minimos cuadrados; R2 verdadero y rho2 (invariante)."""
    k  = float(np.dot(TAUG,tau)/np.dot(tau,tau))
    ss = float(((TAUG-TAUG.mean())**2).sum())
    R2 = 1 - float(((TAUG-k*tau)**2).sum())/ss
    rho2 = float(np.corrcoef(TAUG,tau)[0,1]**2)
    return k, R2, rho2

hip = {}
hip['cilindro']  = serie_en(lambda p: S/DPHI)
hip['caracola']  = serie_en(lambda p: tambor(Q, phi1+p)[0])
Qi = dict(Q, r0=Q['r1'], r1=Q['r0'])
hip['invertida'] = serie_en(lambda p: tambor(Qi, phi1+p)[0])

def coste_fit(x):
    e, phi0 = abs(x[0]), x[1]
    R = R_de(e, phi0)
    if R <= 0 or e > 0.9*R: return 1e9
    rs, tau = serie_en(lambda p: R - e*math.cos(p+phi0))
    if tau is None: return 1e9
    k,_,_ = metricas(tau)
    return float(((TAUG-k*tau)**2).sum())
bf = None
for e0 in (2,6,12,20,30):
    for f0 in np.linspace(-math.pi, math.pi, 13):
        r_ = minimize(coste_fit, [e0,f0], method='Nelder-Mead',
                      options=dict(xatol=1e-6, fatol=1e-9, maxiter=6000))
        if r_.fun < 1e9 and (bf is None or r_.fun < bf.fun): bf = r_
ef = abs(bf.x[0]); p0f = (bf.x[1] + math.pi) % (2*math.pi) - math.pi
Rf = R_de(ef, p0f)
hip['excentrico'] = serie_en(lambda p: Rf - ef*math.cos(p+p0f))

for nom,(rs,tau) in hip.items():
    k,R2,rho2 = metricas(tau)
    N['R2_'+nom] = R2; N['rho2_'+nom] = rho2; N['k_'+nom] = k
    N['tauvar_'+nom] = float(100*(tau[-1]/tau[0]-1))     # variacion sobre el tramo con carga
    N['rvar_'+nom]   = float(100*(rs[-1]/rs[0]-1))
N['tauvar_medido'] = float(100*(TAUG[-1]/TAUG[0]-1))
Tm_ = np.array([T_de_c(CB - tambor(Q,a*2*math.pi/Q['red'])[1]) for a in AV])
N['rvar_medido'] = float(100*((TAUG[-1]/Tm_[-1])/(TAUG[0]/Tm_[0])-1))
N.update(exc_R=float(Rf), exc_e=float(ef), exc_phi0=float(math.degrees(p0f)),
         exc_rel=float(ef/Rf), exc_rmin=float(Rf-ef), exc_rmax=float(Rf+ef))

# ── (c) LA PIEZA REAL: cotas tomadas del modelo CAD ──────────────────────
# Dos cotas radiales sobre la misma diametral, medidas desde el eje de giro.
R_MIN_CAD, R_MAX_CAD = 28.179, 56.00
R_CAD = (R_MAX_CAD + R_MIN_CAD)/2
E_CAD = (R_MAX_CAD - R_MIN_CAD)/2

def dphi_de(phi0, R=R_CAD, e=E_CAD):
    """Giro necesario para pagar S con un tambor dado. s(phi) es creciente si R>e."""
    s = lambda p: R*p - e*(math.sin(p+phi0) - math.sin(phi0))
    try:    return brentq(lambda p: s(p) - S, 1e-6, 2*math.pi)
    except ValueError: return None

def curva_real(phi0, R=R_CAD, e=E_CAD):
    dp = dphi_de(phi0, R, e)
    if dp is None: return None, None
    ph = np.linspace(0, dp, 300); h = ph[1]-ph[0]
    c = C1; o = []
    for p in ph:
        r = R - e*math.cos(p+phi0)
        if r <= 0: return None, None
        T = T_de_c(c)
        if np.isnan(T): return None, None
        o.append((r, c, T*r/1000.0)); c -= r*h
    return np.array(o), dp

fases = np.linspace(-math.pi, math.pi, 721)
res = [(f,)+ (lambda cv,dp: (rizado(cv), dp) if cv is not None else (np.inf, None))(*curva_real(f))
       for f in fases]
ok = [x for x in res if np.isfinite(x[1])]
f_best, riz_best, dp_best = min(ok, key=lambda x: x[1])
f_peor, riz_peor, _       = max(ok, key=lambda x: x[1])
CV_REAL, _ = curva_real(f_best)

# excentricidad ideal manteniendo el radio medio de la pieza real
def coste_e(x):
    e = abs(x[0]); f = x[1]
    if e >= R_CAD: return 1e6
    cv, _ = curva_real(f, R_CAD, e)
    return 1e6 if cv is None else rizado(cv)
be = None
for e0 in (4,8,14,20):
    for f0 in np.linspace(-math.pi, math.pi, 13):
        r_ = minimize(coste_e, [e0,f0], method='Nelder-Mead',
                      options=dict(xatol=1e-6, fatol=1e-9, maxiter=6000))
        if r_.fun < 1e6 and (be is None or r_.fun < be.fun): be = r_

N.update(cad_rmin=R_MIN_CAD, cad_rmax=R_MAX_CAD, cad_R=float(R_CAD), cad_e=float(E_CAD),
         cad_rel=float(E_CAD/R_CAD), cad_phi0=float(math.degrees(f_best)),
         cad_riz=float(riz_best), cad_riz_peor=float(riz_peor),
         cad_dphi_rev=float(dp_best/(2*math.pi)), cad_dphi_deg=float(math.degrees(dp_best)),
         cad_r_ini=float(CV_REAL[0,0]), cad_r_fin=float(CV_REAL[-1,0]),
         cad_tau=[float(CV_REAL[:,2].min()), float(CV_REAL[:,2].max())],
         cad_e_optima=float(abs(be.x[0])), cad_riz_e_optima=float(be.fun),
         cad_rel_optima=float(abs(be.x[0])/R_CAD))

# ── par en el motor frente al nominal del servo ──────────────────────────
TAU_NOM = 1.27
N['tau_nom'] = TAU_NOM
for nom, red in (('act',Q['red']), ('cap',8.55)):
    N['taum_'+nom] = float(max(CV_EXC[:,2])/red)
    N['taum_'+nom+'_pct'] = float(100*max(CV_EXC[:,2])/red/TAU_NOM)
    N['taum_'+nom+'_car'] = float(max(CV_CAR[:,2])/red)
    N['taum_'+nom+'_car_pct'] = float(100*max(CV_CAR[:,2])/red/TAU_NOM)

# ── FIGURA ──────────────────────────────────────────────────────────────
fig, ax = plt.subplots(1, 2, figsize=(6.3, 2.8))
Tm = np.array([T_de_c(CB - tambor(Q,a*2*math.pi/Q['red'])[1]) for a in AV])
ref = TAUG/Tm; ref = ref/ref.max()
ax[0].plot(AV, ref, '-o', ms=3, color=C_M, label=r'medido $\propto\tau_g/T$')
for nom, est, col, lab in (('caracola','--',C_UP,'caracola 45$\\to$25 mm'),
                           ('invertida',':',C_DN,'invertida 25$\\to$45 mm'),
                           ('excentrico','-',C_E,'excéntrico ajustado')):
    rs,_ = hip[nom]
    ax[0].plot(AV, rs/rs.max(), est, color=col, label=lab)
ax[0].set_xlabel('avance desde B (rev de motor)'); ax[0].set_ylabel('radio normalizado')
ax[0].legend(fontsize=6.5, loc='lower right')
ax[0].set_title('(a) ¿qué perfil explica la medida?', fontsize=8.5)

g = np.degrees(PH)
ax[1].plot(g, CV_IDE[:,2], '-',  color=C_F, lw=3.2, alpha=.45, label='ideal')
ax[1].plot(g, CV_CIL[:,2], '-',  color=C_UP, label='cilindro')
ax[1].plot(g, CV_CAR[:,2], '--', color=C_DN, label='caracola config.')
ax[1].plot(g, CV_EXC[:,2], '-',  color=C_E,  label='excéntrico óptimo')
ax[1].set_xlabel('giro del tambor (°)'); ax[1].set_ylabel(r'$\tau$ en el tambor (N·m)')
ax[1].legend(fontsize=6.5); ax[1].set_title('(b) ¿basta un excéntrico?', fontsize=8.5)
save(fig, 'n_excentrico')

json.dump(N, open(f'{OUT}/datos_exc.json','w'), indent=1, ensure_ascii=False, default=float)
print('  recorrido con carga: %.3f rev de tambor, cable %.1f -> %.1f mm (paga %.1f), W = %.3f J'
      % (N['dphi_rev'], C1, C2, S, W))
print('  (a) AJUSTE al par medido:')
print('        %-11s %8s %8s' % ('hipotesis','R2','rho2'))
for k_ in ('cilindro','caracola','invertida','excentrico'):
    print('        %-11s %8.3f %8.3f' % (k_, N['R2_'+k_], N['rho2_'+k_]))
print('      excentrico ajustado: R=%.1f mm  e=%.1f mm  e/R=%.2f  phi0=%.0f deg  (r: %.1f a %.1f mm)'
      % (N['exc_R'], N['exc_e'], N['exc_rel'], N['exc_phi0'], N['exc_rmin'], N['exc_rmax']))
print('  (b) DISENO — rizado de par:')
for k_,lab in (('riz_cil','cilindro'),('riz_car','caracola config.'),
               ('riz_exc','excentrico optimo'),('riz_ide','ideal')):
    print('        %-18s %6.2f %%' % (lab, N[k_]))
print('      excentrico optimo: R=%.1f mm  e=%.1f mm  e/R=%.2f  phi0=%.0f deg'
      % (N['dis_R'], N['dis_e'], N['dis_rel'], N['dis_phi0']))
print('      radio %.1f -> %.1f mm en el recorrido (%+.0f %%), %.1f a %.1f mm en una vuelta'
      % (N['dis_r_ini'], N['dis_r_fin'], N['dis_r_var'], N['dis_rmin'], N['dis_rmax']))
print('  par de pico en el motor con el excentrico optimo:')
print('        N=5,5  -> %.3f N.m (%.0f %% del nominal %.2f)' % (N['taum_act'], N['taum_act_pct'], TAU_NOM))
print('        N=8,55 -> %.3f N.m (%.0f %% del nominal)' % (N['taum_cap'], N['taum_cap_pct']))

print('  (c) LA PIEZA REAL (cotas del CAD: %.3f y %.2f mm desde el eje):' % (R_MIN_CAD, R_MAX_CAD))
print('        R = %.1f mm, e = %.1f mm, e/R = %.2f' % (N['cad_R'], N['cad_e'], N['cad_rel']))
print('        recorrido necesario: %.1f deg (%.3f rev de tambor)' % (N['cad_dphi_deg'], N['cad_dphi_rev']))
print('        mejor calado phi0 = %+.0f deg  ->  rizado %.1f %%  (radio %.1f -> %.1f mm)'
      % (N['cad_phi0'], N['cad_riz'], N['cad_r_ini'], N['cad_r_fin']))
print('        peor calado                ->  rizado %.1f %%' % N['cad_riz_peor'])
print('        par en el tambor: %.2f a %.2f N.m' % tuple(N['cad_tau']))
print('        con R=%.1f fijo, la e ideal seria %.1f mm (e/R=%.2f) -> rizado %.2f %%'
      % (N['cad_R'], N['cad_e_optima'], N['cad_rel_optima'], N['cad_riz_e_optima']))

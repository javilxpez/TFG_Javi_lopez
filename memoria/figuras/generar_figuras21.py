"""Campana del 21-09 con el modelo nuevo: figuras y cifras de la memoria."""
import sys, json, math, glob, os
sys.path.insert(0,'/tmp/claude-1001/-home-uni-Documents-TFG-Javi-lopez/463d5d95-2d9e-457a-9228-f0765570b5ef/scratchpad')
from mec import *
import numpy as np, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter
OUT='/home/uni/Documents/TFG_Javi_lopez/memoria/figuras'
plt.rcParams.update({'font.family':'serif','font.size':9,'axes.grid':True,'grid.alpha':.3,
 'savefig.bbox':'tight','legend.fontsize':8,'axes.titlesize':9,'lines.linewidth':1.1})
C_UP,C_DN,C_M,C_F='#d62728','#1f77b4','#222222','#2ca02c'
coma=FuncFormatter(lambda x,p:('%g'%round(x,6)).replace('.',','))
def save(fig,n):
    for ax in fig.axes: ax.xaxis.set_major_formatter(coma); ax.yaxis.set_major_formatter(coma)
    fig.tight_layout(); fig.savefig(f'{OUT}/{n}.pdf'); plt.close(fig)
def mov(d): return (d['ref_rpm']!=0)&(np.sign(d['rpm'])==np.sign(d['ref_rpm']))&(np.abs(d['rpm'])>=0.6*np.abs(d['ref_rpm']))
def r2(x,y):
    m=~np.isnan(x)&~np.isnan(y)
    return float(np.corrcoef(x[m],y[m])[0,1]**2) if m.sum()>=5 else float('nan')
N={}
LIM=['20260921-180637','20260921-180709','20260921-180846']
DS={k:load('cycles/%s_ciclo.csv'%k) for k in LIM}
OTROS={os.path.basename(p)[:15]:load(p) for p in sorted(glob.glob('cycles/20260921-18*.csv'))
       if os.path.basename(p)[:15] not in LIM}
# ── 1. Arranque con carga ──
ini=[]
for k,d in DS.items():
    m=mov(d)&(d['fase']=='hacia_A'); av,t=d['av'][m],d['tau_ua'][m]
    ini.append(float(av[np.argmax(t>10)]))
N['carga_ini']=float(np.mean(ini)); N['carga_ini_lista']=ini
CARGA=0.65
# ── 2. Temporizacion (ensayos sanos) ──
dts=[];work=[]
for d in list(DS.values())+list(OTROS.values()):
    dt=np.diff(d['t_rel_s'])*1000
    dts.append(dt[(dt>0)&(dt<200)]); work.append(d['work_us']/1000)
dts=np.concatenate(dts); work=np.concatenate(work)
N.update(dt_media=float(dts.mean()),dt_sd=float(dts.std()),dt_frac50=float((np.abs(dts-50)<0.5).mean()),
         work_media=float(work.mean()),work_max=float(work.max()),work_min=float(work.min()))
# ── 3. Binned ──
edges=np.arange(0,1.85,0.05); cent=(edges[:-1]+edges[1:])/2
def binned(par,umbral=6):
    U=[[] for _ in cent]; D=[[] for _ in cent]
    for d in DS.values():
        m0=mov(d); up=d['fase']=='hacia_A'
        for msk,A in ((m0&up,U),(m0&~up,D)):
            for a,v in zip(d['av'][msk],d[par][msk]):
                i=np.searchsorted(edges,a)-1
                if 0<=i<len(cent): A[i].append(v)
    f=lambda A:np.array([np.mean(x) if len(x)>=umbral else np.nan for x in A])
    return f(U),f(D)
TU,TD=binned('tau_ua'); CU,CD=binned('lc1_base')
TG=(TU+TD)/2; TF=(TU-TD)/2; CN=(CU+CD)/2
T0,tau0,ang0,r0m=modelo_series(Q,cent)
Qinv=dict(Q,r0=Q['r1'],r1=Q['r0']); Ti,taui,angi,ri=modelo_series(Qinv,cent)
car=(cent>=CARGA)&~np.isnan(TG)&~np.isnan(T0)
N['bins_carga']=int(car.sum()); N['av_ini']=float(cent[car][0]); N['av_fin']=float(cent[car][-1])
N['taug_ini']=float(TG[car][0]); N['taug_fin']=float(TG[car][-1]); N['taug_var']=100*(TG[car][-1]/TG[car][0]-1)
N['T_ini']=float(T0[car][0]); N['T_fin']=float(T0[car][-1]); N['T_var']=100*(T0[car][-1]/T0[car][0]-1)
ref=TG[car]/T0[car]; N['ref_var']=100*(ref[-1]/ref[0]-1); N['ref_ini']=float(ref[0]); N['ref_fin']=float(ref[-1])
N['r_const_necesario']=100*(T0[car][0]/T0[car][-1]-1)
N['tauf']=float(np.nanmean(TF[car])); N['tauf_pct']=100*N['tauf']/np.nanmean(TG[car])
N['r2_cuentas_T']=r2(CN[car],T0[car]); N['r2_tau_actual']=r2(TG[car],tau0[car]); N['r2_tau_invertida']=r2(TG[car],taui[car])
N['pend_actual']=float(np.polyfit(TG[car],tau0[car],1)[0]); N['pend_invertida']=float(np.polyfit(TG[car],taui[car],1)[0])
N['fit_cuentas_por_N']=float(1/np.polyfit(CN[car],T0[car],1)[0])
# ── 4. Perfiles sobre el recorrido con carga ──
avmax=float(np.mean([d['av'].max() for d in DS.values()])); N['av_max']=avmax
phi1=CARGA*2*math.pi/Q['red']; phi2=avmax*2*math.pi/Q['red']; dphi=phi2-phi1
cB=cHome(Q); c1=cB-tambor(Q,phi1)[1]; c2=cB-tambor(Q,phi2)[1]
def Tde_c(c):
    L=od(Q); co=max(-1,min(1,(L*L+Q['L3']**2-c*c)/(2*L*Q['L3'])))
    a=math.degrees(math.acos(co)); th=incD(Q)-a
    return Q['m']*Q['g']*(Q['L3']+Q['L4'])*c*math.cos(math.radians(th))/(L*Q['L3']*math.sin(math.radians(a)))
cs=np.linspace(c2,c1,300); W=float(abs(np.trapezoid([Tde_c(c) for c in cs],cs))/1000)
N.update(W_J=W,dphi_rev=dphi/(2*math.pi),tau_ideal=W/dphi,c1=float(c1),c2=float(c2))
ph=np.linspace(0,dphi,300)
def curva(rf):
    c=c1; o=[]; h=ph[1]-ph[0]
    for p in ph: r=rf(p,c); o.append((r,c,Tde_c(c)*r/1000)); c-=r*h
    return np.array(o)
rc=(c1-c2)/dphi
cil=curva(lambda p,c: rc); act=curva(lambda p,c: tambor(Q,phi1+p)[0]); ide=curva(lambda p,c:(W/dphi)/Tde_c(c)*1000)
for nom,cv in (('cil',cil),('act',act),('ide',ide)):
    t=cv[:,2]; N['tau_'+nom]=[float(t.min()),float(t.max())]; N['riz_'+nom]=float(100*(t.max()-t.min())/t.mean())
N['r_cilindro']=float(rc); N['r_ideal']=[float(ide[0,0]),float(ide[-1,0])]
# ── FIGURAS ──
d=DS['20260921-180846']; ts=d['t_rel_s']; sube=d['fase']=='hacia_A'
Tm,_,_,_=modelo_series(Q,d['av'])
fig,ax=plt.subplots(4,1,figsize=(6.3,7.4),sharex=True)
tcarga=ts[(d['av']>=N['carga_ini'])&sube][0]
for a in ax:
    a.axvspan(ts[0],tcarga,color='gray',alpha=.16,lw=0)
    a.axvspan(ts[sube][-1],ts[-1] if ts[-1]>ts[sube][-1] else ts[-1],color=C_DN,alpha=.05,lw=0)
ax[0].plot(ts,d['av'],color=C_M); ax[0].axhline(N['carga_ini'],ls=':',color='gray')
ax[0].set_ylabel('avance desde B\n(rev de motor)')
ax[1].step(ts,d['ref_rpm'],where='post',color='gray',label='consigna'); ax[1].plot(ts,d['rpm'],color=C_M,label='medida')
ax[1].set_ylabel('velocidad (rpm)'); ax[1].legend(loc='lower right',ncol=2)
ax[2].plot(ts,d['lc1_base'],color=C_F,label='LC1 (cuentas)')
a2=ax[2].twinx(); a2.plot(ts,Tm,'--',color=C_DN); a2.set_ylabel('T del modelo (N)'); a2.grid(False)
ax[2].set_ylabel('LC1 (cuentas)'); ax[2].legend(loc='center left')
ax[3].plot(ts,d['tau_ua'],color=C_M); ax[3].set_ylabel('|par| (u. accionam.)'); ax[3].set_xlabel('tiempo (s)')
ax[0].set_title('Ensayo 21-09 18:08:46 · ciclo B→A→B (gris: tramo sin carga)')
save(fig,'n_ensayo_tipo')

fig,ax=plt.subplots(3,1,figsize=(6.3,6.6),sharex=True)
for a in ax: a.axvspan(0,CARGA,color='gray',alpha=.16,lw=0)
sel=~np.isnan(TG)
ax[0].plot(cent[sel],CU[sel],'-o',ms=3,color=C_UP,label='subida B→A')
ax[0].plot(cent[sel],CD[sel],'-o',ms=3,color=C_DN,label='bajada A→B')
a0=ax[0].twinx(); a0.plot(cent,T0,'--',lw=2,color=C_M); a0.set_ylabel('T del modelo (N)'); a0.grid(False)
ax[0].set_ylabel('LC1 (cuentas)'); ax[0].legend(loc='lower left')
ax[1].plot(cent[sel],TU[sel],'-o',ms=3,color=C_UP,label='subida'); ax[1].plot(cent[sel],TD[sel],'-o',ms=3,color=C_DN,label='bajada')
ax[1].plot(cent[sel],TG[sel],'-',lw=2,color=C_M,label=r'$\tau_g$'); ax[1].plot(cent[sel],TF[sel],'--',color=C_F,label=r'$\tau_f$')
ax[1].set_ylabel('|par| (u. accionam.)'); ax[1].legend(ncol=2,loc='upper left')
rr=TG/T0; rn=rr/np.nanmax(rr[car])
ax[2].plot(cent[car],rn[car],'-o',ms=3,color=C_M,label=r'medido $\propto\tau_g/T$')
ax[2].plot(cent[car],(r0m[car]/np.nanmax(r0m[car])),'--',color=C_UP,label='config. actual (45→25 mm)')
ax[2].plot(cent[car],(ri[car]/np.nanmax(ri[car])),':',color=C_DN,label='invertida (25→45 mm)')
ax[2].set_ylabel('radio normalizado'); ax[2].set_xlabel('avance desde B (rev de motor)'); ax[2].legend(fontsize=7)
save(fig,'n_descomposicion')

fig,ax=plt.subplots(1,2,figsize=(6.3,2.7))
ax[0].plot(cent[car],TG[car]/np.nanmax(TG[car]),'-o',ms=3,color=C_M,label='medido')
ax[0].plot(cent[car],tau0[car]/np.nanmax(tau0[car]),'--',color=C_UP,label='modelo 45→25')
ax[0].plot(cent[car],taui[car]/np.nanmax(taui[car]),':',color=C_DN,label='modelo 25→45')
ax[0].set_xlabel('avance desde B (rev)'); ax[0].set_ylabel('par normalizado'); ax[0].legend(fontsize=7)
ax[1].plot(np.degrees(ph),cil[:,2],'-',color=C_UP,label='cilindro')
ax[1].plot(np.degrees(ph),act[:,2],'--',color=C_DN,label='config. actual')
ax[1].plot(np.degrees(ph),ide[:,2],'-',color=C_F,label='ideal')
ax[1].set_xlabel(r'giro del tambor (°)'); ax[1].set_ylabel(r'$\tau$ en el tambor (N·m)'); ax[1].legend(fontsize=7)
save(fig,'n_perfiles')

fig,ax=plt.subplots(1,2,figsize=(6.3,2.6))
for d in DS.values():
    m=mov(d); up=d['fase']=='hacia_A'
    ax[0].plot(d['av'][m&up],d['lc1_base'][m&up],'.',ms=2,color=C_UP,alpha=.6)
    ax[0].plot(d['av'][m&~up],d['lc1_base'][m&~up],'.',ms=2,color=C_DN,alpha=.6)
    ax[1].plot(d['av'][m&up],d['tau_ua'][m&up],'.',ms=2,color=C_UP,alpha=.6)
    ax[1].plot(d['av'][m&~up],d['tau_ua'][m&~up],'.',ms=2,color=C_DN,alpha=.6)
for a in ax: a.axvspan(0,CARGA,color='gray',alpha=.16,lw=0); a.set_xlabel('avance desde B (rev)')
ax[0].plot([],[],'o',color=C_UP,label='subida'); ax[0].plot([],[],'o',color=C_DN,label='bajada'); ax[0].legend()
ax[0].set_ylabel('LC1 (cuentas)'); ax[1].set_ylabel('|par| (u. accionam.)')
save(fig,'n_fuerza_par')
# repetibilidad
N['rep']=[dict(ensayo=k,av_max=float(d['av'].max()),av_fin=float(d['av'][-1]),dur=float(d['t_rel_s'][-1])) for k,d in DS.items()]
N['rep_avmax_sd']=float(np.std([r['av_max'] for r in N['rep']],ddof=1))
json.dump(N,open(f'{OUT}/datos21.json','w'),indent=1,ensure_ascii=False,default=float)
for k,v in N.items():
    if not isinstance(v,(list,dict)): print('  %-22s %s'%(k, round(v,4) if isinstance(v,float) else v))
print('  rep:',[(r['ensayo'][-6:],round(r['av_max'],3),round(r['av_fin'],3)) for r in N['rep']])
print('  tau cil/act/ide:',[ (round(N['tau_cil'][0],2),round(N['tau_cil'][1],2)),(round(N['tau_act'][0],2),round(N['tau_act'][1],2)),(round(N['tau_ide'][0],2),round(N['tau_ide'][1],2))])

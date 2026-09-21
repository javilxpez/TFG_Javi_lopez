"""Modelo nuevo del mecanismo (port de static/mecanismo.js) + analisis de ensayos."""
import csv, glob, os, math, json
import numpy as np

Q = dict(tipo='car', L1=25.0, r0=45.0, r1=25.0, barr=0.3, L2=290.0, dx=65.0,
         L3=95.0, L4=110.0, m=2.0, g=9.81, a0=114.0, red=5.5, sentido=1)

def od(q): return math.hypot(q['dx'], q['L2'])
def incD(q): return math.degrees(math.atan2(q['L2'], -q['dx']))
def cable(q, aDeg):
    L=od(q); return math.sqrt(L*L + q['L3']**2 - 2*L*q['L3']*math.cos(math.radians(aDeg)))
def cHome(q): return cable(q, q['a0'])
def limites(q):
    L=od(q); return abs(L-q['L3']), L+q['L3']
def tambor(q, phi):
    if q['tipo']=='cil': return q['L1'], q['L1']*phi
    phitot=q['barr']*2*math.pi
    if phi<=0: return q['r0'], q['r0']*phi
    if phi<=phitot:
        r=q['r0']+(q['r1']-q['r0'])*phi/phitot
        return r, q['r0']*phi+(q['r1']-q['r0'])*phi*phi/(2*phitot)
    sTot=phitot*(q['r0']+q['r1'])/2
    return q['r1'], sTot+q['r1']*(phi-phitot)
def estado(q, avRev):
    phi=avRev*2*math.pi/q['red']
    r,s=tambor(q,phi)
    c=cHome(q)-q['sentido']*s
    cmin,cmax=limites(q)
    if not (cmin<=c<=cmax): return None
    L=od(q); co=(L*L+q['L3']**2-c*c)/(2*L*q['L3'])
    co=max(-1.0,min(1.0,co)); a=math.degrees(math.acos(co))
    th=incD(q)-a
    if not (-90<=th<=90): return None
    sa=math.sin(math.radians(a))
    if sa<1e-6: return None
    T=q['m']*q['g']*(q['L3']+q['L4'])*c*math.cos(math.radians(th))/(L*q['L3']*sa)
    return dict(phi=phi,r=r,s=s,c=c,a=a,th=th,T=T,tau=T*r/1000)

def load(p):
    consts={}; lines=[]
    for l in open(p,encoding='utf-8'):
        if l.startswith('#'):
            for tok in l.lstrip('#').split():
                if '=' in tok: k,v=tok.split('=',1); consts[k]=v
        else: lines.append(l)
    R=list(csv.DictReader(lines))
    d={k:np.array([float(r[k]) for r in R]) for k in
       ['t_rel_s','rep','pos_rev','lc1_raw','lc1_base','rpm','par_x10','ref_rpm','lim_a','lim_b','work_us']}
    d['fase']=np.array([r['fase'] for r in R]); d['name']=os.path.basename(p)[:15]
    cal=float(consts.get('lc1_cuentas_por_N',0) or 0); off=float(consts.get('lc1_cero_N',0) or 0)
    d['cal'],d['off']=cal,off
    d['F']=d['lc1_base']/cal+off if cal else None
    d['tau_ua']=np.abs(d['par_x10'])/10.0
    pB=d['pos_rev'][0]; rel=d['pos_rev']-pB
    i=int(np.argmax(np.abs(rel))); d['dir']=-1.0 if rel[i]<0 else 1.0
    d['av']=rel*d['dir']
    return d

def modelo_series(q, av):
    T=np.full(len(av),np.nan); tau=np.full(len(av),np.nan); ang=np.full(len(av),np.nan); rr=np.full(len(av),np.nan)
    for i,a in enumerate(av):
        st=estado(q,a)
        if st: T[i],tau[i],ang[i],rr[i]=st['T'],st['tau'],st['a'],st['r']
    return T,tau,ang,rr

def r2(y,yh):
    m=~np.isnan(yh)&~np.isnan(y)
    if m.sum()<5: return float('nan'),0
    yy=y[m]; ss=((yy-yy.mean())**2).sum()
    return (1-((yy-yh[m])**2).sum()/ss if ss>0 else float('nan')), int(m.sum())

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wmisleading-indentation"
#endif
/* Original PLECS double core, preserved as a host-test oracle. */
#ifndef TOC_FIRST_ORDER_PADE_TWO_TARGET_CORE_H
#define TOC_FIRST_ORDER_PADE_TWO_TARGET_CORE_H
/* Stateless two-pulse [1/1] Pade planner with separate phase terminal currents.
 * ON-OFF targets [I_ref_valley,Vref]; OFF-ON targets [I_ref_peak,Vref].
 * Physical Iload remains independent
 * in the capacitor dynamics. Each order retains both real quadratic roots;
 * the shortest endpoint-verified nonnegative candidate is selected.
 * No search iteration, time cap, pulse rounding, gain or state clamp.
 * Feasibility is checked by the two forward interval maps, with no separate
 * steady-duty restriction on an arbitrary requested terminal state.
 */
#include <math.h>
#include <float.h>
typedef enum {
 TOC_IDLE=0,TOC_PLAN=1,TOC_ALREADY_TARGET=2,
 TOC_INVALID=-1,TOC_UNHOLDABLE_TARGET=-2,TOC_NO_SOLUTION=-3
} TOC_Status;
/* First five fields preserve the original planner adapter contract. */
typedef struct {
 double ton,toff;int order,valid,status;
 double target_i,target_v;
} TOC_Result;

/* Exactly one coupled physical [1/1] Pade/trapezoidal interval. No measured
 * voltage is substituted inside this predictor, no slope is frozen at Vref. */
static int toc_advance(double *i,double *v,double t,int gate,
 double L,double C,double R,double vin,double iload){
 if(!i||!v||!isfinite(*i)||!isfinite(*v)||!isfinite(t)||t<0||
  (gate!=0&&gate!=1)||!isfinite(L)||!isfinite(C)||!isfinite(R)||
  !isfinite(vin)||!isfinite(iload)||!(L>0&&C>0&&R>=0))return 0;
 if(t==0)return 1;
 double h=t/2,th=R/L,ri=(1-th*h)*(*i)-h/L*(*v)+t*vin*gate/L;
 double rv=h/C*(*i)+(*v)-t*iload/C,det=1+th*h+h*h/(L*C);
 if(!(det>0)||!isfinite(det))return 0;
 double ni=(ri-h/L*rv)/det,nv=(h/C*ri+(1+th*h)*rv)/det;
 if(!isfinite(ni)||!isfinite(nv))return 0;*i=ni;*v=nv;return 1;
}

static void toc_consider(TOC_Result *best,double on,double off,int order,
 double i0,double v0,double iload,double target_i,double target_v,double vin,
 double L,double C,double R){
 if(!isfinite(on)||!isfinite(off)||on<0||off<0||(order!=1&&order!=-1))return;
 double total=on+off;if(!(total>0)||!isfinite(total))return;
 /* Strictly shorter candidate wins; exact ties retain the first order/root. */
 if(best->valid&&total>=best->ton+best->toff)return;
 double i=i0,v=v0;
 if(order>0){if(!toc_advance(&i,&v,on,1,L,C,R,vin,iload)||
                 !toc_advance(&i,&v,off,0,L,C,R,vin,iload))return;}
 else{if(!toc_advance(&i,&v,off,0,L,C,R,vin,iload)||
             !toc_advance(&i,&v,on,1,L,C,R,vin,iload))return;}
 double tol_i=1e-8*fmax(1,fmax(fabs(i0),fabs(target_i)));
 double tol_v=1e-8*fmax(1,fmax(fabs(v0),fabs(target_v)));
 if(fabs(i-target_i)>tol_i||fabs(v-target_v)>tol_v)return;
 best->ton=on;best->toff=off;best->order=order;
 best->valid=1;best->status=TOC_PLAN;
 best->target_i=target_i;best->target_v=target_v;
}

static TOC_Result toc_plan(double i0,double v0,double iload,double vref,double vin,
 double L,double C,double R,double iref_valley,double iref_peak){
 TOC_Result best={0,0,0,0,TOC_INVALID,0,0};
 if(!isfinite(i0)||!isfinite(v0)||!isfinite(iload)||!isfinite(vref)||
  !isfinite(vin)||!isfinite(L)||!isfinite(C)||!isfinite(R)||
  !isfinite(iref_valley)||!isfinite(iref_peak)||!(L>0&&C>0&&R>=0&&vin>0))return best;
 /* Entry is a normal PWM rising edge. Already at the valley target means
  * no forced pulse. A peak-only match is not a no-op: the scheduler would
  * otherwise stay ON instead of beginning the required normal OFF phase.
  * Equal targets retain the previous single-target already-there behavior. */
 if(fabs(i0-iref_valley)<=32*DBL_EPSILON*fmax(1,fmax(fabs(i0),fabs(iref_valley)))&&
    fabs(v0-vref)<=32*DBL_EPSILON*fmax(1,fmax(fabs(v0),fabs(vref)))){
  best.status=TOC_ALREADY_TARGET;best.order=1;
  best.target_i=iref_valley;best.target_v=vref;return best;
 }
 best.status=TOC_NO_SOLUTION;
 const double U=1e-6; /* Conditioning units only; no duration bound. */
 for(int order=1;order>=-1;order-=2){
  double target_i=order>0?iref_valley:iref_peak,target_v=vref;
  double d0=i0-target_i,d1=v0-target_v;
  double mi=.5*i0+.5*target_i,mv=.5*v0+.5*target_v;
  double p0=fma(-R,mi,vin-mv)/L*U;
  double p1=(mi-iload)/C*U;
  double q0=fma(-R,mi,-mv)/L*U,q1=p1;
  double G=fma(-R,d0,2*order*vin-d1);
  double k0=-(d0+(R*C/L)*G)/(4*L*C)*U*U;
  double k1=G/(4*L*C)*U*U;
  /* Clearing the commuting Pade denominators gives
   * d+p*u+q*w+k*u*w=0, u=Ton/U,w=Toff/U.
   * Eliminate w: (d0+p0*u)(q1+k1*u)
   *             -(d1+p1*u)(q0+k0*u)=0. */
  double c2=fma(p0,k1,-p1*k0);
  double c1=fma(d0,k1,fma(p0,q1,-fma(d1,k0,p1*q0)));
  double c0=fma(d0,q1,-d1*q0);
  if(!isfinite(p0)||!isfinite(p1)||!isfinite(q0)||!isfinite(k0)||
     !isfinite(k1)||!isfinite(c2)||!isfinite(c1)||!isfinite(c0))continue;
  double scale=fmax(fabs(c2),fmax(fabs(c1),fabs(c0)));
  if(!(scale>0)||!isfinite(scale))continue;c2/=scale;c1/=scale;c0/=scale;
  double roots[2];int count=0;
  if(c2==0){if(c1!=0)roots[count++]=-c0/c1;}
  else{
   double cross=4*c2*c0,disc=fma(c1,c1,-cross);
   double disc_roundoff=64*DBL_EPSILON*(c1*c1+fabs(cross));
   if(!isfinite(disc)||disc < -disc_roundoff)continue;
   if(disc<0)disc=0;
   double aux=-.5*(c1+copysign(sqrt(disc),c1));
   if(aux==0)roots[count++]=0;
   else{roots[count++]=aux/c2;roots[count++]=c0/aux;}
  }
  for(int j=0;j<count;++j){
   double u=roots[j];if(!isfinite(u))continue;
   double den0=fma(k0,u,q0),den1=fma(k1,u,q1);
   int row=fabs(den0)>=fabs(den1);double den=row?den0:den1;
   if(den==0||!isfinite(den))continue;
   double w=-fma(row?p0:p1,u,row?d0:d1)/den;
   if(!isfinite(w))continue;
   double roundoff=128*DBL_EPSILON*fmax(1,fmax(fabs(u),fabs(w)));
   if(fabs(u)<=roundoff)u=0;if(fabs(w)<=roundoff)w=0;
   toc_consider(&best,u*U,w*U,order,i0,v0,iload,target_i,target_v,vin,L,C,R);
  }
 }
 return best;
}
#endif






#ifndef TIMING_MATH_H
#define TIMING_MATH_H
/* Lightweight exact derivatives of the coupled first-order [1/1] Pade map.
 * State order: [inductor current (A), output voltage (V)].
 * Timing-Jacobian columns: [Ton (s), Toff (s)], regardless of pulse order.
 * No heap, finite differences, learned plant parameters or gain clamps.
 * Every function returns TM_OK on success and leaves outputs unchanged on error.
 */
#include <math.h>
#include <float.h>

typedef struct { double i, v; } TM_State;
typedef struct { double m00, m01, m10, m11; } TM_Mat;
typedef struct { double L, C, R, vin, load; } TM_Model;
enum { TM_OK=1, TM_BAD_INPUT=-1, TM_NONFINITE=-2, TM_SINGULAR=-3 };

static int tm_state_finite(TM_State x) {
 return isfinite(x.i)&&isfinite(x.v);
}
static int tm_mat_finite(TM_Mat p) {
 return isfinite(p.m00)&&isfinite(p.m01)&&isfinite(p.m10)&&isfinite(p.m11);
}
static int tm_model_valid(const TM_Model *p) {
 return p&&isfinite(p->L)&&p->L>0&&isfinite(p->C)&&p->C>0&&
  isfinite(p->R)&&p->R>=0&&isfinite(p->vin)&&isfinite(p->load);
}
/* Standard left multiplication, suitable for post-TOC sensitivity transport. */
static int tm_mat_vec(TM_Mat p, TM_State x, TM_State *out) {
 TM_State y;
 if(!out||!tm_mat_finite(p)||!tm_state_finite(x))return TM_BAD_INPUT;
 y.i=fma(p.m00,x.i,p.m01*x.v);y.v=fma(p.m10,x.i,p.m11*x.v);
 if(!tm_state_finite(y))return TM_NONFINITE;*out=y;return TM_OK;
}
/* x1=F_t(x0), optional phi=dF_t/dx0 and q=dF_t/dt.
 * F_t=(I-tA/2)^-1[(I+tA/2)x0+t*b_gate],
 * q=(I-tA/2)^-1[A*(x0+x1)/2+b_gate].
 * L/C/R/Vin/load are constant over this actual ON or OFF interval.
 */
static int tm_interval(const TM_Model *p, TM_State x0, double t, int gate,
 TM_State *x1, TM_Mat *phi, TM_State *q) {
 TM_State z,dq;TM_Mat pp;
 double k,l,alpha,kl,det,ri,rv;
 if(!x1||!tm_model_valid(p)||!tm_state_finite(x0)||!isfinite(t)||t<0||
  (gate!=0&&gate!=1))return TM_BAD_INPUT;
 k=(t/2)/p->L;l=(t/2)/p->C;alpha=p->R*k;kl=k*l;
 det=1+alpha+kl;
 if(!isfinite(k)||!isfinite(l)||!isfinite(alpha)||!isfinite(kl)||
  !isfinite(det))return TM_NONFINITE;
 if(!(det>0))return TM_SINGULAR;
 ri=fma(1-alpha,x0.i,-k*x0.v)+2*k*p->vin*gate;
 rv=fma(l,x0.i,x0.v)-2*l*p->load;
 z.i=(ri-k*rv)/det;z.v=(l*ri+(1+alpha)*rv)/det;
 if(!tm_state_finite(z))return TM_NONFINITE;
 if(phi){
  pp.m00=(1-alpha-kl)/det;pp.m01=-2*k/det;
  pp.m10=2*l/det;pp.m11=(1+alpha-kl)/det;
  if(!tm_mat_finite(pp))return TM_NONFINITE;
 }
 if(q){
  double mi=x0.i/2+z.i/2,mv=x0.v/2+z.v/2;
  double fi=(fma(-p->R,mi,p->vin*gate-mv))/p->L;
  double fv=(mi-p->load)/p->C;
  dq.i=(fi-k*fv)/det;dq.v=(l*fi+(1+alpha)*fv)/det;
  if(!tm_state_finite(dq))return TM_NONFINITE;
 }
 *x1=z;if(phi)*phi=pp;if(q)*q=dq;return TM_OK;
}
/* Two-pulse endpoint and exact Ton/Toff sensitivity. order=+1:ON-OFF,
 * order=-1:OFF-ON. Zero durations are allowed (their derivative remains valid).
 */
static int tm_endpoint(const TM_Model *p, TM_State x0,
 double ton,double toff,int order,TM_State *xend,TM_Mat *jt) {
 TM_State middle,end,q1,q2,qprop;TM_Mat phi2,j;int rc;
 if(!xend||!jt||(order!=1&&order!=-1)||!isfinite(ton)||ton<0||
  !isfinite(toff)||toff<0)return TM_BAD_INPUT;
 if(order==1){
  rc=tm_interval(p,x0,ton,1,&middle,0,&q1);if(rc!=TM_OK)return rc;
  rc=tm_interval(p,middle,toff,0,&end,&phi2,&q2);if(rc!=TM_OK)return rc;
  rc=tm_mat_vec(phi2,q1,&qprop);if(rc!=TM_OK)return rc;
  j.m00=qprop.i;j.m10=qprop.v;j.m01=q2.i;j.m11=q2.v;
 }else{
  rc=tm_interval(p,x0,toff,0,&middle,0,&q1);if(rc!=TM_OK)return rc;
  rc=tm_interval(p,middle,ton,1,&end,&phi2,&q2);if(rc!=TM_OK)return rc;
  rc=tm_mat_vec(phi2,q1,&qprop);if(rc!=TM_OK)return rc;
  j.m00=q2.i;j.m10=q2.v;j.m01=qprop.i;j.m11=qprop.v;
 }
 if(!tm_mat_finite(j))return TM_NONFINITE;*xend=end;*jt=j;return TM_OK;
}
/* Chain rule for physical durations t=diag(g)*t_nom. */
static int tm_gain_jacobian(TM_Mat jt,double ton_nom,double toff_nom,TM_Mat *jg){
 TM_Mat j;
 if(!jg||!tm_mat_finite(jt)||!isfinite(ton_nom)||ton_nom<0||
  !isfinite(toff_nom)||toff_nom<0)return TM_BAD_INPUT;
 j.m00=jt.m00*ton_nom;j.m10=jt.m10*ton_nom;
 j.m01=jt.m01*toff_nom;j.m11=jt.m11*toff_nom;
 if(!tm_mat_finite(j))return TM_NONFINITE;*jg=j;return TM_OK;
}
/* Damped noise-weighted Gauss-Newton increment:
 * dg=-eta*(Jg' W Jg+lambda*I)^-1*Jg' W residual,
 * W=diag(1/sigma_i^2,1/sigma_v^2).
 * dg.i=delta_gon, dg.v=delta_goff (these are gains, not state units).
 * sigma values are positive residual-noise scales in A/V; lambda>=0,
 * eta>=0. No positivity/bound clamp is imposed on the caller's gains.
 * Normalization below preserves the objective and prevents avoidable overflow.
 */
static int tm_gain_step(TM_Mat j,TM_State residual,double sigma_i,double sigma_v,
 double lambda,double eta,TM_State *dg){
 double a,b,c,d,e,f,s,root,reg,h00,h01,h11,z0,z1,cross,det,det_scale;
 TM_State step;
 if(!dg||!tm_mat_finite(j)||!tm_state_finite(residual)||
  !isfinite(sigma_i)||sigma_i<=0||!isfinite(sigma_v)||sigma_v<=0||
  !isfinite(lambda)||lambda<0||!isfinite(eta)||eta<0)return TM_BAD_INPUT;
 a=j.m00/sigma_i;b=j.m01/sigma_i;c=j.m10/sigma_v;d=j.m11/sigma_v;
 e=residual.i/sigma_i;f=residual.v/sigma_v;root=sqrt(lambda);
 if(!isfinite(a)||!isfinite(b)||!isfinite(c)||!isfinite(d)||
  !isfinite(e)||!isfinite(f))return TM_NONFINITE;
 s=fmax(root,fmax(fmax(fabs(a),fabs(b)),fmax(fabs(c),fabs(d))));
 if(!(s>0))return TM_SINGULAR;
 a/=s;b/=s;c/=s;d/=s;e/=s;f/=s;reg=(root/s)*(root/s);
 if(!isfinite(e)||!isfinite(f))return TM_NONFINITE;
 h00=fma(a,a,c*c)+reg;h01=fma(a,b,c*d);h11=fma(b,b,d*d)+reg;
 z0=fma(a,e,c*f);z1=fma(b,e,d*f);
 cross=fma(a,d,-b*c);
 /* Expanded determinant avoids subtracting nearly equal Gram products. */
 det=cross*cross+reg*(a*a+b*b+c*c+d*d)+reg*reg;
 det_scale=h00*h11+h01*h01;
 if(!isfinite(det)||!isfinite(z0)||!isfinite(z1))return TM_NONFINITE;
 if(!(det>64*DBL_EPSILON*det_scale))return TM_SINGULAR;
 step.i=-eta*fma(h11,z0,-h01*z1)/det;
 step.v=-eta*fma(h00,z1,-h01*z0)/det;
 if(!tm_state_finite(step))return TM_NONFINITE;*dg=step;return TM_OK;
}
#endif



#include <math.h>
#include <float.h>
typedef struct {
 double L,C,R,vin,load,kvp,kvi,kip,kii,dmin,dmax;
} AP_Model;
typedef struct { double i,v; } AP_State;
typedef struct { double a,b,c,d; } AP_Mat;
typedef struct { double i,v,si,sv,q,sq; } AP_Tan;
enum { AP_OK=1,AP_BAD_INPUT=-1,AP_SINGULAR=-2,AP_NONFINITE=-3,
 AP_ZERO_NOMINAL=-4 };

static int ap_valid(const AP_Model*m){
 return m&&isfinite(m->L)&&m->L>0&&isfinite(m->C)&&m->C>0&&
 isfinite(m->R)&&m->R>=0&&isfinite(m->vin)&&isfinite(m->load)&&
 isfinite(m->kvp)&&isfinite(m->kvi)&&isfinite(m->kip)&&isfinite(m->kii)&&
 isfinite(m->dmin)&&isfinite(m->dmax)&&m->dmin>=0&&m->dmax<=1&&m->dmin<m->dmax;
}
static int ap_finite_state(AP_State x){return isfinite(x.i)&&isfinite(x.v);}
static int ap_interval(const AP_Model*m,AP_State x,double t,int gate,
 AP_State*out,AP_Mat*phi,AP_State*qt){
 if(!out||!ap_valid(m)||!ap_finite_state(x)||!isfinite(t)||t<0||(gate!=0&&gate!=1))return AP_BAD_INPUT;
 double k=.5*t/m->L,l=.5*t/m->C,alpha=m->R*k,kl=k*l,det=1+alpha+kl;
 if(!(det>0)||!isfinite(det))return AP_NONFINITE;
 double ri=(1-alpha)*x.i-k*x.v+2*k*m->vin*gate;
 double rv=l*x.i+x.v-2*l*m->load;
 AP_State z={(ri-k*rv)/det,(l*ri+(1+alpha)*rv)/det};
 if(!ap_finite_state(z))return AP_NONFINITE;
 if(phi){AP_Mat p={(1-alpha-kl)/det,-2*k/det,2*l/det,(1+alpha-kl)/det};
  if(!isfinite(p.a)||!isfinite(p.b)||!isfinite(p.c)||!isfinite(p.d))return AP_NONFINITE;*phi=p;}
 if(qt){double mi=.5*(x.i+z.i),mv=.5*(x.v+z.v);
  double fi=(m->vin*gate-mv-m->R*mi)/m->L,fv=(mi-m->load)/m->C;
  AP_State q={(fi-k*fv)/det,(l*fi+(1+alpha)*fv)/det};
  if(!ap_finite_state(q))return AP_NONFINITE;*qt=q;}
 *out=z;return AP_OK;
}
static int ap_endpoint_j(const AP_Model*m,AP_State x,double on,double off,
 int order,AP_Mat*j){
 if(!j||(order!=1&&order!=-1))return AP_BAD_INPUT;
 AP_State mid,end,q1,q2;AP_Mat p;int rc;
 if(order>0){
  rc=ap_interval(m,x,on,1,&mid,0,&q1);if(rc<0)return rc;
  rc=ap_interval(m,mid,off,0,&end,&p,&q2);if(rc<0)return rc;
  AP_Mat z={p.a*q1.i+p.b*q1.v,q2.i,p.c*q1.i+p.d*q1.v,q2.v};*j=z;
 }else{
  rc=ap_interval(m,x,off,0,&mid,0,&q1);if(rc<0)return rc;
  rc=ap_interval(m,mid,on,1,&end,&p,&q2);if(rc<0)return rc;
  AP_Mat z={q2.i,p.a*q1.i+p.b*q1.v,q2.v,p.c*q1.i+p.d*q1.v};*j=z;
 }
 return isfinite(j->a)&&isfinite(j->b)&&isfinite(j->c)&&isfinite(j->d)?AP_OK:AP_NONFINITE;
}
static int ap_span(const AP_Model*m,AP_Tan*x,double t,int gate,double st){
 if(!x||!isfinite(st))return AP_BAD_INPUT;
 AP_State old={x->i,x->v},z,qt;AP_Mat p;
 int rc=ap_interval(m,old,t,gate,&z,&p,&qt);if(rc<0)return rc;
 double nsi=p.a*x->si+p.b*x->sv+qt.i*st;
 double nsv=p.c*x->si+p.d*x->sv+qt.v*st;
 double nsq=x->sq+.5*st*(x->i+z.i)+.5*t*(x->si+nsi);
 double nq=x->q+.5*t*(x->i+z.i);
 if(!isfinite(nsi)||!isfinite(nsv)||!isfinite(nsq)||!isfinite(nq))return AP_NONFINITE;
 x->i=z.i;x->v=z.v;x->si=nsi;x->sv=nsv;x->q=nq;x->sq=nsq;return AP_OK;
}

/* q is an ADDITIVE signed current target offset (A), not a multiplier.
 * Inputs:
 * - held launch i0,v0 and independent held physical-model Ieq=m->load;
 * - nominal closed-form Ton/Toff, actual G-scaled Ton/Toff and chosen order;
 * - measured/corrected endpoint iE,vE and first normal rise iA,vA;
 * - actual completion-to-A OFF tail, actual A->B period and ON interval;
 * - IavgA measured over the last actual TOC rise -> A, required for OFF-ON.
 * Actual durations include 1ns scheduling; derivatives remain continuous-model
 * derivatives. Ratios actual/nominal hold the existing timing gains fixed.
 * A zero nominal span has no recoverable ratio here; return AP_ZERO_NOMINAL.
 * Optional se_i,se_v are the executed TOC-end target sensitivities before
 * the fixed nominal measurement anchoring. Outputs change only on success.
 */
static int ap_phase_jacobian(const AP_Model*m,int order,
 double i0,double v0,double ton_nom,double toff_nom,
 double ton_actual,double toff_actual,double iE,double vE,
 double tail,double T,double on1,double iavgA,double iA,double vA,
 double*Jphi,double*se_i,double*se_v){
 if(!Jphi||!ap_valid(m)||(order!=1&&order!=-1)||!isfinite(i0)||!isfinite(v0)||
 !isfinite(ton_nom)||!isfinite(toff_nom)||ton_nom<0||toff_nom<0||
 !isfinite(ton_actual)||!isfinite(toff_actual)||ton_actual<0||toff_actual<0||
 !isfinite(iE)||!isfinite(vE)||!isfinite(tail)||tail<0||!isfinite(T)||!(T>0)||
 !isfinite(on1)||on1<0||on1>T||!isfinite(iavgA)||!isfinite(iA)||!isfinite(vA))return AP_BAD_INPUT;
 if(!(ton_nom>0&&toff_nom>0))return AP_ZERO_NOMINAL;
 if((order>0&&tail>1.01e-9)||(order<0&&tail>T+1.01e-9))return AP_BAD_INPUT;
 AP_State initial={i0,v0};AP_Mat j;int rc=ap_endpoint_j(m,initial,ton_nom,toff_nom,order,&j);
 if(rc<0)return rc;
 double det=j.a*j.d-j.b*j.c,scale=fabs(j.a*j.d)+fabs(j.b*j.c);
 if(!isfinite(det)||!isfinite(scale))return AP_NONFINITE;
 if(!(fabs(det)>64*DBL_EPSILON*scale))return AP_SINGULAR;
 /* Implicit derivative of endpoint=[Ieq+q,Vref]: Jnom*dtnom/dq=[1,0]. */
 double sn_on=j.d/det,sn_off=-j.c/det;
 double st_on=(ton_actual/ton_nom)*sn_on,st_off=(toff_actual/toff_nom)*sn_off;
 if(!isfinite(st_on)||!isfinite(st_off))return AP_NONFINITE;
 AP_Tan x={0};x.i=i0;x.v=v0;
 if(order>0){
  rc=ap_span(m,&x,ton_actual,1,st_on);if(rc<0)return rc;
  rc=ap_span(m,&x,toff_actual,0,st_off);if(rc<0)return rc;
 }else{
  rc=ap_span(m,&x,toff_actual,0,st_off);if(rc<0)return rc;
  /* Current-average sensor starts its new interval at this TOC rising edge. */
  x.q=0;x.sq=0;
  rc=ap_span(m,&x,ton_actual,1,st_on);if(rc<0)return rc;
 }
 double end_si=x.si,end_sv=x.sv;
 /* Hold the nominal endpoint measurement correction fixed in this derivative. */
 x.i=iE;x.v=vE;
 double du=0;
 if(order<0){
  double window=ton_actual+tail;
  if(!(window>0)||!isfinite(window))return AP_BAD_INPUT;
  rc=ap_span(m,&x,tail,0,0);if(rc<0)return rc;
  /* A current integral gets a fixed nominal anchor to the measured average.
   * The moving last-ON duration remains in the denominator derivative. */
  double savgA=(x.sq-iavgA*st_on)/window;
  x.i=iA;x.v=vA;
  /* At OFF-ON A, normal PI replaces the DONE P=0 state. Both integral
   * preloads were independent of q; ki is per event, not multiplied by T. */
  double sref=-(m->kvp+m->kvi)*x.sv,se=sref-savgA;
  du=(m->kip+m->kii)*se;
 }else{x.i=iA;x.v=vA;}
 /* Existing physical duty bounds: derivative is zero on saturated branch.
  * 1ns/period tolerance identifies only the active set, not a new clamp. */
 double observed=on1/T,tol=1e-9/T+64*DBL_EPSILON;
 if(observed<=m->dmin+tol||observed>=m->dmax-tol)du=0;
 if(!isfinite(du))return AP_NONFINITE;
 x.q=0;x.sq=0;
 rc=ap_span(m,&x,on1,1,T*du);if(rc<0)return rc;
 rc=ap_span(m,&x,T-on1,0,-T*du);if(rc<0)return rc;
 double ans=x.sq/T;if(!isfinite(ans))return AP_NONFINITE;
 *Jphi=ans;if(se_i)*se_i=end_si;if(se_v)*se_v=end_sv;return AP_OK;
}

/* OFF-ON phase teaching BEFORE the first normal PI reaction.
 * Residual e_tail=C*(v_A-v_DONE)/(A-DONE) is capacitor mean current over
 * the first OFF tail. q changes the selected peak target only; corrected
 * load, timing gains and DONE Dff are held fixed in this local derivative.
 * With DONE P=0, the tail length is independent of q. The exact continuous
 * configured-map sensitivity is C/tail*(sv_A-sv_DONE). The capacitor row of
 * [1/1] Pade makes this IDENTICALLY .5*(si_DONE+si_A); the latter avoids
 * cancellation for short positive tails. It is not a J=1 approximation.
 * No post-A PI/current-average window appears in this teaching residual.
 */
static int ap_phase_tail_jacobian(const AP_Model*m,int order,
 double i0,double v0,double ton_nom,double toff_nom,
 double ton_actual,double toff_actual,double iE,double vE,double tail,
 double*Jphi){
 if(!Jphi||!ap_valid(m)||order!=-1||!isfinite(i0)||!isfinite(v0)||
 !isfinite(ton_nom)||!isfinite(toff_nom)||ton_nom<0||toff_nom<0||
 !isfinite(ton_actual)||!isfinite(toff_actual)||ton_actual<0||toff_actual<0||
 !isfinite(iE)||!isfinite(vE)||!isfinite(tail)||!(tail>0))return AP_BAD_INPUT;
 if(!(ton_nom>0&&toff_nom>0))return AP_ZERO_NOMINAL;
 AP_State initial={i0,v0};AP_Mat j;
 int rc=ap_endpoint_j(m,initial,ton_nom,toff_nom,order,&j);if(rc<0)return rc;
 double det=j.a*j.d-j.b*j.c,scale=fabs(j.a*j.d)+fabs(j.b*j.c);
 if(!isfinite(det)||!isfinite(scale))return AP_NONFINITE;
 if(!(fabs(det)>64*DBL_EPSILON*scale))return AP_SINGULAR;
 double st_on=(ton_actual/ton_nom)*(j.d/det);
 double st_off=(toff_actual/toff_nom)*(-j.c/det);
 if(!isfinite(st_on)||!isfinite(st_off))return AP_NONFINITE;
 AP_Tan x={0};x.i=i0;x.v=v0;
 rc=ap_span(m,&x,toff_actual,0,st_off);if(rc<0)return rc;
 rc=ap_span(m,&x,ton_actual,1,st_on);if(rc<0)return rc;
 double end_si=x.si;
 x.i=iE;x.v=vE; /* Same fixed nominal endpoint anchoring as the full helper. */
 rc=ap_span(m,&x,tail,0,0);if(rc<0)return rc;
 double ans=.5*(end_si+x.si);
 if(!isfinite(ans))return AP_NONFINITE;*Jphi=ans;return AP_OK;
}

/* Derivative in V/A of e_v = v_S-v_DONE with respect to the selected
 * additive signed q target. S=A for OFF-ON; S=B for ON-OFF.
 *
 * Causal intervals only: OFF-ON DONE->A is an OFF tail with no PI reaction.
 * ON-OFF DONE->A is the existing <=1.01ns HIGH alignment tail, followed by
 * the observed full normal A->B ON/OFF cycle. All post-DONE durations are
 * held fixed: the normal PWM epoch is restarted at DONE and DONE uses P=0,
 * Dff, and equilibrium-current preloads independent of q. The first normal
 * PI reaction at S cannot affect the already observed drift.
 *
 * Endpoint and A measurement-correction offsets are held fixed, including
 * their voltage offsets. Differentiating the capacitor row therefore gives
 * d(v_S-v_DONE)/dq = d integral_D^S i_L dt /dq / C. Integral sensitivities
 * are reset at DONE; their stable sum avoids subtracting nearly equal
 * endpoint voltage tangents. No nominal-unit sensitivity is substituted.
 *
 * ton/toff nominal derivative is implicit J_endpoint^-1*[1,0]; the applied
 * duration tangent holds G_on=actual_on/nominal_on and G_off likewise fixed.
 * This is an exact derivative of the configured causal [1/1] Padé map on
 * its fixed branch, not an identified plant derivative. It does not include
 * observer innovation, ADC noise, quantization or adaptive-load derivatives.
 * T and on1 may be zero for OFF-ON, where no complete PWM cycle is needed.
 * Outputs remain unchanged on failure.
 */
static int ap_phase_voltage_jacobian(const AP_Model*m,int order,
 double i0,double v0,double ton_nom,double toff_nom,
 double ton_actual,double toff_actual,double iE,double vE,
 double tail,double T,double on1,double iA,double vA,
 double*Jv,double*se_i,double*se_v){
 if(!Jv||!ap_valid(m)||(order!=1&&order!=-1)||!isfinite(i0)||!isfinite(v0)||
 !isfinite(ton_nom)||!isfinite(toff_nom)||ton_nom<0||toff_nom<0||
 !isfinite(ton_actual)||!isfinite(toff_actual)||ton_actual<0||toff_actual<0||
 !isfinite(iE)||!isfinite(vE)||!isfinite(tail)||tail<0||
 !isfinite(T)||!isfinite(on1)||!isfinite(iA)||!isfinite(vA))return AP_BAD_INPUT;
 if(!(ton_nom>0&&toff_nom>0))return AP_ZERO_NOMINAL;
 if(order<0){if(!(tail>0))return AP_BAD_INPUT;}
 else if(tail>1.01e-9||!(T>0)||on1<0||on1>T)return AP_BAD_INPUT;
 AP_State initial={i0,v0};AP_Mat j;
 int rc=ap_endpoint_j(m,initial,ton_nom,toff_nom,order,&j);if(rc<0)return rc;
 double det=j.a*j.d-j.b*j.c,scale=fabs(j.a*j.d)+fabs(j.b*j.c);
 if(!isfinite(det)||!isfinite(scale))return AP_NONFINITE;
 if(!(fabs(det)>64*DBL_EPSILON*scale))return AP_SINGULAR;
 double st_on=(ton_actual/ton_nom)*(j.d/det);
 double st_off=(toff_actual/toff_nom)*(-j.c/det);
 if(!isfinite(st_on)||!isfinite(st_off))return AP_NONFINITE;
 AP_Tan x={0};x.i=i0;x.v=v0;
 if(order>0){
  rc=ap_span(m,&x,ton_actual,1,st_on);if(rc<0)return rc;
  rc=ap_span(m,&x,toff_actual,0,st_off);if(rc<0)return rc;
 }else{
  rc=ap_span(m,&x,toff_actual,0,st_off);if(rc<0)return rc;
  rc=ap_span(m,&x,ton_actual,1,st_on);if(rc<0)return rc;
 }
 double end_si=x.si,end_sv=x.sv;
 x.i=iE;x.v=vE;x.q=0;x.sq=0;
 if(order<0){
  rc=ap_span(m,&x,tail,0,0);if(rc<0)return rc;
 }else{
  rc=ap_span(m,&x,tail,1,0);if(rc<0)return rc;
  x.i=iA;x.v=vA; /* Preserve state tangents: correction offsets held. */
  rc=ap_span(m,&x,on1,1,0);if(rc<0)return rc;
  rc=ap_span(m,&x,T-on1,0,0);if(rc<0)return rc;
 }
 double ans=x.sq/m->C;if(!isfinite(ans))return AP_NONFINITE;
 *Jv=ans;if(se_i)*se_i=end_si;if(se_v)*se_v=end_sv;return AP_OK;
}


#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

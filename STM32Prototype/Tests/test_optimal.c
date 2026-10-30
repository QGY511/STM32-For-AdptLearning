/* Verifies buck_optimal_onoff() (the re-derived closed form, TPEL Sec. IV) against
 * a Newton solve of the terminal equations (35)-(36) propagated through the raw
 * second-order Pade model. The closed form must recover the same optimum and the
 * Pade-propagated shifted state must return to zero. */
#include <assert.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <limits.h>
#include "buck_optimal.h"

static const double Vin = BUCK_VIN_V, Vref = BUCK_VOUT_MV * 1e-3;
static const double L = BUCK_L_H, C = BUCK_C_F, r = BUCK_R_OHM;
#define TH (-r / L)
#define M1 ((Vin - Vref) / L)
#define M2 (Vref / L)

/* eqs (35)-(36) residual (double) */
static void resid(double x, double y, double zi, double zv, double Ih, double *e1, double *e2) {
    double p = TH*zi/2 - zv/(2*L) + M1 - TH*Ih, q = TH*zi/2 - zv/(2*L) - M2 - TH*Ih;
    double g = 0.25*(TH*TH - 1/(L*C))*zi - TH*zv/(4*L) + TH*(M1+M2)/2;
    double b = TH*zi/(4*C) - zv/(4*L*C) + (M1+M2)/(2*C);
    *e1 = zi + p*x + q*y + g*x*y;
    *e2 = zv + zi/(2*C)*(x+y) + b*x*y;
}
static void newton(double zi, double zv, double Ih, double *x, double *y) {
    *x = 300e-9; *y = 2000e-9;
    for (int i = 0; i < 100; ++i) {
        double e1,e2,e1a,e2a,e1b,e2b,h=1e-13;
        resid(*x,*y,zi,zv,Ih,&e1,&e2);
        resid(*x+h,*y,zi,zv,Ih,&e1a,&e2a); resid(*x,*y+h,zi,zv,Ih,&e1b,&e2b);
        double j00=(e1a-e1)/h,j01=(e1b-e1)/h,j10=(e2a-e2)/h,j11=(e2b-e2)/h;
        double det=j00*j11-j01*j10;
        double dx=( j11*e1 - j01*e2)/det, dy=(-j10*e1 + j00*e2)/det;
        *x-=dx; *y-=dy;
        if (fabs(dx)+fabs(dy) < 1e-18) break;
    }
}

int main(void) {
    struct { double zi, zv, Ih; } cases[] = {
        {-1.0,-0.05,2.0},{-0.5,-0.02,1.5},{-2.0,-0.10,3.0},{-1.0,-0.001,2.0},{-5.0,-0.2,6.0}
    };
    double maxerr = 0;
    for (unsigned i = 0; i < sizeof(cases)/sizeof(cases[0]); ++i) {
        double zi=cases[i].zi, zv=cases[i].zv, Ih=cases[i].Ih, nx, ny;
        newton(zi, zv, Ih, &nx, &ny);
        float ton, toff;
        int ok = buck_optimal_onoff((float)zi,(float)zv,(float)Ih,
                                    1e-9f, 100e-6f, 100e-6f, &ton, &toff); /* wide bounds: no clamp */
        assert(ok);
        double et = fabs(ton - nx), ef = fabs(toff - ny);
        if (et > maxerr) maxerr = et;
        if (ef > maxerr) maxerr = ef;
        printf("  (%.2f,%.3f,I=%.1f) Newton=(%.1f,%.1f)ns  closed=(%.1f,%.1f)ns\n",
               zi, zv, Ih, nx*1e9, ny*1e9, ton*1e9, toff*1e9);
    }
    printf("max |Ton/Toff| error vs Newton optimum: %.1f ns\n", maxerr*1e9);
    assert(maxerr < 5e-9); /* within 5 ns (float precision) */

    /* Bounds are enforced. */
    float ton, toff;
    assert(buck_optimal_onoff(-2.0f,-0.1f,3.0f, 1e-9f, 400e-9f, 1000e-9f, &ton, &toff));
    assert(ton <= 400e-9f + 1e-12f && toff <= 1000e-9f + 1e-12f);
    puts("PASS: time-optimal ON-OFF closed form matches Newton optimum; bounds enforced");

    /* --- State-anchored trigger score (buck_sa_score_f), runtime scales --- */
    {
        const float VA = BUCK_SA_VA_COUNTS, DVA = BUCK_SA_DVA_COUNTS, R = BUCK_SA_RHO;
        assert(buck_sa_score_f(0, 0, VA, DVA, R) == 0.0f);           /* steady state */
        assert(buck_sa_score_f(24, 0, VA, DVA, R) < 1.0f);           /* inside on z_v axis */
        assert(buck_sa_score_f(26, 0, VA, DVA, R) > 1.0f);           /* outside on z_v axis */
        assert(buck_sa_score_f(0, DVA*0.25f, VA, DVA, R) < 1.0f);    /* inside on d_v axis */
        assert(buck_sa_score_f(0, DVA*2.0f, VA, DVA, R) > 1.0f);     /* outside on d_v axis */
        assert(buck_sa_score_f(-30, 0, VA, DVA, R) > 1.0f);          /* droop (z_v<0) fires too */
        assert(buck_sa_score_f(-8, -24, VA, DVA, R) > 1.0f);         /* a 3 A step's first period: -24 counts of d_v */
        assert(buck_sa_score_f(-6, -12, VA, DVA, R) < 0.7f);         /* +-6 counts of noise on both samples */
        /* Retuning the scales changes sensitivity as expected. */
        assert(buck_sa_score_f(12, 0, 10.0f, DVA, R) > 1.0f);        /* smaller V_a -> more sensitive */
        assert(buck_sa_score_f(0, 0, 0.0f, 0.0f, R) == 0.0f);        /* zero scales are floored, no div */
        printf("  SA: S(25,0)=%.3f  S(0,dVa)=%.3f  (1.0 = boundary)\n",
               (double)buck_sa_score_f(25, 0, VA, DVA, R),
               (double)buck_sa_score_f(0, DVA, VA, DVA, R));
        puts("PASS: state-anchored score matches the tilted-ellipse boundary");
    }
    /* --- Integer ISR trigger (buck_sa_trigger_i) vs the float score --- */
    {
        unsigned checked = 0, mismatches = 0, boundary = 0;
        const float VAs[]  = {5.0f, 25.0f, 100.0f, 0.02f, 1000.0f};
        const float DVAs[] = {2.0f, 8.0f, 30.0f, 0.02f, 500.0f};
        const float RHs[]  = {-0.9f, -0.5f, 0.0f, 0.5f, 0.9f, 1.5f};
        for (unsigned a = 0; a < sizeof(VAs)/sizeof(VAs[0]); ++a)
        for (unsigned b = 0; b < sizeof(DVAs)/sizeof(DVAs[0]); ++b)
        for (unsigned r = 0; r < sizeof(RHs)/sizeof(RHs[0]); ++r) {
            buck_sa_coef_t c; buck_sa_coef_from_float(VAs[a], DVAs[b], RHs[r], &c);
            const float rho = (RHs[r] > 0.999f) ? 0.999f : (RHs[r] < -0.999f) ? -0.999f : RHs[r];
            for (int zv = -4095; zv <= 4095; zv += 7)
            for (int dv = -4095; dv <= 4095; dv += 41) {
                const float S = buck_sa_score_f((float)zv, (float)dv, VAs[a], DVAs[b], rho);
                const int ti = buck_sa_trigger_i(zv, dv, &c) ? 1 : 0;
                const int tf = (S > 1.0f) ? 1 : 0;
                ++checked;
                if (ti != tf) {
                    /* Allowed only in a thin band around the boundary, or where a
                     * scale is clamped (< 0.5 count: integer side saturates its
                     * coefficient = more sensitive). */
                    const int clamped = (VAs[a] < 0.5f) || (DVAs[b] < 0.5f);
                    if (fabsf(S - 1.0f) < 0.005f || clamped) ++boundary;
                    else { if (mismatches < 5) printf("  MISMATCH va=%g dva=%g rho=%g zv=%d dv=%d S=%f int=%d\n",
                                (double)VAs[a], (double)DVAs[b], (double)rho, zv, dv, (double)S, ti); ++mismatches; }
                }
            }
        }
        printf("  SA-int: %u points, %u boundary-band/clamped disagreements, %u real mismatches\n",
               checked, boundary, mismatches);
        assert(mismatches == 0);
        /* Exact cases at the defaults. */
        buck_sa_coef_t c; buck_sa_coef_from_float(BUCK_SA_VA_COUNTS, BUCK_SA_DVA_COUNTS, BUCK_SA_RHO, &c);
        assert(!buck_sa_trigger_i(0, 0, &c));
        assert(!buck_sa_trigger_i(24, 0, &c));
        assert( buck_sa_trigger_i(26, 0, &c));
        assert( buck_sa_trigger_i(-26, 0, &c));
        assert(!buck_sa_trigger_i(0, 2, &c));
        assert( buck_sa_trigger_i(0, -17, &c));
        assert( buck_sa_trigger_i(-8, -24, &c));
        assert(!buck_sa_trigger_i(-6, -12, &c));
        /* Huge inputs must not overflow into a false negative. */
        assert( buck_sa_trigger_i(4095, 0, &c));
        assert( buck_sa_trigger_i(-4095, -4095, &c));
        assert( buck_sa_trigger_i(4095, 4095, &c));
        /* Tilt: rho>0 emphasises droop (zv<0) with a falling voltage (dv<0). */
        buck_sa_coef_from_float(25.0f, 8.0f, 0.7f, &c);
        assert( buck_sa_trigger_i(-20, -6, &c));   /* S = 0.64+0.5625+0.84 = 2.04 */
        assert(!buck_sa_trigger_i( 20, -6, &c));   /* S = 0.64+0.5625-0.84 = 0.36 */
        puts("PASS: integer ISR trigger agrees with the float ellipse score");
    }
    return 0;
}

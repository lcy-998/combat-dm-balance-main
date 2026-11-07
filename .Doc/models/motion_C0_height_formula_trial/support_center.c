#include <math.h>
#include <stddef.h>
#include "support_center.h"
/* Global C0 quintic fit. Source six anchors are offline fitting evidence,
   not a runtime lookup table. x=(command_height-.35)/.05. */
int motion31_support_center(double h,double out[4])
{
    if(out==NULL || !isfinite(h) || h<.30-1e-7 || h>.40+1e-7) return -1;
    const double x=(h-.35)/.05;
    const double L=h+.000024-.060;
    const double beta=acos((L*L+.21*.21-.25*.25)/(2*L*.21));
    const double bh=(((((-0.04464695144541761*x+(-0.0094959546615917666))*x+(0.070546704606128441))*x+(0.012914498339764401))*x+(-0.022292895866604146))*x+(-0.34856693858410187));
    const double ba=(((((0.016045303181803701*x+(0.0061990859752017189))*x+(-0.026839290520086146))*x+(-0.0084307569262746084))*x+(0.027428468197591423))*x+(-0.3586744187278344));
    const double hip=beta-47.1033888*(3.14159265358979323846/180.)+bh;
    const double active=beta-47.1080837*(3.14159265358979323846/180.)+ba;
    out[0]=hip;out[1]=active;out[2]=-hip;out[3]=active;
    return 0;
}

#include <math.h>
#include "actor_runtime.h"
static const double H[6]={0.29999999999999999,0.32000000000000001,0.34000000000000002,0.35999999999999999,0.38,0.40000000000000002};
static const double BH[6]={-0.34875525220003523,-0.3435389986364944,-0.34415705961703547,-0.35197404473890415,-0.34675779117536332,-0.34154153761182243};
static const double BA[6]={-0.37754057053821566,-0.37281356662054022,-0.36427784427970067,-0.35372561665494917,-0.34899861273727373,-0.34427160881959828};

int motion31_support_center(double h,double out[4]){
 /*1e-7m accommodates only FP32 command-boundary rounding.*/
 if(!isfinite(h)||h<.30-1e-7||h>.40+1e-7)return -1;
 int i=0;while(i<4 && h>H[i+1])i++;
 double a=fmax(0.,fmin(1.,(h-H[i])/(H[i+1]-H[i])));
 double L=h+.000024-.060,beta=acos((L*L+.21*.21-.25*.25)/(2*L*.21));
 double hip=beta-47.1033888*(3.14159265358979323846/180.)+BH[i]+a*(BH[i+1]-BH[i]);
 double active=beta-47.1080837*(3.14159265358979323846/180.)+BA[i]+a*(BA[i+1]-BA[i]);
 out[0]=hip;out[1]=active;out[2]=-hip;out[3]=active;return 0;
}

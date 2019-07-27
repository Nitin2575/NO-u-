/*vex-vision-config:begin*/
#include "v5.h"
#include "v5_vcs.h"
vex::vision::signature O = vex::vision::signature(1, 8053, 9575, 8814, -2575, -1589, -2082, 8.8, 0);
vex::vision::signature G = vex::vision::signature(2, -6879, -5607, -6243, -3957, -2283, -3120, 5.5, 0);
vex::vision::signature P = vex::vision::signature(3, 1135, 2121, 1628, 7883, 9603, 8743, 6.8, 0);
vex::vision::signature SIG_4 = vex::vision::signature(4, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_5 = vex::vision::signature(5, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_6 = vex::vision::signature(6, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_7 = vex::vision::signature(7, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision VisionCamera = vex::vision(vex::PORT8, 18, O, G, P, SIG_4, SIG_5, SIG_6, SIG_7);
/*vex-vision-config:end*/
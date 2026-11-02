#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_2670852950093832942) {
   out_2670852950093832942[0] = delta_x[0] + nom_x[0];
   out_2670852950093832942[1] = delta_x[1] + nom_x[1];
   out_2670852950093832942[2] = delta_x[2] + nom_x[2];
   out_2670852950093832942[3] = delta_x[3] + nom_x[3];
   out_2670852950093832942[4] = delta_x[4] + nom_x[4];
   out_2670852950093832942[5] = delta_x[5] + nom_x[5];
   out_2670852950093832942[6] = delta_x[6] + nom_x[6];
   out_2670852950093832942[7] = delta_x[7] + nom_x[7];
   out_2670852950093832942[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1893451610308699874) {
   out_1893451610308699874[0] = -nom_x[0] + true_x[0];
   out_1893451610308699874[1] = -nom_x[1] + true_x[1];
   out_1893451610308699874[2] = -nom_x[2] + true_x[2];
   out_1893451610308699874[3] = -nom_x[3] + true_x[3];
   out_1893451610308699874[4] = -nom_x[4] + true_x[4];
   out_1893451610308699874[5] = -nom_x[5] + true_x[5];
   out_1893451610308699874[6] = -nom_x[6] + true_x[6];
   out_1893451610308699874[7] = -nom_x[7] + true_x[7];
   out_1893451610308699874[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_6889375267756851687) {
   out_6889375267756851687[0] = 1.0;
   out_6889375267756851687[1] = 0.0;
   out_6889375267756851687[2] = 0.0;
   out_6889375267756851687[3] = 0.0;
   out_6889375267756851687[4] = 0.0;
   out_6889375267756851687[5] = 0.0;
   out_6889375267756851687[6] = 0.0;
   out_6889375267756851687[7] = 0.0;
   out_6889375267756851687[8] = 0.0;
   out_6889375267756851687[9] = 0.0;
   out_6889375267756851687[10] = 1.0;
   out_6889375267756851687[11] = 0.0;
   out_6889375267756851687[12] = 0.0;
   out_6889375267756851687[13] = 0.0;
   out_6889375267756851687[14] = 0.0;
   out_6889375267756851687[15] = 0.0;
   out_6889375267756851687[16] = 0.0;
   out_6889375267756851687[17] = 0.0;
   out_6889375267756851687[18] = 0.0;
   out_6889375267756851687[19] = 0.0;
   out_6889375267756851687[20] = 1.0;
   out_6889375267756851687[21] = 0.0;
   out_6889375267756851687[22] = 0.0;
   out_6889375267756851687[23] = 0.0;
   out_6889375267756851687[24] = 0.0;
   out_6889375267756851687[25] = 0.0;
   out_6889375267756851687[26] = 0.0;
   out_6889375267756851687[27] = 0.0;
   out_6889375267756851687[28] = 0.0;
   out_6889375267756851687[29] = 0.0;
   out_6889375267756851687[30] = 1.0;
   out_6889375267756851687[31] = 0.0;
   out_6889375267756851687[32] = 0.0;
   out_6889375267756851687[33] = 0.0;
   out_6889375267756851687[34] = 0.0;
   out_6889375267756851687[35] = 0.0;
   out_6889375267756851687[36] = 0.0;
   out_6889375267756851687[37] = 0.0;
   out_6889375267756851687[38] = 0.0;
   out_6889375267756851687[39] = 0.0;
   out_6889375267756851687[40] = 1.0;
   out_6889375267756851687[41] = 0.0;
   out_6889375267756851687[42] = 0.0;
   out_6889375267756851687[43] = 0.0;
   out_6889375267756851687[44] = 0.0;
   out_6889375267756851687[45] = 0.0;
   out_6889375267756851687[46] = 0.0;
   out_6889375267756851687[47] = 0.0;
   out_6889375267756851687[48] = 0.0;
   out_6889375267756851687[49] = 0.0;
   out_6889375267756851687[50] = 1.0;
   out_6889375267756851687[51] = 0.0;
   out_6889375267756851687[52] = 0.0;
   out_6889375267756851687[53] = 0.0;
   out_6889375267756851687[54] = 0.0;
   out_6889375267756851687[55] = 0.0;
   out_6889375267756851687[56] = 0.0;
   out_6889375267756851687[57] = 0.0;
   out_6889375267756851687[58] = 0.0;
   out_6889375267756851687[59] = 0.0;
   out_6889375267756851687[60] = 1.0;
   out_6889375267756851687[61] = 0.0;
   out_6889375267756851687[62] = 0.0;
   out_6889375267756851687[63] = 0.0;
   out_6889375267756851687[64] = 0.0;
   out_6889375267756851687[65] = 0.0;
   out_6889375267756851687[66] = 0.0;
   out_6889375267756851687[67] = 0.0;
   out_6889375267756851687[68] = 0.0;
   out_6889375267756851687[69] = 0.0;
   out_6889375267756851687[70] = 1.0;
   out_6889375267756851687[71] = 0.0;
   out_6889375267756851687[72] = 0.0;
   out_6889375267756851687[73] = 0.0;
   out_6889375267756851687[74] = 0.0;
   out_6889375267756851687[75] = 0.0;
   out_6889375267756851687[76] = 0.0;
   out_6889375267756851687[77] = 0.0;
   out_6889375267756851687[78] = 0.0;
   out_6889375267756851687[79] = 0.0;
   out_6889375267756851687[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_4874872257159454940) {
   out_4874872257159454940[0] = state[0];
   out_4874872257159454940[1] = state[1];
   out_4874872257159454940[2] = state[2];
   out_4874872257159454940[3] = state[3];
   out_4874872257159454940[4] = state[4];
   out_4874872257159454940[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_4874872257159454940[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_4874872257159454940[7] = state[7];
   out_4874872257159454940[8] = state[8];
}
void F_fun(double *state, double dt, double *out_2486571753367481834) {
   out_2486571753367481834[0] = 1;
   out_2486571753367481834[1] = 0;
   out_2486571753367481834[2] = 0;
   out_2486571753367481834[3] = 0;
   out_2486571753367481834[4] = 0;
   out_2486571753367481834[5] = 0;
   out_2486571753367481834[6] = 0;
   out_2486571753367481834[7] = 0;
   out_2486571753367481834[8] = 0;
   out_2486571753367481834[9] = 0;
   out_2486571753367481834[10] = 1;
   out_2486571753367481834[11] = 0;
   out_2486571753367481834[12] = 0;
   out_2486571753367481834[13] = 0;
   out_2486571753367481834[14] = 0;
   out_2486571753367481834[15] = 0;
   out_2486571753367481834[16] = 0;
   out_2486571753367481834[17] = 0;
   out_2486571753367481834[18] = 0;
   out_2486571753367481834[19] = 0;
   out_2486571753367481834[20] = 1;
   out_2486571753367481834[21] = 0;
   out_2486571753367481834[22] = 0;
   out_2486571753367481834[23] = 0;
   out_2486571753367481834[24] = 0;
   out_2486571753367481834[25] = 0;
   out_2486571753367481834[26] = 0;
   out_2486571753367481834[27] = 0;
   out_2486571753367481834[28] = 0;
   out_2486571753367481834[29] = 0;
   out_2486571753367481834[30] = 1;
   out_2486571753367481834[31] = 0;
   out_2486571753367481834[32] = 0;
   out_2486571753367481834[33] = 0;
   out_2486571753367481834[34] = 0;
   out_2486571753367481834[35] = 0;
   out_2486571753367481834[36] = 0;
   out_2486571753367481834[37] = 0;
   out_2486571753367481834[38] = 0;
   out_2486571753367481834[39] = 0;
   out_2486571753367481834[40] = 1;
   out_2486571753367481834[41] = 0;
   out_2486571753367481834[42] = 0;
   out_2486571753367481834[43] = 0;
   out_2486571753367481834[44] = 0;
   out_2486571753367481834[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_2486571753367481834[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_2486571753367481834[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2486571753367481834[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2486571753367481834[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_2486571753367481834[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_2486571753367481834[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_2486571753367481834[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_2486571753367481834[53] = -9.8100000000000005*dt;
   out_2486571753367481834[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_2486571753367481834[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_2486571753367481834[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2486571753367481834[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2486571753367481834[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_2486571753367481834[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_2486571753367481834[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_2486571753367481834[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2486571753367481834[62] = 0;
   out_2486571753367481834[63] = 0;
   out_2486571753367481834[64] = 0;
   out_2486571753367481834[65] = 0;
   out_2486571753367481834[66] = 0;
   out_2486571753367481834[67] = 0;
   out_2486571753367481834[68] = 0;
   out_2486571753367481834[69] = 0;
   out_2486571753367481834[70] = 1;
   out_2486571753367481834[71] = 0;
   out_2486571753367481834[72] = 0;
   out_2486571753367481834[73] = 0;
   out_2486571753367481834[74] = 0;
   out_2486571753367481834[75] = 0;
   out_2486571753367481834[76] = 0;
   out_2486571753367481834[77] = 0;
   out_2486571753367481834[78] = 0;
   out_2486571753367481834[79] = 0;
   out_2486571753367481834[80] = 1;
}
void h_25(double *state, double *unused, double *out_6896276414448313325) {
   out_6896276414448313325[0] = state[6];
}
void H_25(double *state, double *unused, double *out_6653676980768035089) {
   out_6653676980768035089[0] = 0;
   out_6653676980768035089[1] = 0;
   out_6653676980768035089[2] = 0;
   out_6653676980768035089[3] = 0;
   out_6653676980768035089[4] = 0;
   out_6653676980768035089[5] = 0;
   out_6653676980768035089[6] = 1;
   out_6653676980768035089[7] = 0;
   out_6653676980768035089[8] = 0;
}
void h_24(double *state, double *unused, double *out_344120072819404180) {
   out_344120072819404180[0] = state[4];
   out_344120072819404180[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7810743941543748629) {
   out_7810743941543748629[0] = 0;
   out_7810743941543748629[1] = 0;
   out_7810743941543748629[2] = 0;
   out_7810743941543748629[3] = 0;
   out_7810743941543748629[4] = 1;
   out_7810743941543748629[5] = 0;
   out_7810743941543748629[6] = 0;
   out_7810743941543748629[7] = 0;
   out_7810743941543748629[8] = 0;
   out_7810743941543748629[9] = 0;
   out_7810743941543748629[10] = 0;
   out_7810743941543748629[11] = 0;
   out_7810743941543748629[12] = 0;
   out_7810743941543748629[13] = 0;
   out_7810743941543748629[14] = 1;
   out_7810743941543748629[15] = 0;
   out_7810743941543748629[16] = 0;
   out_7810743941543748629[17] = 0;
}
void h_30(double *state, double *unused, double *out_3258826886194290975) {
   out_3258826886194290975[0] = state[4];
}
void H_30(double *state, double *unused, double *out_4135344022260786462) {
   out_4135344022260786462[0] = 0;
   out_4135344022260786462[1] = 0;
   out_4135344022260786462[2] = 0;
   out_4135344022260786462[3] = 0;
   out_4135344022260786462[4] = 1;
   out_4135344022260786462[5] = 0;
   out_4135344022260786462[6] = 0;
   out_4135344022260786462[7] = 0;
   out_4135344022260786462[8] = 0;
}
void h_26(double *state, double *unused, double *out_7684187801492522508) {
   out_7684187801492522508[0] = state[7];
}
void H_26(double *state, double *unused, double *out_8051563774067460303) {
   out_8051563774067460303[0] = 0;
   out_8051563774067460303[1] = 0;
   out_8051563774067460303[2] = 0;
   out_8051563774067460303[3] = 0;
   out_8051563774067460303[4] = 0;
   out_8051563774067460303[5] = 0;
   out_8051563774067460303[6] = 0;
   out_8051563774067460303[7] = 1;
   out_8051563774067460303[8] = 0;
}
void h_27(double *state, double *unused, double *out_1900033049817683815) {
   out_1900033049817683815[0] = state[3];
}
void H_27(double *state, double *unused, double *out_6310107334061211373) {
   out_6310107334061211373[0] = 0;
   out_6310107334061211373[1] = 0;
   out_6310107334061211373[2] = 0;
   out_6310107334061211373[3] = 1;
   out_6310107334061211373[4] = 0;
   out_6310107334061211373[5] = 0;
   out_6310107334061211373[6] = 0;
   out_6310107334061211373[7] = 0;
   out_6310107334061211373[8] = 0;
}
void h_29(double *state, double *unused, double *out_1742846381466554670) {
   out_1742846381466554670[0] = state[1];
}
void H_29(double *state, double *unused, double *out_3625112677946394278) {
   out_3625112677946394278[0] = 0;
   out_3625112677946394278[1] = 1;
   out_3625112677946394278[2] = 0;
   out_3625112677946394278[3] = 0;
   out_3625112677946394278[4] = 0;
   out_3625112677946394278[5] = 0;
   out_3625112677946394278[6] = 0;
   out_3625112677946394278[7] = 0;
   out_3625112677946394278[8] = 0;
}
void h_28(double *state, double *unused, double *out_912519053452169568) {
   out_912519053452169568[0] = state[0];
}
void H_28(double *state, double *unused, double *out_8707511695015924852) {
   out_8707511695015924852[0] = 1;
   out_8707511695015924852[1] = 0;
   out_8707511695015924852[2] = 0;
   out_8707511695015924852[3] = 0;
   out_8707511695015924852[4] = 0;
   out_8707511695015924852[5] = 0;
   out_8707511695015924852[6] = 0;
   out_8707511695015924852[7] = 0;
   out_8707511695015924852[8] = 0;
}
void h_31(double *state, double *unused, double *out_6739089746097184180) {
   out_6739089746097184180[0] = state[8];
}
void H_31(double *state, double *unused, double *out_7425355671834108827) {
   out_7425355671834108827[0] = 0;
   out_7425355671834108827[1] = 0;
   out_7425355671834108827[2] = 0;
   out_7425355671834108827[3] = 0;
   out_7425355671834108827[4] = 0;
   out_7425355671834108827[5] = 0;
   out_7425355671834108827[6] = 0;
   out_7425355671834108827[7] = 0;
   out_7425355671834108827[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_2670852950093832942) {
  err_fun(nom_x, delta_x, out_2670852950093832942);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1893451610308699874) {
  inv_err_fun(nom_x, true_x, out_1893451610308699874);
}
void car_H_mod_fun(double *state, double *out_6889375267756851687) {
  H_mod_fun(state, out_6889375267756851687);
}
void car_f_fun(double *state, double dt, double *out_4874872257159454940) {
  f_fun(state,  dt, out_4874872257159454940);
}
void car_F_fun(double *state, double dt, double *out_2486571753367481834) {
  F_fun(state,  dt, out_2486571753367481834);
}
void car_h_25(double *state, double *unused, double *out_6896276414448313325) {
  h_25(state, unused, out_6896276414448313325);
}
void car_H_25(double *state, double *unused, double *out_6653676980768035089) {
  H_25(state, unused, out_6653676980768035089);
}
void car_h_24(double *state, double *unused, double *out_344120072819404180) {
  h_24(state, unused, out_344120072819404180);
}
void car_H_24(double *state, double *unused, double *out_7810743941543748629) {
  H_24(state, unused, out_7810743941543748629);
}
void car_h_30(double *state, double *unused, double *out_3258826886194290975) {
  h_30(state, unused, out_3258826886194290975);
}
void car_H_30(double *state, double *unused, double *out_4135344022260786462) {
  H_30(state, unused, out_4135344022260786462);
}
void car_h_26(double *state, double *unused, double *out_7684187801492522508) {
  h_26(state, unused, out_7684187801492522508);
}
void car_H_26(double *state, double *unused, double *out_8051563774067460303) {
  H_26(state, unused, out_8051563774067460303);
}
void car_h_27(double *state, double *unused, double *out_1900033049817683815) {
  h_27(state, unused, out_1900033049817683815);
}
void car_H_27(double *state, double *unused, double *out_6310107334061211373) {
  H_27(state, unused, out_6310107334061211373);
}
void car_h_29(double *state, double *unused, double *out_1742846381466554670) {
  h_29(state, unused, out_1742846381466554670);
}
void car_H_29(double *state, double *unused, double *out_3625112677946394278) {
  H_29(state, unused, out_3625112677946394278);
}
void car_h_28(double *state, double *unused, double *out_912519053452169568) {
  h_28(state, unused, out_912519053452169568);
}
void car_H_28(double *state, double *unused, double *out_8707511695015924852) {
  H_28(state, unused, out_8707511695015924852);
}
void car_h_31(double *state, double *unused, double *out_6739089746097184180) {
  h_31(state, unused, out_6739089746097184180);
}
void car_H_31(double *state, double *unused, double *out_7425355671834108827) {
  H_31(state, unused, out_7425355671834108827);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)

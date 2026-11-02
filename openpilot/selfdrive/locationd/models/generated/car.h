#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_2670852950093832942);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1893451610308699874);
void car_H_mod_fun(double *state, double *out_6889375267756851687);
void car_f_fun(double *state, double dt, double *out_4874872257159454940);
void car_F_fun(double *state, double dt, double *out_2486571753367481834);
void car_h_25(double *state, double *unused, double *out_6896276414448313325);
void car_H_25(double *state, double *unused, double *out_6653676980768035089);
void car_h_24(double *state, double *unused, double *out_344120072819404180);
void car_H_24(double *state, double *unused, double *out_7810743941543748629);
void car_h_30(double *state, double *unused, double *out_3258826886194290975);
void car_H_30(double *state, double *unused, double *out_4135344022260786462);
void car_h_26(double *state, double *unused, double *out_7684187801492522508);
void car_H_26(double *state, double *unused, double *out_8051563774067460303);
void car_h_27(double *state, double *unused, double *out_1900033049817683815);
void car_H_27(double *state, double *unused, double *out_6310107334061211373);
void car_h_29(double *state, double *unused, double *out_1742846381466554670);
void car_H_29(double *state, double *unused, double *out_3625112677946394278);
void car_h_28(double *state, double *unused, double *out_912519053452169568);
void car_H_28(double *state, double *unused, double *out_8707511695015924852);
void car_h_31(double *state, double *unused, double *out_6739089746097184180);
void car_H_31(double *state, double *unused, double *out_7425355671834108827);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
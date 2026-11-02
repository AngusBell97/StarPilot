#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_5650027980803817657);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_4887051929097092475);
void pose_H_mod_fun(double *state, double *out_9161776254870342825);
void pose_f_fun(double *state, double dt, double *out_1517811443874763877);
void pose_F_fun(double *state, double dt, double *out_5066981328479507862);
void pose_h_4(double *state, double *unused, double *out_6644950656028215533);
void pose_H_4(double *state, double *unused, double *out_2498880070693830364);
void pose_h_10(double *state, double *unused, double *out_59421258148822753);
void pose_H_10(double *state, double *unused, double *out_1808581595619348280);
void pose_h_13(double *state, double *unused, double *out_1847156105641674536);
void pose_H_13(double *state, double *unused, double *out_5711153896026163165);
void pose_h_14(double *state, double *unused, double *out_1855766992358449398);
void pose_H_14(double *state, double *unused, double *out_583908361601541932);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}
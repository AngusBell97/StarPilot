#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_5650027980803817657) {
   out_5650027980803817657[0] = delta_x[0] + nom_x[0];
   out_5650027980803817657[1] = delta_x[1] + nom_x[1];
   out_5650027980803817657[2] = delta_x[2] + nom_x[2];
   out_5650027980803817657[3] = delta_x[3] + nom_x[3];
   out_5650027980803817657[4] = delta_x[4] + nom_x[4];
   out_5650027980803817657[5] = delta_x[5] + nom_x[5];
   out_5650027980803817657[6] = delta_x[6] + nom_x[6];
   out_5650027980803817657[7] = delta_x[7] + nom_x[7];
   out_5650027980803817657[8] = delta_x[8] + nom_x[8];
   out_5650027980803817657[9] = delta_x[9] + nom_x[9];
   out_5650027980803817657[10] = delta_x[10] + nom_x[10];
   out_5650027980803817657[11] = delta_x[11] + nom_x[11];
   out_5650027980803817657[12] = delta_x[12] + nom_x[12];
   out_5650027980803817657[13] = delta_x[13] + nom_x[13];
   out_5650027980803817657[14] = delta_x[14] + nom_x[14];
   out_5650027980803817657[15] = delta_x[15] + nom_x[15];
   out_5650027980803817657[16] = delta_x[16] + nom_x[16];
   out_5650027980803817657[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4887051929097092475) {
   out_4887051929097092475[0] = -nom_x[0] + true_x[0];
   out_4887051929097092475[1] = -nom_x[1] + true_x[1];
   out_4887051929097092475[2] = -nom_x[2] + true_x[2];
   out_4887051929097092475[3] = -nom_x[3] + true_x[3];
   out_4887051929097092475[4] = -nom_x[4] + true_x[4];
   out_4887051929097092475[5] = -nom_x[5] + true_x[5];
   out_4887051929097092475[6] = -nom_x[6] + true_x[6];
   out_4887051929097092475[7] = -nom_x[7] + true_x[7];
   out_4887051929097092475[8] = -nom_x[8] + true_x[8];
   out_4887051929097092475[9] = -nom_x[9] + true_x[9];
   out_4887051929097092475[10] = -nom_x[10] + true_x[10];
   out_4887051929097092475[11] = -nom_x[11] + true_x[11];
   out_4887051929097092475[12] = -nom_x[12] + true_x[12];
   out_4887051929097092475[13] = -nom_x[13] + true_x[13];
   out_4887051929097092475[14] = -nom_x[14] + true_x[14];
   out_4887051929097092475[15] = -nom_x[15] + true_x[15];
   out_4887051929097092475[16] = -nom_x[16] + true_x[16];
   out_4887051929097092475[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_9161776254870342825) {
   out_9161776254870342825[0] = 1.0;
   out_9161776254870342825[1] = 0.0;
   out_9161776254870342825[2] = 0.0;
   out_9161776254870342825[3] = 0.0;
   out_9161776254870342825[4] = 0.0;
   out_9161776254870342825[5] = 0.0;
   out_9161776254870342825[6] = 0.0;
   out_9161776254870342825[7] = 0.0;
   out_9161776254870342825[8] = 0.0;
   out_9161776254870342825[9] = 0.0;
   out_9161776254870342825[10] = 0.0;
   out_9161776254870342825[11] = 0.0;
   out_9161776254870342825[12] = 0.0;
   out_9161776254870342825[13] = 0.0;
   out_9161776254870342825[14] = 0.0;
   out_9161776254870342825[15] = 0.0;
   out_9161776254870342825[16] = 0.0;
   out_9161776254870342825[17] = 0.0;
   out_9161776254870342825[18] = 0.0;
   out_9161776254870342825[19] = 1.0;
   out_9161776254870342825[20] = 0.0;
   out_9161776254870342825[21] = 0.0;
   out_9161776254870342825[22] = 0.0;
   out_9161776254870342825[23] = 0.0;
   out_9161776254870342825[24] = 0.0;
   out_9161776254870342825[25] = 0.0;
   out_9161776254870342825[26] = 0.0;
   out_9161776254870342825[27] = 0.0;
   out_9161776254870342825[28] = 0.0;
   out_9161776254870342825[29] = 0.0;
   out_9161776254870342825[30] = 0.0;
   out_9161776254870342825[31] = 0.0;
   out_9161776254870342825[32] = 0.0;
   out_9161776254870342825[33] = 0.0;
   out_9161776254870342825[34] = 0.0;
   out_9161776254870342825[35] = 0.0;
   out_9161776254870342825[36] = 0.0;
   out_9161776254870342825[37] = 0.0;
   out_9161776254870342825[38] = 1.0;
   out_9161776254870342825[39] = 0.0;
   out_9161776254870342825[40] = 0.0;
   out_9161776254870342825[41] = 0.0;
   out_9161776254870342825[42] = 0.0;
   out_9161776254870342825[43] = 0.0;
   out_9161776254870342825[44] = 0.0;
   out_9161776254870342825[45] = 0.0;
   out_9161776254870342825[46] = 0.0;
   out_9161776254870342825[47] = 0.0;
   out_9161776254870342825[48] = 0.0;
   out_9161776254870342825[49] = 0.0;
   out_9161776254870342825[50] = 0.0;
   out_9161776254870342825[51] = 0.0;
   out_9161776254870342825[52] = 0.0;
   out_9161776254870342825[53] = 0.0;
   out_9161776254870342825[54] = 0.0;
   out_9161776254870342825[55] = 0.0;
   out_9161776254870342825[56] = 0.0;
   out_9161776254870342825[57] = 1.0;
   out_9161776254870342825[58] = 0.0;
   out_9161776254870342825[59] = 0.0;
   out_9161776254870342825[60] = 0.0;
   out_9161776254870342825[61] = 0.0;
   out_9161776254870342825[62] = 0.0;
   out_9161776254870342825[63] = 0.0;
   out_9161776254870342825[64] = 0.0;
   out_9161776254870342825[65] = 0.0;
   out_9161776254870342825[66] = 0.0;
   out_9161776254870342825[67] = 0.0;
   out_9161776254870342825[68] = 0.0;
   out_9161776254870342825[69] = 0.0;
   out_9161776254870342825[70] = 0.0;
   out_9161776254870342825[71] = 0.0;
   out_9161776254870342825[72] = 0.0;
   out_9161776254870342825[73] = 0.0;
   out_9161776254870342825[74] = 0.0;
   out_9161776254870342825[75] = 0.0;
   out_9161776254870342825[76] = 1.0;
   out_9161776254870342825[77] = 0.0;
   out_9161776254870342825[78] = 0.0;
   out_9161776254870342825[79] = 0.0;
   out_9161776254870342825[80] = 0.0;
   out_9161776254870342825[81] = 0.0;
   out_9161776254870342825[82] = 0.0;
   out_9161776254870342825[83] = 0.0;
   out_9161776254870342825[84] = 0.0;
   out_9161776254870342825[85] = 0.0;
   out_9161776254870342825[86] = 0.0;
   out_9161776254870342825[87] = 0.0;
   out_9161776254870342825[88] = 0.0;
   out_9161776254870342825[89] = 0.0;
   out_9161776254870342825[90] = 0.0;
   out_9161776254870342825[91] = 0.0;
   out_9161776254870342825[92] = 0.0;
   out_9161776254870342825[93] = 0.0;
   out_9161776254870342825[94] = 0.0;
   out_9161776254870342825[95] = 1.0;
   out_9161776254870342825[96] = 0.0;
   out_9161776254870342825[97] = 0.0;
   out_9161776254870342825[98] = 0.0;
   out_9161776254870342825[99] = 0.0;
   out_9161776254870342825[100] = 0.0;
   out_9161776254870342825[101] = 0.0;
   out_9161776254870342825[102] = 0.0;
   out_9161776254870342825[103] = 0.0;
   out_9161776254870342825[104] = 0.0;
   out_9161776254870342825[105] = 0.0;
   out_9161776254870342825[106] = 0.0;
   out_9161776254870342825[107] = 0.0;
   out_9161776254870342825[108] = 0.0;
   out_9161776254870342825[109] = 0.0;
   out_9161776254870342825[110] = 0.0;
   out_9161776254870342825[111] = 0.0;
   out_9161776254870342825[112] = 0.0;
   out_9161776254870342825[113] = 0.0;
   out_9161776254870342825[114] = 1.0;
   out_9161776254870342825[115] = 0.0;
   out_9161776254870342825[116] = 0.0;
   out_9161776254870342825[117] = 0.0;
   out_9161776254870342825[118] = 0.0;
   out_9161776254870342825[119] = 0.0;
   out_9161776254870342825[120] = 0.0;
   out_9161776254870342825[121] = 0.0;
   out_9161776254870342825[122] = 0.0;
   out_9161776254870342825[123] = 0.0;
   out_9161776254870342825[124] = 0.0;
   out_9161776254870342825[125] = 0.0;
   out_9161776254870342825[126] = 0.0;
   out_9161776254870342825[127] = 0.0;
   out_9161776254870342825[128] = 0.0;
   out_9161776254870342825[129] = 0.0;
   out_9161776254870342825[130] = 0.0;
   out_9161776254870342825[131] = 0.0;
   out_9161776254870342825[132] = 0.0;
   out_9161776254870342825[133] = 1.0;
   out_9161776254870342825[134] = 0.0;
   out_9161776254870342825[135] = 0.0;
   out_9161776254870342825[136] = 0.0;
   out_9161776254870342825[137] = 0.0;
   out_9161776254870342825[138] = 0.0;
   out_9161776254870342825[139] = 0.0;
   out_9161776254870342825[140] = 0.0;
   out_9161776254870342825[141] = 0.0;
   out_9161776254870342825[142] = 0.0;
   out_9161776254870342825[143] = 0.0;
   out_9161776254870342825[144] = 0.0;
   out_9161776254870342825[145] = 0.0;
   out_9161776254870342825[146] = 0.0;
   out_9161776254870342825[147] = 0.0;
   out_9161776254870342825[148] = 0.0;
   out_9161776254870342825[149] = 0.0;
   out_9161776254870342825[150] = 0.0;
   out_9161776254870342825[151] = 0.0;
   out_9161776254870342825[152] = 1.0;
   out_9161776254870342825[153] = 0.0;
   out_9161776254870342825[154] = 0.0;
   out_9161776254870342825[155] = 0.0;
   out_9161776254870342825[156] = 0.0;
   out_9161776254870342825[157] = 0.0;
   out_9161776254870342825[158] = 0.0;
   out_9161776254870342825[159] = 0.0;
   out_9161776254870342825[160] = 0.0;
   out_9161776254870342825[161] = 0.0;
   out_9161776254870342825[162] = 0.0;
   out_9161776254870342825[163] = 0.0;
   out_9161776254870342825[164] = 0.0;
   out_9161776254870342825[165] = 0.0;
   out_9161776254870342825[166] = 0.0;
   out_9161776254870342825[167] = 0.0;
   out_9161776254870342825[168] = 0.0;
   out_9161776254870342825[169] = 0.0;
   out_9161776254870342825[170] = 0.0;
   out_9161776254870342825[171] = 1.0;
   out_9161776254870342825[172] = 0.0;
   out_9161776254870342825[173] = 0.0;
   out_9161776254870342825[174] = 0.0;
   out_9161776254870342825[175] = 0.0;
   out_9161776254870342825[176] = 0.0;
   out_9161776254870342825[177] = 0.0;
   out_9161776254870342825[178] = 0.0;
   out_9161776254870342825[179] = 0.0;
   out_9161776254870342825[180] = 0.0;
   out_9161776254870342825[181] = 0.0;
   out_9161776254870342825[182] = 0.0;
   out_9161776254870342825[183] = 0.0;
   out_9161776254870342825[184] = 0.0;
   out_9161776254870342825[185] = 0.0;
   out_9161776254870342825[186] = 0.0;
   out_9161776254870342825[187] = 0.0;
   out_9161776254870342825[188] = 0.0;
   out_9161776254870342825[189] = 0.0;
   out_9161776254870342825[190] = 1.0;
   out_9161776254870342825[191] = 0.0;
   out_9161776254870342825[192] = 0.0;
   out_9161776254870342825[193] = 0.0;
   out_9161776254870342825[194] = 0.0;
   out_9161776254870342825[195] = 0.0;
   out_9161776254870342825[196] = 0.0;
   out_9161776254870342825[197] = 0.0;
   out_9161776254870342825[198] = 0.0;
   out_9161776254870342825[199] = 0.0;
   out_9161776254870342825[200] = 0.0;
   out_9161776254870342825[201] = 0.0;
   out_9161776254870342825[202] = 0.0;
   out_9161776254870342825[203] = 0.0;
   out_9161776254870342825[204] = 0.0;
   out_9161776254870342825[205] = 0.0;
   out_9161776254870342825[206] = 0.0;
   out_9161776254870342825[207] = 0.0;
   out_9161776254870342825[208] = 0.0;
   out_9161776254870342825[209] = 1.0;
   out_9161776254870342825[210] = 0.0;
   out_9161776254870342825[211] = 0.0;
   out_9161776254870342825[212] = 0.0;
   out_9161776254870342825[213] = 0.0;
   out_9161776254870342825[214] = 0.0;
   out_9161776254870342825[215] = 0.0;
   out_9161776254870342825[216] = 0.0;
   out_9161776254870342825[217] = 0.0;
   out_9161776254870342825[218] = 0.0;
   out_9161776254870342825[219] = 0.0;
   out_9161776254870342825[220] = 0.0;
   out_9161776254870342825[221] = 0.0;
   out_9161776254870342825[222] = 0.0;
   out_9161776254870342825[223] = 0.0;
   out_9161776254870342825[224] = 0.0;
   out_9161776254870342825[225] = 0.0;
   out_9161776254870342825[226] = 0.0;
   out_9161776254870342825[227] = 0.0;
   out_9161776254870342825[228] = 1.0;
   out_9161776254870342825[229] = 0.0;
   out_9161776254870342825[230] = 0.0;
   out_9161776254870342825[231] = 0.0;
   out_9161776254870342825[232] = 0.0;
   out_9161776254870342825[233] = 0.0;
   out_9161776254870342825[234] = 0.0;
   out_9161776254870342825[235] = 0.0;
   out_9161776254870342825[236] = 0.0;
   out_9161776254870342825[237] = 0.0;
   out_9161776254870342825[238] = 0.0;
   out_9161776254870342825[239] = 0.0;
   out_9161776254870342825[240] = 0.0;
   out_9161776254870342825[241] = 0.0;
   out_9161776254870342825[242] = 0.0;
   out_9161776254870342825[243] = 0.0;
   out_9161776254870342825[244] = 0.0;
   out_9161776254870342825[245] = 0.0;
   out_9161776254870342825[246] = 0.0;
   out_9161776254870342825[247] = 1.0;
   out_9161776254870342825[248] = 0.0;
   out_9161776254870342825[249] = 0.0;
   out_9161776254870342825[250] = 0.0;
   out_9161776254870342825[251] = 0.0;
   out_9161776254870342825[252] = 0.0;
   out_9161776254870342825[253] = 0.0;
   out_9161776254870342825[254] = 0.0;
   out_9161776254870342825[255] = 0.0;
   out_9161776254870342825[256] = 0.0;
   out_9161776254870342825[257] = 0.0;
   out_9161776254870342825[258] = 0.0;
   out_9161776254870342825[259] = 0.0;
   out_9161776254870342825[260] = 0.0;
   out_9161776254870342825[261] = 0.0;
   out_9161776254870342825[262] = 0.0;
   out_9161776254870342825[263] = 0.0;
   out_9161776254870342825[264] = 0.0;
   out_9161776254870342825[265] = 0.0;
   out_9161776254870342825[266] = 1.0;
   out_9161776254870342825[267] = 0.0;
   out_9161776254870342825[268] = 0.0;
   out_9161776254870342825[269] = 0.0;
   out_9161776254870342825[270] = 0.0;
   out_9161776254870342825[271] = 0.0;
   out_9161776254870342825[272] = 0.0;
   out_9161776254870342825[273] = 0.0;
   out_9161776254870342825[274] = 0.0;
   out_9161776254870342825[275] = 0.0;
   out_9161776254870342825[276] = 0.0;
   out_9161776254870342825[277] = 0.0;
   out_9161776254870342825[278] = 0.0;
   out_9161776254870342825[279] = 0.0;
   out_9161776254870342825[280] = 0.0;
   out_9161776254870342825[281] = 0.0;
   out_9161776254870342825[282] = 0.0;
   out_9161776254870342825[283] = 0.0;
   out_9161776254870342825[284] = 0.0;
   out_9161776254870342825[285] = 1.0;
   out_9161776254870342825[286] = 0.0;
   out_9161776254870342825[287] = 0.0;
   out_9161776254870342825[288] = 0.0;
   out_9161776254870342825[289] = 0.0;
   out_9161776254870342825[290] = 0.0;
   out_9161776254870342825[291] = 0.0;
   out_9161776254870342825[292] = 0.0;
   out_9161776254870342825[293] = 0.0;
   out_9161776254870342825[294] = 0.0;
   out_9161776254870342825[295] = 0.0;
   out_9161776254870342825[296] = 0.0;
   out_9161776254870342825[297] = 0.0;
   out_9161776254870342825[298] = 0.0;
   out_9161776254870342825[299] = 0.0;
   out_9161776254870342825[300] = 0.0;
   out_9161776254870342825[301] = 0.0;
   out_9161776254870342825[302] = 0.0;
   out_9161776254870342825[303] = 0.0;
   out_9161776254870342825[304] = 1.0;
   out_9161776254870342825[305] = 0.0;
   out_9161776254870342825[306] = 0.0;
   out_9161776254870342825[307] = 0.0;
   out_9161776254870342825[308] = 0.0;
   out_9161776254870342825[309] = 0.0;
   out_9161776254870342825[310] = 0.0;
   out_9161776254870342825[311] = 0.0;
   out_9161776254870342825[312] = 0.0;
   out_9161776254870342825[313] = 0.0;
   out_9161776254870342825[314] = 0.0;
   out_9161776254870342825[315] = 0.0;
   out_9161776254870342825[316] = 0.0;
   out_9161776254870342825[317] = 0.0;
   out_9161776254870342825[318] = 0.0;
   out_9161776254870342825[319] = 0.0;
   out_9161776254870342825[320] = 0.0;
   out_9161776254870342825[321] = 0.0;
   out_9161776254870342825[322] = 0.0;
   out_9161776254870342825[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_1517811443874763877) {
   out_1517811443874763877[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_1517811443874763877[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_1517811443874763877[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_1517811443874763877[3] = dt*state[12] + state[3];
   out_1517811443874763877[4] = dt*state[13] + state[4];
   out_1517811443874763877[5] = dt*state[14] + state[5];
   out_1517811443874763877[6] = state[6];
   out_1517811443874763877[7] = state[7];
   out_1517811443874763877[8] = state[8];
   out_1517811443874763877[9] = state[9];
   out_1517811443874763877[10] = state[10];
   out_1517811443874763877[11] = state[11];
   out_1517811443874763877[12] = state[12];
   out_1517811443874763877[13] = state[13];
   out_1517811443874763877[14] = state[14];
   out_1517811443874763877[15] = state[15];
   out_1517811443874763877[16] = state[16];
   out_1517811443874763877[17] = state[17];
}
void F_fun(double *state, double dt, double *out_5066981328479507862) {
   out_5066981328479507862[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5066981328479507862[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5066981328479507862[2] = 0;
   out_5066981328479507862[3] = 0;
   out_5066981328479507862[4] = 0;
   out_5066981328479507862[5] = 0;
   out_5066981328479507862[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5066981328479507862[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5066981328479507862[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5066981328479507862[9] = 0;
   out_5066981328479507862[10] = 0;
   out_5066981328479507862[11] = 0;
   out_5066981328479507862[12] = 0;
   out_5066981328479507862[13] = 0;
   out_5066981328479507862[14] = 0;
   out_5066981328479507862[15] = 0;
   out_5066981328479507862[16] = 0;
   out_5066981328479507862[17] = 0;
   out_5066981328479507862[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5066981328479507862[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5066981328479507862[20] = 0;
   out_5066981328479507862[21] = 0;
   out_5066981328479507862[22] = 0;
   out_5066981328479507862[23] = 0;
   out_5066981328479507862[24] = 0;
   out_5066981328479507862[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5066981328479507862[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5066981328479507862[27] = 0;
   out_5066981328479507862[28] = 0;
   out_5066981328479507862[29] = 0;
   out_5066981328479507862[30] = 0;
   out_5066981328479507862[31] = 0;
   out_5066981328479507862[32] = 0;
   out_5066981328479507862[33] = 0;
   out_5066981328479507862[34] = 0;
   out_5066981328479507862[35] = 0;
   out_5066981328479507862[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5066981328479507862[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5066981328479507862[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5066981328479507862[39] = 0;
   out_5066981328479507862[40] = 0;
   out_5066981328479507862[41] = 0;
   out_5066981328479507862[42] = 0;
   out_5066981328479507862[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5066981328479507862[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5066981328479507862[45] = 0;
   out_5066981328479507862[46] = 0;
   out_5066981328479507862[47] = 0;
   out_5066981328479507862[48] = 0;
   out_5066981328479507862[49] = 0;
   out_5066981328479507862[50] = 0;
   out_5066981328479507862[51] = 0;
   out_5066981328479507862[52] = 0;
   out_5066981328479507862[53] = 0;
   out_5066981328479507862[54] = 0;
   out_5066981328479507862[55] = 0;
   out_5066981328479507862[56] = 0;
   out_5066981328479507862[57] = 1;
   out_5066981328479507862[58] = 0;
   out_5066981328479507862[59] = 0;
   out_5066981328479507862[60] = 0;
   out_5066981328479507862[61] = 0;
   out_5066981328479507862[62] = 0;
   out_5066981328479507862[63] = 0;
   out_5066981328479507862[64] = 0;
   out_5066981328479507862[65] = 0;
   out_5066981328479507862[66] = dt;
   out_5066981328479507862[67] = 0;
   out_5066981328479507862[68] = 0;
   out_5066981328479507862[69] = 0;
   out_5066981328479507862[70] = 0;
   out_5066981328479507862[71] = 0;
   out_5066981328479507862[72] = 0;
   out_5066981328479507862[73] = 0;
   out_5066981328479507862[74] = 0;
   out_5066981328479507862[75] = 0;
   out_5066981328479507862[76] = 1;
   out_5066981328479507862[77] = 0;
   out_5066981328479507862[78] = 0;
   out_5066981328479507862[79] = 0;
   out_5066981328479507862[80] = 0;
   out_5066981328479507862[81] = 0;
   out_5066981328479507862[82] = 0;
   out_5066981328479507862[83] = 0;
   out_5066981328479507862[84] = 0;
   out_5066981328479507862[85] = dt;
   out_5066981328479507862[86] = 0;
   out_5066981328479507862[87] = 0;
   out_5066981328479507862[88] = 0;
   out_5066981328479507862[89] = 0;
   out_5066981328479507862[90] = 0;
   out_5066981328479507862[91] = 0;
   out_5066981328479507862[92] = 0;
   out_5066981328479507862[93] = 0;
   out_5066981328479507862[94] = 0;
   out_5066981328479507862[95] = 1;
   out_5066981328479507862[96] = 0;
   out_5066981328479507862[97] = 0;
   out_5066981328479507862[98] = 0;
   out_5066981328479507862[99] = 0;
   out_5066981328479507862[100] = 0;
   out_5066981328479507862[101] = 0;
   out_5066981328479507862[102] = 0;
   out_5066981328479507862[103] = 0;
   out_5066981328479507862[104] = dt;
   out_5066981328479507862[105] = 0;
   out_5066981328479507862[106] = 0;
   out_5066981328479507862[107] = 0;
   out_5066981328479507862[108] = 0;
   out_5066981328479507862[109] = 0;
   out_5066981328479507862[110] = 0;
   out_5066981328479507862[111] = 0;
   out_5066981328479507862[112] = 0;
   out_5066981328479507862[113] = 0;
   out_5066981328479507862[114] = 1;
   out_5066981328479507862[115] = 0;
   out_5066981328479507862[116] = 0;
   out_5066981328479507862[117] = 0;
   out_5066981328479507862[118] = 0;
   out_5066981328479507862[119] = 0;
   out_5066981328479507862[120] = 0;
   out_5066981328479507862[121] = 0;
   out_5066981328479507862[122] = 0;
   out_5066981328479507862[123] = 0;
   out_5066981328479507862[124] = 0;
   out_5066981328479507862[125] = 0;
   out_5066981328479507862[126] = 0;
   out_5066981328479507862[127] = 0;
   out_5066981328479507862[128] = 0;
   out_5066981328479507862[129] = 0;
   out_5066981328479507862[130] = 0;
   out_5066981328479507862[131] = 0;
   out_5066981328479507862[132] = 0;
   out_5066981328479507862[133] = 1;
   out_5066981328479507862[134] = 0;
   out_5066981328479507862[135] = 0;
   out_5066981328479507862[136] = 0;
   out_5066981328479507862[137] = 0;
   out_5066981328479507862[138] = 0;
   out_5066981328479507862[139] = 0;
   out_5066981328479507862[140] = 0;
   out_5066981328479507862[141] = 0;
   out_5066981328479507862[142] = 0;
   out_5066981328479507862[143] = 0;
   out_5066981328479507862[144] = 0;
   out_5066981328479507862[145] = 0;
   out_5066981328479507862[146] = 0;
   out_5066981328479507862[147] = 0;
   out_5066981328479507862[148] = 0;
   out_5066981328479507862[149] = 0;
   out_5066981328479507862[150] = 0;
   out_5066981328479507862[151] = 0;
   out_5066981328479507862[152] = 1;
   out_5066981328479507862[153] = 0;
   out_5066981328479507862[154] = 0;
   out_5066981328479507862[155] = 0;
   out_5066981328479507862[156] = 0;
   out_5066981328479507862[157] = 0;
   out_5066981328479507862[158] = 0;
   out_5066981328479507862[159] = 0;
   out_5066981328479507862[160] = 0;
   out_5066981328479507862[161] = 0;
   out_5066981328479507862[162] = 0;
   out_5066981328479507862[163] = 0;
   out_5066981328479507862[164] = 0;
   out_5066981328479507862[165] = 0;
   out_5066981328479507862[166] = 0;
   out_5066981328479507862[167] = 0;
   out_5066981328479507862[168] = 0;
   out_5066981328479507862[169] = 0;
   out_5066981328479507862[170] = 0;
   out_5066981328479507862[171] = 1;
   out_5066981328479507862[172] = 0;
   out_5066981328479507862[173] = 0;
   out_5066981328479507862[174] = 0;
   out_5066981328479507862[175] = 0;
   out_5066981328479507862[176] = 0;
   out_5066981328479507862[177] = 0;
   out_5066981328479507862[178] = 0;
   out_5066981328479507862[179] = 0;
   out_5066981328479507862[180] = 0;
   out_5066981328479507862[181] = 0;
   out_5066981328479507862[182] = 0;
   out_5066981328479507862[183] = 0;
   out_5066981328479507862[184] = 0;
   out_5066981328479507862[185] = 0;
   out_5066981328479507862[186] = 0;
   out_5066981328479507862[187] = 0;
   out_5066981328479507862[188] = 0;
   out_5066981328479507862[189] = 0;
   out_5066981328479507862[190] = 1;
   out_5066981328479507862[191] = 0;
   out_5066981328479507862[192] = 0;
   out_5066981328479507862[193] = 0;
   out_5066981328479507862[194] = 0;
   out_5066981328479507862[195] = 0;
   out_5066981328479507862[196] = 0;
   out_5066981328479507862[197] = 0;
   out_5066981328479507862[198] = 0;
   out_5066981328479507862[199] = 0;
   out_5066981328479507862[200] = 0;
   out_5066981328479507862[201] = 0;
   out_5066981328479507862[202] = 0;
   out_5066981328479507862[203] = 0;
   out_5066981328479507862[204] = 0;
   out_5066981328479507862[205] = 0;
   out_5066981328479507862[206] = 0;
   out_5066981328479507862[207] = 0;
   out_5066981328479507862[208] = 0;
   out_5066981328479507862[209] = 1;
   out_5066981328479507862[210] = 0;
   out_5066981328479507862[211] = 0;
   out_5066981328479507862[212] = 0;
   out_5066981328479507862[213] = 0;
   out_5066981328479507862[214] = 0;
   out_5066981328479507862[215] = 0;
   out_5066981328479507862[216] = 0;
   out_5066981328479507862[217] = 0;
   out_5066981328479507862[218] = 0;
   out_5066981328479507862[219] = 0;
   out_5066981328479507862[220] = 0;
   out_5066981328479507862[221] = 0;
   out_5066981328479507862[222] = 0;
   out_5066981328479507862[223] = 0;
   out_5066981328479507862[224] = 0;
   out_5066981328479507862[225] = 0;
   out_5066981328479507862[226] = 0;
   out_5066981328479507862[227] = 0;
   out_5066981328479507862[228] = 1;
   out_5066981328479507862[229] = 0;
   out_5066981328479507862[230] = 0;
   out_5066981328479507862[231] = 0;
   out_5066981328479507862[232] = 0;
   out_5066981328479507862[233] = 0;
   out_5066981328479507862[234] = 0;
   out_5066981328479507862[235] = 0;
   out_5066981328479507862[236] = 0;
   out_5066981328479507862[237] = 0;
   out_5066981328479507862[238] = 0;
   out_5066981328479507862[239] = 0;
   out_5066981328479507862[240] = 0;
   out_5066981328479507862[241] = 0;
   out_5066981328479507862[242] = 0;
   out_5066981328479507862[243] = 0;
   out_5066981328479507862[244] = 0;
   out_5066981328479507862[245] = 0;
   out_5066981328479507862[246] = 0;
   out_5066981328479507862[247] = 1;
   out_5066981328479507862[248] = 0;
   out_5066981328479507862[249] = 0;
   out_5066981328479507862[250] = 0;
   out_5066981328479507862[251] = 0;
   out_5066981328479507862[252] = 0;
   out_5066981328479507862[253] = 0;
   out_5066981328479507862[254] = 0;
   out_5066981328479507862[255] = 0;
   out_5066981328479507862[256] = 0;
   out_5066981328479507862[257] = 0;
   out_5066981328479507862[258] = 0;
   out_5066981328479507862[259] = 0;
   out_5066981328479507862[260] = 0;
   out_5066981328479507862[261] = 0;
   out_5066981328479507862[262] = 0;
   out_5066981328479507862[263] = 0;
   out_5066981328479507862[264] = 0;
   out_5066981328479507862[265] = 0;
   out_5066981328479507862[266] = 1;
   out_5066981328479507862[267] = 0;
   out_5066981328479507862[268] = 0;
   out_5066981328479507862[269] = 0;
   out_5066981328479507862[270] = 0;
   out_5066981328479507862[271] = 0;
   out_5066981328479507862[272] = 0;
   out_5066981328479507862[273] = 0;
   out_5066981328479507862[274] = 0;
   out_5066981328479507862[275] = 0;
   out_5066981328479507862[276] = 0;
   out_5066981328479507862[277] = 0;
   out_5066981328479507862[278] = 0;
   out_5066981328479507862[279] = 0;
   out_5066981328479507862[280] = 0;
   out_5066981328479507862[281] = 0;
   out_5066981328479507862[282] = 0;
   out_5066981328479507862[283] = 0;
   out_5066981328479507862[284] = 0;
   out_5066981328479507862[285] = 1;
   out_5066981328479507862[286] = 0;
   out_5066981328479507862[287] = 0;
   out_5066981328479507862[288] = 0;
   out_5066981328479507862[289] = 0;
   out_5066981328479507862[290] = 0;
   out_5066981328479507862[291] = 0;
   out_5066981328479507862[292] = 0;
   out_5066981328479507862[293] = 0;
   out_5066981328479507862[294] = 0;
   out_5066981328479507862[295] = 0;
   out_5066981328479507862[296] = 0;
   out_5066981328479507862[297] = 0;
   out_5066981328479507862[298] = 0;
   out_5066981328479507862[299] = 0;
   out_5066981328479507862[300] = 0;
   out_5066981328479507862[301] = 0;
   out_5066981328479507862[302] = 0;
   out_5066981328479507862[303] = 0;
   out_5066981328479507862[304] = 1;
   out_5066981328479507862[305] = 0;
   out_5066981328479507862[306] = 0;
   out_5066981328479507862[307] = 0;
   out_5066981328479507862[308] = 0;
   out_5066981328479507862[309] = 0;
   out_5066981328479507862[310] = 0;
   out_5066981328479507862[311] = 0;
   out_5066981328479507862[312] = 0;
   out_5066981328479507862[313] = 0;
   out_5066981328479507862[314] = 0;
   out_5066981328479507862[315] = 0;
   out_5066981328479507862[316] = 0;
   out_5066981328479507862[317] = 0;
   out_5066981328479507862[318] = 0;
   out_5066981328479507862[319] = 0;
   out_5066981328479507862[320] = 0;
   out_5066981328479507862[321] = 0;
   out_5066981328479507862[322] = 0;
   out_5066981328479507862[323] = 1;
}
void h_4(double *state, double *unused, double *out_6644950656028215533) {
   out_6644950656028215533[0] = state[6] + state[9];
   out_6644950656028215533[1] = state[7] + state[10];
   out_6644950656028215533[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_2498880070693830364) {
   out_2498880070693830364[0] = 0;
   out_2498880070693830364[1] = 0;
   out_2498880070693830364[2] = 0;
   out_2498880070693830364[3] = 0;
   out_2498880070693830364[4] = 0;
   out_2498880070693830364[5] = 0;
   out_2498880070693830364[6] = 1;
   out_2498880070693830364[7] = 0;
   out_2498880070693830364[8] = 0;
   out_2498880070693830364[9] = 1;
   out_2498880070693830364[10] = 0;
   out_2498880070693830364[11] = 0;
   out_2498880070693830364[12] = 0;
   out_2498880070693830364[13] = 0;
   out_2498880070693830364[14] = 0;
   out_2498880070693830364[15] = 0;
   out_2498880070693830364[16] = 0;
   out_2498880070693830364[17] = 0;
   out_2498880070693830364[18] = 0;
   out_2498880070693830364[19] = 0;
   out_2498880070693830364[20] = 0;
   out_2498880070693830364[21] = 0;
   out_2498880070693830364[22] = 0;
   out_2498880070693830364[23] = 0;
   out_2498880070693830364[24] = 0;
   out_2498880070693830364[25] = 1;
   out_2498880070693830364[26] = 0;
   out_2498880070693830364[27] = 0;
   out_2498880070693830364[28] = 1;
   out_2498880070693830364[29] = 0;
   out_2498880070693830364[30] = 0;
   out_2498880070693830364[31] = 0;
   out_2498880070693830364[32] = 0;
   out_2498880070693830364[33] = 0;
   out_2498880070693830364[34] = 0;
   out_2498880070693830364[35] = 0;
   out_2498880070693830364[36] = 0;
   out_2498880070693830364[37] = 0;
   out_2498880070693830364[38] = 0;
   out_2498880070693830364[39] = 0;
   out_2498880070693830364[40] = 0;
   out_2498880070693830364[41] = 0;
   out_2498880070693830364[42] = 0;
   out_2498880070693830364[43] = 0;
   out_2498880070693830364[44] = 1;
   out_2498880070693830364[45] = 0;
   out_2498880070693830364[46] = 0;
   out_2498880070693830364[47] = 1;
   out_2498880070693830364[48] = 0;
   out_2498880070693830364[49] = 0;
   out_2498880070693830364[50] = 0;
   out_2498880070693830364[51] = 0;
   out_2498880070693830364[52] = 0;
   out_2498880070693830364[53] = 0;
}
void h_10(double *state, double *unused, double *out_59421258148822753) {
   out_59421258148822753[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_59421258148822753[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_59421258148822753[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_1808581595619348280) {
   out_1808581595619348280[0] = 0;
   out_1808581595619348280[1] = 9.8100000000000005*cos(state[1]);
   out_1808581595619348280[2] = 0;
   out_1808581595619348280[3] = 0;
   out_1808581595619348280[4] = -state[8];
   out_1808581595619348280[5] = state[7];
   out_1808581595619348280[6] = 0;
   out_1808581595619348280[7] = state[5];
   out_1808581595619348280[8] = -state[4];
   out_1808581595619348280[9] = 0;
   out_1808581595619348280[10] = 0;
   out_1808581595619348280[11] = 0;
   out_1808581595619348280[12] = 1;
   out_1808581595619348280[13] = 0;
   out_1808581595619348280[14] = 0;
   out_1808581595619348280[15] = 1;
   out_1808581595619348280[16] = 0;
   out_1808581595619348280[17] = 0;
   out_1808581595619348280[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_1808581595619348280[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_1808581595619348280[20] = 0;
   out_1808581595619348280[21] = state[8];
   out_1808581595619348280[22] = 0;
   out_1808581595619348280[23] = -state[6];
   out_1808581595619348280[24] = -state[5];
   out_1808581595619348280[25] = 0;
   out_1808581595619348280[26] = state[3];
   out_1808581595619348280[27] = 0;
   out_1808581595619348280[28] = 0;
   out_1808581595619348280[29] = 0;
   out_1808581595619348280[30] = 0;
   out_1808581595619348280[31] = 1;
   out_1808581595619348280[32] = 0;
   out_1808581595619348280[33] = 0;
   out_1808581595619348280[34] = 1;
   out_1808581595619348280[35] = 0;
   out_1808581595619348280[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_1808581595619348280[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_1808581595619348280[38] = 0;
   out_1808581595619348280[39] = -state[7];
   out_1808581595619348280[40] = state[6];
   out_1808581595619348280[41] = 0;
   out_1808581595619348280[42] = state[4];
   out_1808581595619348280[43] = -state[3];
   out_1808581595619348280[44] = 0;
   out_1808581595619348280[45] = 0;
   out_1808581595619348280[46] = 0;
   out_1808581595619348280[47] = 0;
   out_1808581595619348280[48] = 0;
   out_1808581595619348280[49] = 0;
   out_1808581595619348280[50] = 1;
   out_1808581595619348280[51] = 0;
   out_1808581595619348280[52] = 0;
   out_1808581595619348280[53] = 1;
}
void h_13(double *state, double *unused, double *out_1847156105641674536) {
   out_1847156105641674536[0] = state[3];
   out_1847156105641674536[1] = state[4];
   out_1847156105641674536[2] = state[5];
}
void H_13(double *state, double *unused, double *out_5711153896026163165) {
   out_5711153896026163165[0] = 0;
   out_5711153896026163165[1] = 0;
   out_5711153896026163165[2] = 0;
   out_5711153896026163165[3] = 1;
   out_5711153896026163165[4] = 0;
   out_5711153896026163165[5] = 0;
   out_5711153896026163165[6] = 0;
   out_5711153896026163165[7] = 0;
   out_5711153896026163165[8] = 0;
   out_5711153896026163165[9] = 0;
   out_5711153896026163165[10] = 0;
   out_5711153896026163165[11] = 0;
   out_5711153896026163165[12] = 0;
   out_5711153896026163165[13] = 0;
   out_5711153896026163165[14] = 0;
   out_5711153896026163165[15] = 0;
   out_5711153896026163165[16] = 0;
   out_5711153896026163165[17] = 0;
   out_5711153896026163165[18] = 0;
   out_5711153896026163165[19] = 0;
   out_5711153896026163165[20] = 0;
   out_5711153896026163165[21] = 0;
   out_5711153896026163165[22] = 1;
   out_5711153896026163165[23] = 0;
   out_5711153896026163165[24] = 0;
   out_5711153896026163165[25] = 0;
   out_5711153896026163165[26] = 0;
   out_5711153896026163165[27] = 0;
   out_5711153896026163165[28] = 0;
   out_5711153896026163165[29] = 0;
   out_5711153896026163165[30] = 0;
   out_5711153896026163165[31] = 0;
   out_5711153896026163165[32] = 0;
   out_5711153896026163165[33] = 0;
   out_5711153896026163165[34] = 0;
   out_5711153896026163165[35] = 0;
   out_5711153896026163165[36] = 0;
   out_5711153896026163165[37] = 0;
   out_5711153896026163165[38] = 0;
   out_5711153896026163165[39] = 0;
   out_5711153896026163165[40] = 0;
   out_5711153896026163165[41] = 1;
   out_5711153896026163165[42] = 0;
   out_5711153896026163165[43] = 0;
   out_5711153896026163165[44] = 0;
   out_5711153896026163165[45] = 0;
   out_5711153896026163165[46] = 0;
   out_5711153896026163165[47] = 0;
   out_5711153896026163165[48] = 0;
   out_5711153896026163165[49] = 0;
   out_5711153896026163165[50] = 0;
   out_5711153896026163165[51] = 0;
   out_5711153896026163165[52] = 0;
   out_5711153896026163165[53] = 0;
}
void h_14(double *state, double *unused, double *out_1855766992358449398) {
   out_1855766992358449398[0] = state[6];
   out_1855766992358449398[1] = state[7];
   out_1855766992358449398[2] = state[8];
}
void H_14(double *state, double *unused, double *out_583908361601541932) {
   out_583908361601541932[0] = 0;
   out_583908361601541932[1] = 0;
   out_583908361601541932[2] = 0;
   out_583908361601541932[3] = 0;
   out_583908361601541932[4] = 0;
   out_583908361601541932[5] = 0;
   out_583908361601541932[6] = 1;
   out_583908361601541932[7] = 0;
   out_583908361601541932[8] = 0;
   out_583908361601541932[9] = 0;
   out_583908361601541932[10] = 0;
   out_583908361601541932[11] = 0;
   out_583908361601541932[12] = 0;
   out_583908361601541932[13] = 0;
   out_583908361601541932[14] = 0;
   out_583908361601541932[15] = 0;
   out_583908361601541932[16] = 0;
   out_583908361601541932[17] = 0;
   out_583908361601541932[18] = 0;
   out_583908361601541932[19] = 0;
   out_583908361601541932[20] = 0;
   out_583908361601541932[21] = 0;
   out_583908361601541932[22] = 0;
   out_583908361601541932[23] = 0;
   out_583908361601541932[24] = 0;
   out_583908361601541932[25] = 1;
   out_583908361601541932[26] = 0;
   out_583908361601541932[27] = 0;
   out_583908361601541932[28] = 0;
   out_583908361601541932[29] = 0;
   out_583908361601541932[30] = 0;
   out_583908361601541932[31] = 0;
   out_583908361601541932[32] = 0;
   out_583908361601541932[33] = 0;
   out_583908361601541932[34] = 0;
   out_583908361601541932[35] = 0;
   out_583908361601541932[36] = 0;
   out_583908361601541932[37] = 0;
   out_583908361601541932[38] = 0;
   out_583908361601541932[39] = 0;
   out_583908361601541932[40] = 0;
   out_583908361601541932[41] = 0;
   out_583908361601541932[42] = 0;
   out_583908361601541932[43] = 0;
   out_583908361601541932[44] = 1;
   out_583908361601541932[45] = 0;
   out_583908361601541932[46] = 0;
   out_583908361601541932[47] = 0;
   out_583908361601541932[48] = 0;
   out_583908361601541932[49] = 0;
   out_583908361601541932[50] = 0;
   out_583908361601541932[51] = 0;
   out_583908361601541932[52] = 0;
   out_583908361601541932[53] = 0;
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

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_5650027980803817657) {
  err_fun(nom_x, delta_x, out_5650027980803817657);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_4887051929097092475) {
  inv_err_fun(nom_x, true_x, out_4887051929097092475);
}
void pose_H_mod_fun(double *state, double *out_9161776254870342825) {
  H_mod_fun(state, out_9161776254870342825);
}
void pose_f_fun(double *state, double dt, double *out_1517811443874763877) {
  f_fun(state,  dt, out_1517811443874763877);
}
void pose_F_fun(double *state, double dt, double *out_5066981328479507862) {
  F_fun(state,  dt, out_5066981328479507862);
}
void pose_h_4(double *state, double *unused, double *out_6644950656028215533) {
  h_4(state, unused, out_6644950656028215533);
}
void pose_H_4(double *state, double *unused, double *out_2498880070693830364) {
  H_4(state, unused, out_2498880070693830364);
}
void pose_h_10(double *state, double *unused, double *out_59421258148822753) {
  h_10(state, unused, out_59421258148822753);
}
void pose_H_10(double *state, double *unused, double *out_1808581595619348280) {
  H_10(state, unused, out_1808581595619348280);
}
void pose_h_13(double *state, double *unused, double *out_1847156105641674536) {
  h_13(state, unused, out_1847156105641674536);
}
void pose_H_13(double *state, double *unused, double *out_5711153896026163165) {
  H_13(state, unused, out_5711153896026163165);
}
void pose_h_14(double *state, double *unused, double *out_1855766992358449398) {
  h_14(state, unused, out_1855766992358449398);
}
void pose_H_14(double *state, double *unused, double *out_583908361601541932) {
  H_14(state, unused, out_583908361601541932);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)

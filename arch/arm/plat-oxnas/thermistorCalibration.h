#ifndef __THERMISTOR_LOOKUP_TABLE_10K3A_H
#define __THERMISTOR_LOOKUP_TABLE_10K3A_H

/* Thermistor is a 10K3A*/
/* THERM_COEF_A == 0.001129241*/
/* THERM_COEF_B == 0.0002341077. */
/* THERM_COEF_C == 0.00000008775468. */

/* Capacitor is 100 nF */
/* Stepped frequency increment is 128000 Hz */
/* Schmitdt trigger threshold assumed:  1 / 3.3 V */


/* Inverse C.ln(V') == 216 */
#define THERM_INTERPOLATION_STEP  8
#define THERM_ENTRIES_IN_CALIB_TABLE  128

static const unsigned long TvsCnt[THERM_ENTRIES_IN_CALIB_TABLE] = {
    416,    /* == 143.37 deg C: Count == 0, R == 216 Ohms */
    340,    /* == 67.07 deg C: Count == 8, R == 1948 Ohms */
    323,    /* == 49.59 deg C: Count == 16, R == 3679 Ohms */
    313,    /* == 39.76 deg C: Count == 24, R == 5410 Ohms */
    306,    /* == 33 deg C: Count == 32, R == 7141 Ohms */
    301,    /* == 27.9 deg C: Count == 40, R == 8873 Ohms */
    297,    /* == 23.82 deg C: Count == 48, R == 10604 Ohms */
    293,    /* == 20.43 deg C: Count == 56, R == 12335 Ohms */
    291,    /* == 17.55 deg C: Count == 64, R == 14066 Ohms */
    288,    /* == 15.04 deg C: Count == 72, R == 15798 Ohms */
    286,    /* == 12.82 deg C: Count == 80, R == 17529 Ohms */
    284,    /* == 10.84 deg C: Count == 88, R == 19260 Ohms */
    282,    /* == 9.04 deg C: Count == 96, R == 20991 Ohms */
    280,    /* == 7.41 deg C: Count == 104, R == 22722 Ohms */
    279,    /* == 5.91 deg C: Count == 112, R == 24454 Ohms */
    278,    /* == 4.53 deg C: Count == 120, R == 26185 Ohms */
    276,    /* == 3.25 deg C: Count == 128, R == 27916 Ohms */
    275,    /* == 2.05 deg C: Count == 136, R == 29647 Ohms */
    274,    /* == 0.93 deg C: Count == 144, R == 31379 Ohms */
    273,    /* == -0.12 deg C: Count == 152, R == 33110 Ohms */
    272,    /* == -1.12 deg C: Count == 160, R == 34841 Ohms */
    271,    /* == -2.06 deg C: Count == 168, R == 36572 Ohms */
    270,    /* == -2.95 deg C: Count == 176, R == 38304 Ohms */
    269,    /* == -3.8 deg C: Count == 184, R == 40035 Ohms */
    268,    /* == -4.6 deg C: Count == 192, R == 41766 Ohms */
    268,    /* == -5.37 deg C: Count == 200, R == 43497 Ohms */
    267,    /* == -6.11 deg C: Count == 208, R == 45229 Ohms */
    266,    /* == -6.81 deg C: Count == 216, R == 46960 Ohms */
    266,    /* == -7.49 deg C: Count == 224, R == 48691 Ohms */
    265,    /* == -8.14 deg C: Count == 232, R == 50422 Ohms */
    264,    /* == -8.77 deg C: Count == 240, R == 52154 Ohms */
    264,    /* == -9.37 deg C: Count == 248, R == 53885 Ohms */
    263,    /* == -9.95 deg C: Count == 256, R == 55616 Ohms */
    262,    /* == -10.52 deg C: Count == 264, R == 57347 Ohms */
    262,    /* == -11.06 deg C: Count == 272, R == 59078 Ohms */
    261,    /* == -11.59 deg C: Count == 280, R == 60810 Ohms */
    261,    /* == -12.1 deg C: Count == 288, R == 62541 Ohms */
    260,    /* == -12.59 deg C: Count == 296, R == 64272 Ohms */
    260,    /* == -13.07 deg C: Count == 304, R == 66003 Ohms */
    259,    /* == -13.53 deg C: Count == 312, R == 67735 Ohms */
    259,    /* == -13.99 deg C: Count == 320, R == 69466 Ohms */
    259,    /* == -14.43 deg C: Count == 328, R == 71197 Ohms */
    258,    /* == -14.86 deg C: Count == 336, R == 72928 Ohms */
    258,    /* == -15.27 deg C: Count == 344, R == 74660 Ohms */
    257,    /* == -15.68 deg C: Count == 352, R == 76391 Ohms */
    257,    /* == -16.08 deg C: Count == 360, R == 78122 Ohms */
    257,    /* == -16.46 deg C: Count == 368, R == 79853 Ohms */
    256,    /* == -16.84 deg C: Count == 376, R == 81585 Ohms */
    256,    /* == -17.21 deg C: Count == 384, R == 83316 Ohms */
    255,    /* == -17.57 deg C: Count == 392, R == 85047 Ohms */
    255,    /* == -17.92 deg C: Count == 400, R == 86778 Ohms */
    255,    /* == -18.26 deg C: Count == 408, R == 88510 Ohms */
    254,    /* == -18.6 deg C: Count == 416, R == 90241 Ohms */
    254,    /* == -18.93 deg C: Count == 424, R == 91972 Ohms */
    254,    /* == -19.25 deg C: Count == 432, R == 93703 Ohms */
    253,    /* == -19.57 deg C: Count == 440, R == 95434 Ohms */
    253,    /* == -19.88 deg C: Count == 448, R == 97166 Ohms */
    253,    /* == -20.18 deg C: Count == 456, R == 98897 Ohms */
    253,    /* == -20.48 deg C: Count == 464, R == 100628 Ohms */
    252,    /* == -20.77 deg C: Count == 472, R == 102359 Ohms */
    252,    /* == -21.06 deg C: Count == 480, R == 104091 Ohms */
    252,    /* == -21.34 deg C: Count == 488, R == 105822 Ohms */
    251,    /* == -21.62 deg C: Count == 496, R == 107553 Ohms */
    251,    /* == -21.89 deg C: Count == 504, R == 109284 Ohms */
    251,    /* == -22.16 deg C: Count == 512, R == 111016 Ohms */
    251,    /* == -22.42 deg C: Count == 520, R == 112747 Ohms */
    250,    /* == -22.68 deg C: Count == 528, R == 114478 Ohms */
    250,    /* == -22.93 deg C: Count == 536, R == 116209 Ohms */
    250,    /* == -23.18 deg C: Count == 544, R == 117941 Ohms */
    250,    /* == -23.43 deg C: Count == 552, R == 119672 Ohms */
    249,    /* == -23.67 deg C: Count == 560, R == 121403 Ohms */
    249,    /* == -23.91 deg C: Count == 568, R == 123134 Ohms */
    249,    /* == -24.14 deg C: Count == 576, R == 124866 Ohms */
    249,    /* == -24.37 deg C: Count == 584, R == 126597 Ohms */
    248,    /* == -24.6 deg C: Count == 592, R == 128328 Ohms */
    248,    /* == -24.82 deg C: Count == 600, R == 130059 Ohms */
    248,    /* == -25.04 deg C: Count == 608, R == 131790 Ohms */
    248,    /* == -25.26 deg C: Count == 616, R == 133522 Ohms */
    248,    /* == -25.47 deg C: Count == 624, R == 135253 Ohms */
    247,    /* == -25.68 deg C: Count == 632, R == 136984 Ohms */
    247,    /* == -25.89 deg C: Count == 640, R == 138715 Ohms */
    247,    /* == -26.1 deg C: Count == 648, R == 140447 Ohms */
    247,    /* == -26.3 deg C: Count == 656, R == 142178 Ohms */
    247,    /* == -26.5 deg C: Count == 664, R == 143909 Ohms */
    246,    /* == -26.69 deg C: Count == 672, R == 145640 Ohms */
    246,    /* == -26.89 deg C: Count == 680, R == 147372 Ohms */
    246,    /* == -27.08 deg C: Count == 688, R == 149103 Ohms */
    246,    /* == -27.27 deg C: Count == 696, R == 150834 Ohms */
    246,    /* == -27.46 deg C: Count == 704, R == 152565 Ohms */
    245,    /* == -27.64 deg C: Count == 712, R == 154297 Ohms */
    245,    /* == -27.82 deg C: Count == 720, R == 156028 Ohms */
    245,    /* == -28 deg C: Count == 728, R == 157759 Ohms */
    245,    /* == -28.18 deg C: Count == 736, R == 159490 Ohms */
    245,    /* == -28.36 deg C: Count == 744, R == 161222 Ohms */
    244,    /* == -28.53 deg C: Count == 752, R == 162953 Ohms */
    244,    /* == -28.7 deg C: Count == 760, R == 164684 Ohms */
    244,    /* == -28.87 deg C: Count == 768, R == 166415 Ohms */
    244,    /* == -29.04 deg C: Count == 776, R == 168146 Ohms */
    244,    /* == -29.21 deg C: Count == 784, R == 169878 Ohms */
    244,    /* == -29.37 deg C: Count == 792, R == 171609 Ohms */
    243,    /* == -29.53 deg C: Count == 800, R == 173340 Ohms */
    243,    /* == -29.69 deg C: Count == 808, R == 175071 Ohms */
    243,    /* == -29.85 deg C: Count == 816, R == 176803 Ohms */
    243,    /* == -30.01 deg C: Count == 824, R == 178534 Ohms */
    243,    /* == -30.16 deg C: Count == 832, R == 180265 Ohms */
    243,    /* == -30.32 deg C: Count == 840, R == 181996 Ohms */
    243,    /* == -30.47 deg C: Count == 848, R == 183728 Ohms */
    242,    /* == -30.62 deg C: Count == 856, R == 185459 Ohms */
    242,    /* == -30.77 deg C: Count == 864, R == 187190 Ohms */
    242,    /* == -30.92 deg C: Count == 872, R == 188921 Ohms */
    242,    /* == -31.06 deg C: Count == 880, R == 190653 Ohms */
    242,    /* == -31.21 deg C: Count == 888, R == 192384 Ohms */
    242,    /* == -31.35 deg C: Count == 896, R == 194115 Ohms */
    242,    /* == -31.49 deg C: Count == 904, R == 195846 Ohms */
    241,    /* == -31.63 deg C: Count == 912, R == 197578 Ohms */
    241,    /* == -31.77 deg C: Count == 920, R == 199309 Ohms */
    241,    /* == -31.91 deg C: Count == 928, R == 201040 Ohms */
    241,    /* == -32.04 deg C: Count == 936, R == 202771 Ohms */
    241,    /* == -32.18 deg C: Count == 944, R == 204502 Ohms */
    241,    /* == -32.31 deg C: Count == 952, R == 206234 Ohms */
    241,    /* == -32.44 deg C: Count == 960, R == 207965 Ohms */
    240,    /* == -32.58 deg C: Count == 968, R == 209696 Ohms */
    240,    /* == -32.71 deg C: Count == 976, R == 211427 Ohms */
    240,    /* == -32.83 deg C: Count == 984, R == 213159 Ohms */
    240,    /* == -32.96 deg C: Count == 992, R == 214890 Ohms */
    240,    /* == -33.09 deg C: Count == 1000, R == 216621 Ohms */
    240,    /* == -33.21 deg C: Count == 1008, R == 218352 Ohms */
    240,    /* == -33.34 deg C: Count == 1016, R == 220084 Ohms */
};

#endif

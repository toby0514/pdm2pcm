//琮閔
// ^
// |
//莓機機

/* Includes ------------------------------------------------------------------*/

#include "OpenPDMFilter.h"
#include "Sinc_coef.h"
#include <stdio.h>
#include <unistd.h>
#include <errno.h>

/* Variables -----------------------------------------------------------------*/
#define PI 3.14159265358979323846

const int64_t sub_const = 1048576; //(sinc[0] + sinc[1] + ... + sinc[383]) / 2

int AddCount = 0, MulCount = 0, DivCount = 0, shiftCount = 0;
int ConvolveCountAdd = 0, ConvolveCountMul = 0, Bandpass_Filter_Init_LP_Mul = 0, Bandpass_Filter_Init_LP_Add = 0, Bandpass_Filter_Init_LP_Div = 0, Bandpass_Filter_Init_HP_Mul = 0, Bandpass_Filter_Init_HP_Add = 0, Bandpass_Filter_Init_HP_Div = 0, CIC_Filter_Init_Add = 0, CIC_Filter_Init_Mul = 0, CIC_Filter_Init_Div = 0, CIC_Filter_Init_Shift = 0, CIC_Filter_Add = 0, Bandpass_Filter_HP_Add = 0, Bandpass_Filter_HP_Mul = 0, Bandpass_Filter_HP_Shift = 0, Bandpass_Filter_LP_Add = 0, Bandpass_Filter_LP_Mul = 0, Bandpass_Filter_LP_Shift = 0, Post_processing_Div = 0, Post_processing_Add = 0, Post_processing_Mul = 0, Open_PDM_Filter_128_Add = 0;
double div_const = 0;

/* Functions -----------------------------------------------------------------*/

int32_t filter_table_mono_128(uint8_t *data, uint8_t sincn)
{
  CIC_Filter_Add += 15;
  return (int32_t)
         sinc_coef[data[0]][0][sincn] +
         sinc_coef[data[1]][1][sincn] +
         sinc_coef[data[2]][2][sincn] +
         sinc_coef[data[3]][3][sincn] +
         sinc_coef[data[4]][4][sincn] +
         sinc_coef[data[5]][5][sincn] +
         sinc_coef[data[6]][6][sincn] +
         sinc_coef[data[7]][7][sincn] +
         sinc_coef[data[8]][8][sincn] +
         sinc_coef[data[9]][9][sincn] +
         sinc_coef[data[10]][10][sincn] +
         sinc_coef[data[11]][11][sincn] +
         sinc_coef[data[12]][12][sincn] +
         sinc_coef[data[13]][13][sincn] +
         sinc_coef[data[14]][14][sincn] +
         sinc_coef[data[15]][15][sincn];
}

int32_t filter_table_stereo_128(uint8_t *data, uint8_t sincn)
{
  return (int32_t)
         sinc_coef[data[0]][0][sincn] +
         sinc_coef[data[2]][1][sincn] +
         sinc_coef[data[4]][2][sincn] +
         sinc_coef[data[6]][3][sincn] +
         sinc_coef[data[8]][4][sincn] +
         sinc_coef[data[10]][5][sincn] +
         sinc_coef[data[12]][6][sincn] +
         sinc_coef[data[14]][7][sincn] +
         sinc_coef[data[16]][8][sincn] +
         sinc_coef[data[18]][9][sincn] +
         sinc_coef[data[20]][10][sincn] +
         sinc_coef[data[22]][11][sincn] +
         sinc_coef[data[24]][12][sincn] +
         sinc_coef[data[26]][13][sincn] +
         sinc_coef[data[28]][14][sincn] +
         sinc_coef[data[30]][15][sincn];
}

int32_t (*filter_tables_128[2])(uint8_t *data, uint8_t sincn) = {filter_table_mono_128, filter_table_stereo_128};

//init
void Bandpass_Filter_Init(TPDMFilter_InitStruct *Param)
{
  // Param->LP_ALFA = (Param->LP_HZ != 0 ? (uint16_t)(Param->LP_HZ * 256 / (Param->LP_HZ + Param->Fs / (2 * PI))) : 0); // LPF濾波係數
  Param->LP_ALFA = (Param->LP_HZ != 0 ? ((float)Param->LP_HZ / (Param->LP_HZ + Param->Fs / (2 * PI))) : 0); // LPF濾波係數
  // Param->LP_ALFA = 194;  //(uint16_t)(Param->LP_HZ * 256 / (Param->LP_HZ + Param->Fs / (2 * PI)))
  fprintf(stderr, "Param->LP_ALFA = %f \n", Param->LP_ALFA);
  fprintf(stderr, "Param->HP_ALFA = %f \n", Param->HP_ALFA);
  Bandpass_Filter_Init_LP_Mul += 2;
  Bandpass_Filter_Init_LP_Add += 1;
  Bandpass_Filter_Init_LP_Div += 2;
  // Param->HP_ALFA = (Param->HP_HZ != 0 ? (uint16_t)(Param->Fs * 256 / (2 * PI * Param->HP_HZ + Param->Fs)) : 0); // HPF濾波係數
  Param->HP_ALFA = (Param->HP_HZ != 0 ? ((float)Param->Fs / (2 * PI * Param->HP_HZ + Param->Fs)) : 0); // HPF濾波係數
  // Param->HP_ALFA = 254;  //(uint16_t)(Param->Fs * 256 / (2 * PI * Param->HP_HZ + Param->Fs))
  fprintf(stderr, "Param->HP_ALFA = %f \n", Param->HP_ALFA);
  Bandpass_Filter_Init_HP_Mul += 3;
  Bandpass_Filter_Init_HP_Add += 1;
  Bandpass_Filter_Init_HP_Div += 1;
}

void CIC_Filter_Init(TPDMFilter_InitStruct *Param)
{
  uint16_t i;
  // uint8_t decimation = Param->Decimation;

  Param->OldOut = Param->OldIn = Param->OldZ = 0;

  for (i = 0; i < SINCN; i++)
  {
    Param->Coef[i] = 0;
  }

  // Param->FilterLen = decimation * SINCN;

  // div_const = sub_const * Param->MaxVolume / 32768 / FILTER_GAIN;
  div_const = (double)(32768 * FILTER_GAIN) / (sub_const * Param->MaxVolume); //改倒數
  div_const = (div_const == 0 ? 1 : div_const);
  // CIC_Filter_Init_Shift += 1;
  CIC_Filter_Init_Div += 1;
  CIC_Filter_Init_Mul += 2;
}

void Open_PDM_Filter_Init(TPDMFilter_InitStruct *Param)
{
  // Bandpass_Filter_Init(Param);
  CIC_Filter_Init(Param);
}

int CIC_Filter(uint8_t *data, TPDMFilter_InitStruct *Param, int64_t Z, uint8_t j)
{
  int64_t Z0, Z1, Z2;

  Z0 = filter_tables_128[j](data, 0);
  Z1 = filter_tables_128[j](data, 1);
  Z2 = filter_tables_128[j](data, 2);

  //  comb filter implement
  Z = Param->Coef[1] + Z2 - sub_const;
  CIC_Filter_Add += 2;
  Param->Coef[1] = Param->Coef[0] + Z1;
  CIC_Filter_Add += 1;
  Param->Coef[0] = Z0;
  return Z;
}

int Bandpass_Filter(TPDMFilter_InitStruct *Param, int64_t Zin, int64_t OldZ, int64_t OldIn, int64_t OldOut)
{
  // OldOut = (Param->HP_ALFA * (OldOut + Zin - OldIn)) >> 8; // 一階RC LPF模型
  OldOut = (Param->HP_ALFA * (OldOut + Zin - OldIn)); // 一階RC LPF模型
  Bandpass_Filter_HP_Add += 2;
  Bandpass_Filter_HP_Mul += 1;
  // Bandpass_Filter_HP_Shift += 1;
  OldIn = Zin;
  // OldZ = ((256 - Param->LP_ALFA) * OldZ + Param->LP_ALFA * OldOut) >> 8; // 一階RC HPF模型
  OldZ = ((1 - Param->LP_ALFA) * OldZ + Param->LP_ALFA * OldOut); // 一階RC HPF模型
  Bandpass_Filter_LP_Add += 2;
  Bandpass_Filter_LP_Mul += 2;
  // Bandpass_Filter_LP_Shift += 1;
  return OldZ;
}

int Post_processing(int64_t Zin, int64_t New_OldZ, uint16_t volume)
{
  Zin = New_OldZ * volume;
  //fprintf(stderr,"%ld\n",Zin);
  Post_processing_Mul += 1;
  //fprintf(stderr,"div_const = %d\n",div_const);
  Zin = RoundDiv(Zin, div_const);
  //fprintf(stderr,"%ld\n",Zin);
  Post_processing_Add += 1;
  Post_processing_Mul += 2;
  Zin = SaturaLH(Zin, -32700, 32700);
  // fprintf(stderr,"%lf\n",(double)Zin);
  // fprintf(stderr,"%ld\n",Zin);
  return Zin;
}

void Open_PDM_Filter_128(uint8_t *data, int16_t *dataOut, uint16_t volume, TPDMFilter_InitStruct *Param)
{
  uint8_t i, data_out_index;
  int64_t Z, Zin, OldZ, OldIn, OldOut, New_OldZ;
  uint8_t channels = Param->In_MicChannels;
  uint8_t data_inc = ((DECIMATION_MAX >> 3) * channels);

  OldOut = Param->OldOut;
  OldIn = Param->OldIn;
  OldZ = Param->OldZ;

#ifdef USE_LUT
  uint8_t j = channels - 1; //j=0
#endif
//轉出16個PCMsample
  for (i = 0, data_out_index = 0; i < Param->nSamples; i++, data_out_index += channels)
  {
    Zin = CIC_Filter(data, Param, Z, j);
    New_OldZ = Bandpass_Filter(Param, Zin, OldZ, OldIn, OldOut);
    dataOut[data_out_index] = Post_processing(Zin, New_OldZ, volume);
    data += data_inc;
    Open_PDM_Filter_128_Add += 1;
  }
  //Init the Oldout, Oldin, OldZ
  Param->OldOut = OldOut;
  Param->OldIn = OldIn;
  Param->OldZ = OldZ;
}

void Operation_Count()
{
  fprintf(stderr, "ConvolveCountAdd = %d times\n", ConvolveCountAdd);
  fprintf(stderr, "ConvolveCountMul = %d times\n", ConvolveCountMul);
  fprintf(stderr, "Bandpass_Filter_Init_LP_Add = %d times\n", Bandpass_Filter_Init_LP_Add);
  fprintf(stderr, "Bandpass_Filter_Init_LP_Mul = %d times\n", Bandpass_Filter_Init_LP_Mul);
  fprintf(stderr, "Bandpass_Filter_Init_LP_Div = %d times\n", Bandpass_Filter_Init_LP_Div);
  fprintf(stderr, "Bandpass_Filter_Init_HP_Add = %d times\n", Bandpass_Filter_Init_HP_Add);
  fprintf(stderr, "Bandpass_Filter_Init_HP_Mul = %d times\n", Bandpass_Filter_Init_HP_Mul);
  fprintf(stderr, "Bandpass_Filter_Init_HP_Div = %d times\n", Bandpass_Filter_Init_HP_Div);
  fprintf(stderr, "CIC_Filter_Init_Add = %d times\n", CIC_Filter_Init_Add);
  fprintf(stderr, "CIC_Filter_Init_Mul = %d times\n", CIC_Filter_Init_Mul);
  fprintf(stderr, "CIC_Filter_Init_Div = %d times\n", CIC_Filter_Init_Div);
  fprintf(stderr, "CIC_Filter_Init_Shift = %d times\n", CIC_Filter_Init_Shift);
  fprintf(stderr, "CIC_Filter_Add = %d times\n", CIC_Filter_Add);
  fprintf(stderr, "Bandpass_Filter_HP_Add = %d times\n", Bandpass_Filter_HP_Add);
  fprintf(stderr, "Bandpass_Filter_HP_Mul = %d times\n", Bandpass_Filter_HP_Mul);
  fprintf(stderr, "Bandpass_Filter_HP_Shift = %d times\n", Bandpass_Filter_HP_Shift);
  fprintf(stderr, "Bandpass_Filter_LP_Add = %d times\n", Bandpass_Filter_LP_Add);
  fprintf(stderr, "Bandpass_Filter_LP_Mul = %d times\n", Bandpass_Filter_LP_Mul);
  fprintf(stderr, "Bandpass_Filter_LP_Shift = %d times\n", Bandpass_Filter_LP_Shift);
  fprintf(stderr, "Post_processing_Add = %d times\n", Post_processing_Add);
  fprintf(stderr, "Post_processing_Mul = %d times\n", Post_processing_Mul);
  fprintf(stderr, "Post_processing_Div = %d times\n", Post_processing_Div);
  fprintf(stderr, "Open_PDM_Filter_128_Add = %d times\n", Open_PDM_Filter_128_Add);
  fprintf(stderr, "AddCount = %d times\n", ConvolveCountAdd + Bandpass_Filter_Init_LP_Add + Bandpass_Filter_Init_HP_Add + CIC_Filter_Init_Add + CIC_Filter_Add + Bandpass_Filter_HP_Add + Bandpass_Filter_LP_Add + Post_processing_Add + Open_PDM_Filter_128_Add);
  fprintf(stderr, "MulCount = %d times\n", ConvolveCountMul + Bandpass_Filter_Init_LP_Mul + Bandpass_Filter_Init_HP_Mul + CIC_Filter_Init_Mul + Bandpass_Filter_HP_Mul + Bandpass_Filter_LP_Mul + Post_processing_Mul);
  fprintf(stderr, "DivCount = %d times\n", Bandpass_Filter_Init_LP_Div + Bandpass_Filter_Init_HP_Div + CIC_Filter_Init_Div + Post_processing_Div);
  fprintf(stderr, "shiftCount = %d times\n", CIC_Filter_Init_Shift + Bandpass_Filter_HP_Shift + Bandpass_Filter_LP_Shift);
}
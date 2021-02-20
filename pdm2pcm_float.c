/**
 *******************************************************************************
 * @file    OpenPDMFilter.c
 * @author  CL
 * @version V1.0.0
 * @date    9-September-2015
 * @brief   Open PDM audio software decoding Library.   
 *          This Library is used to decode and reconstruct the audio signal
 *          produced by ST MEMS microphone (MP45Dxxx, MP34Dxxx). 
 *******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2018 STMicroelectronics</center></h2>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************
 */


/* Includes ------------------------------------------------------------------*/

#include "OpenPDMFilter.h"
#include "Sinc_coef.h"
#include <stdio.h>
#include <unistd.h>
#include <errno.h>

/* Variables -----------------------------------------------------------------*/
int AddCount=0, MulCount=0, DivCount=0,shiftCount=0;
int ConvolveCountAdd=0,ConvolveCountMul=0,Bandpass_Filter_Init_LP_Mul=0,Bandpass_Filter_Init_LP_Add=0,Bandpass_Filter_Init_LP_Div=0,Bandpass_Filter_Init_HP_Mul=0,Bandpass_Filter_Init_HP_Add=0,Bandpass_Filter_Init_HP_Div=0,CIC_Filter_Init_Add=0,CIC_Filter_Init_Mul=0,CIC_Filter_Init_Div=0,CIC_Filter_Init_Shift=0,CIC_Filter_Add=0,Bandpass_Filter_HP_Add=0,Bandpass_Filter_HP_Mul=0,Bandpass_Filter_HP_Shift=0,Bandpass_Filter_LP_Add=0,Bandpass_Filter_LP_Mul=0,Bandpass_Filter_LP_Shift=0,Post_processing_Div=0,Post_processing_Add=0,Post_processing_Mul=0,Open_PDM_Filter_128_Add=0;
uint32_t div_const = 0;
int64_t sub_const = 0;
#ifdef USE_LUT
int32_t lut[256][DECIMATION_MAX / 8][SINCN];
#endif

/* Functions -----------------------------------------------------------------*/

int32_t filter_table_mono_128(uint8_t *data, uint8_t sincn)
{
  return (int32_t)
    lut[data[0]][0][sincn] +
    lut[data[1]][1][sincn] +
    lut[data[2]][2][sincn] +
    lut[data[3]][3][sincn] +
    lut[data[4]][4][sincn] +
    lut[data[5]][5][sincn] +
    lut[data[6]][6][sincn] +
    lut[data[7]][7][sincn] +
    lut[data[8]][8][sincn] +
    lut[data[9]][9][sincn] +
    lut[data[10]][10][sincn] +
    lut[data[11]][11][sincn] +
    lut[data[12]][12][sincn] +
    lut[data[13]][13][sincn] +
    lut[data[14]][14][sincn] +
    lut[data[15]][15][sincn];
    AddCount += 15;
}
int32_t filter_table_stereo_128(uint8_t *data, uint8_t sincn)
{
  return (int32_t)
    lut[data[0]][0][sincn] +
    lut[data[2]][1][sincn] +
    lut[data[4]][2][sincn] +
    lut[data[6]][3][sincn] +
    lut[data[8]][4][sincn] +
    lut[data[10]][5][sincn] +
    lut[data[12]][6][sincn] +
    lut[data[14]][7][sincn] +
    lut[data[16]][8][sincn] +
    lut[data[18]][9][sincn] +
    lut[data[20]][10][sincn] +
    lut[data[22]][11][sincn] +
    lut[data[24]][12][sincn] +
    lut[data[26]][13][sincn] +
    lut[data[28]][14][sincn] +
    lut[data[30]][15][sincn];
}
int32_t (* filter_tables_128[2]) (uint8_t *data, uint8_t sincn) = {filter_table_mono_128, filter_table_stereo_128};

//init
void Bandpass_Filter_Init(TPDMFilter_InitStruct *Param)
{
  Param->OldOut = Param->OldIn = Param->OldZ = 0;
  Param->LP_ALFA = (Param->LP_HZ != 0 ? (uint16_t) (Param->LP_HZ * 256 / (Param->LP_HZ + Param->Fs / (2 * 3.14159))) : 0); // LPF濾波係數
  Bandpass_Filter_Init_LP_Mul += 2;
  Bandpass_Filter_Init_LP_Add += 1;
  Bandpass_Filter_Init_LP_Div += 2;
  Param->HP_ALFA = (Param->HP_HZ != 0 ? (uint16_t) (Param->Fs * 256 / (2 * 3.14159 * Param->HP_HZ + Param->Fs)) : 0); // HPF濾波係數
  Bandpass_Filter_Init_HP_Mul += 3;
  Bandpass_Filter_Init_HP_Add += 1;
  Bandpass_Filter_Init_HP_Div += 1;
}

void CIC_Filter_Init(TPDMFilter_InitStruct *Param)
{  
  uint16_t i, j;
  int64_t sum = 0;
  uint8_t decimation = Param->Decimation;

  sub_const = sum >> 1;
  CIC_Filter_Init_Shift += 1;
  div_const = sub_const * Param->MaxVolume / 32768 / FILTER_GAIN;
  fprintf(stderr,"%ld * %d / 32768 / %d = %d\n",sub_const,Param->MaxVolume,FILTER_GAIN,div_const);
  CIC_Filter_Init_Div += 2;
  CIC_Filter_Init_Mul += 1;
  div_const = (div_const == 0 ? 1 : div_const);
}

void Open_PDM_Filter_Init(TPDMFilter_InitStruct *Param)
{
  Bandpass_Filter_Init(Param);
  CIC_Filter_Init(Param);
}

int CIC_Filter(uint8_t* data, TPDMFilter_InitStruct *Param, int64_t Z, uint8_t j)
{
  int64_t Z0, Z1, Z2;

  // LUT
#ifdef USE_LUT
    Z0 = filter_tables_128[j](data, 0);
    Z1 = filter_tables_128[j](data, 1);
    Z2 = filter_tables_128[j](data, 2);
#else
    Z0 = filter_table(data, 0, Param);
    Z1 = filter_table(data, 1, Param);
    Z2 = filter_table(data, 2, Param);
#endif
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
    OldOut = (Param->HP_ALFA * (OldOut + Zin - OldIn)) >> 8; // 一階RC LPF模型 
    Bandpass_Filter_HP_Add += 2;
    Bandpass_Filter_HP_Mul += 1;
    Bandpass_Filter_HP_Shift += 1;
    OldIn = Zin;
    OldZ = ((256 - Param->LP_ALFA) * OldZ + Param->LP_ALFA * OldOut) >> 8; // 一階RC HPF模型
    Bandpass_Filter_LP_Add += 1;
    Bandpass_Filter_LP_Mul += 2;
    Bandpass_Filter_LP_Shift += 1;
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
    Post_processing_Div += 2;
    Zin = SaturaLH(Zin, -32700, 32700);
	//fprintf(stderr,"%ld\n",Zin);
    return Zin;
}

void Open_PDM_Filter_128(uint8_t* data, int16_t* dataOut, uint16_t volume, TPDMFilter_InitStruct *Param)
{
  uint8_t i,data_out_index;
  int64_t Z,Zin,OldZ,OldIn,OldOut,New_OldZ;
  uint8_t channels = Param->In_MicChannels;
  uint8_t data_inc = ((DECIMATION_MAX >> 3) * channels); 

  OldOut = Param->OldOut;
  OldIn = Param->OldIn;
  OldZ = Param->OldZ;

  #ifdef USE_LUT
  uint8_t j = channels - 1;
  #endif

  for (i = 0, data_out_index = 0; i < Param->nSamples; i++, data_out_index += channels)
  {
    Zin = CIC_Filter(data,Param,Z,j);
    New_OldZ = Bandpass_Filter(Param,Zin,OldZ,OldIn,OldOut);
    dataOut[data_out_index] = Post_processing(Zin,New_OldZ,volume);

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
  fprintf(stderr,"ConvolveCountAdd = %d times\n",ConvolveCountAdd);
  fprintf(stderr,"ConvolveCountMul = %d times\n",ConvolveCountMul);
  fprintf(stderr,"Bandpass_Filter_Init_LP_Add = %d times\n",Bandpass_Filter_Init_LP_Add);
  fprintf(stderr,"Bandpass_Filter_Init_LP_Mul = %d times\n",Bandpass_Filter_Init_LP_Mul);
  fprintf(stderr,"Bandpass_Filter_Init_LP_Div = %d times\n",Bandpass_Filter_Init_LP_Div);
  fprintf(stderr,"Bandpass_Filter_Init_HP_Add = %d times\n",Bandpass_Filter_Init_HP_Add);
  fprintf(stderr,"Bandpass_Filter_Init_HP_Mul = %d times\n",Bandpass_Filter_Init_HP_Mul);
  fprintf(stderr,"Bandpass_Filter_Init_HP_Div = %d times\n",Bandpass_Filter_Init_HP_Div);
  fprintf(stderr,"CIC_Filter_Init_Add = %d times\n",CIC_Filter_Init_Add);
  fprintf(stderr,"CIC_Filter_Init_Mul = %d times\n",CIC_Filter_Init_Mul);
  fprintf(stderr,"CIC_Filter_Init_Div = %d times\n",CIC_Filter_Init_Div);
  fprintf(stderr,"CIC_Filter_Init_Shift = %d times\n",CIC_Filter_Init_Shift);
  fprintf(stderr,"CIC_Filter_Add = %d times\n",CIC_Filter_Add);
  fprintf(stderr,"Bandpass_Filter_HP_Add = %d times\n",Bandpass_Filter_HP_Add);
  fprintf(stderr,"Bandpass_Filter_HP_Mul = %d times\n",Bandpass_Filter_HP_Mul);
  fprintf(stderr,"Bandpass_Filter_HP_Shift = %d times\n",Bandpass_Filter_HP_Shift);
  fprintf(stderr,"Bandpass_Filter_LP_Add = %d times\n",Bandpass_Filter_LP_Add);
  fprintf(stderr,"Bandpass_Filter_LP_Mul = %d times\n",Bandpass_Filter_LP_Mul);
  fprintf(stderr,"Bandpass_Filter_LP_Shift = %d times\n",Bandpass_Filter_LP_Shift);
  fprintf(stderr,"Post_processing_Add = %d times\n",Post_processing_Add);
  fprintf(stderr,"Post_processing_Mul = %d times\n",Post_processing_Mul);
  fprintf(stderr,"Post_processing_Div = %d times\n",Post_processing_Div);
  fprintf(stderr,"Open_PDM_Filter_128_Add = %d times\n",Open_PDM_Filter_128_Add);
  fprintf(stderr,"AddCount = %d times\n", ConvolveCountAdd+Bandpass_Filter_Init_LP_Add+Bandpass_Filter_Init_HP_Add+CIC_Filter_Init_Add+CIC_Filter_Add+Bandpass_Filter_HP_Add+Bandpass_Filter_LP_Add+Post_processing_Add+Open_PDM_Filter_128_Add); 
	fprintf(stderr,"MulCount = %d times\n", ConvolveCountMul+Bandpass_Filter_Init_LP_Mul+Bandpass_Filter_Init_HP_Mul+CIC_Filter_Init_Mul+Bandpass_Filter_HP_Mul+Bandpass_Filter_LP_Mul+Post_processing_Mul);
	fprintf(stderr,"DivCount = %d times\n", Bandpass_Filter_Init_LP_Div+Bandpass_Filter_Init_HP_Div+CIC_Filter_Init_Div+Post_processing_Div);
	fprintf(stderr,"shiftCount = %d times\n", CIC_Filter_Init_Shift+Bandpass_Filter_HP_Shift+Bandpass_Filter_LP_Shift);
}
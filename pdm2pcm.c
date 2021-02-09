/** \file pdm2pcm.c
 * \author david.siorpaes@gmail.com
 *
 * Uses OpenPDM library to decode pdm data coming from standard input
 * and sends PCM data to standard output
 * Example usage:
 *
 * bzcat bellazio.txt.bz2 | ./txt2bin | ./pdm2pcm -f 1024000 -d128 | aplay -fS16_LE -c1 -r8000
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include "OpenPDMFilter.h"
#include <time.h>

int main(int argc, char** argv)
{
	
	int opt, ret, dataCount; // operation, readtxt's data
	int finished = 0;
	int Count=0;
	unsigned int pdmSamplingF, decimationF, pcmSamplingF, pdmBufLen, pcmBufLen; // pdm取樣頻率, Decimation(抽取率), pcm取樣頻率, pdmBuf大小(bit),  pcmBuf大小(bit)
	uint8_t* pdmBuf; // pdmBuf(byte)
	int16_t* pcmBuf; // pcmBuf(-32768 ~ 32767)
	TPDMFilter_InitStruct filter; // Open_PDM_filter 結構(可看.h file)
	clock_t start,finish;
	double  duration;
	/* Get user options */
	pdmSamplingF = decimationF = 0;
	while((opt = getopt (argc, argv, "hf:d:")) != -1){
		switch(opt){
			case 'h':
				printf("%s -h(elp) -f <PDM sampling frequency> -d <decimation factor>\n", argv[0]);
				printf("Example usage: bzcat bellazio.txt.bz2 | ./txt2bin | ./pdm2pcm -f 1024000 -d128 | aplay -fS16_LE -c1 -r8000\n");
				exit(0);
				break;
				
			case 'f':
				pdmSamplingF = atoi(optarg);
				break;
				
			case 'd':
				decimationF = atoi(optarg);
				if(decimationF != 64 && decimationF != 128){
					fprintf(stderr, "Decimation factor must be 64 or 128\n");
					exit(-EINVAL);
				}
				break;

			case '?':
				if(optopt == 'f' || optopt == 'd'){
					fprintf(stderr, "Option %c requires argument\n", optopt);
					exit(-1);
				}
				break;
				
			default:
				break;
		}
	}

	if(decimationF == 0 || pdmSamplingF == 0){
		fprintf(stderr, "Must specify both PDM sampling frequency and decimation factor\n");
		exit(-EINVAL);
	}
	start = clock(); 
	pcmSamplingF = pdmSamplingF/decimationF; // PCM 取樣頻率 = PDM 取樣頻率 / Decimation , ex : 16000 = 1024000 / 128

	/* Allocate buffers to contain 1ms worth of data */
	pdmBufLen = pdmSamplingF/1000; // 1ms pdmBuf 大小(bit)
	pcmBufLen = pdmBufLen/decimationF; // 1ms pcmBuf 大小(bit)
	pdmBuf = malloc(pdmBufLen/8); // 1ms pdmBuf 大小(uint_8_t,byte) 
	if(pdmBuf == NULL){
		fprintf(stderr, "Cannot allocate memory\n");
		exit(-ENOMEM);
	}

	pcmBuf = malloc(sizeof(int16_t)*pcmBufLen); // pcmBuf大小 - 1個pcm為int16_t(-32768 ~ 32767), 以上面例子 pdmBufLen = 1024 則會產生 8 個 pcm, 所以 pcmBuf 大小為 int16_t * 8
		if(pcmBuf == NULL){
		fprintf(stderr, "Cannot allocate memory\n");
		exit(-ENOMEM);
	}
	
	/* Initialize Open PDM library */
	filter.Fs = pcmSamplingF; 
	filter.nSamples = pcmBufLen;
	filter.LP_HZ = pcmSamplingF/2;
	filter.HP_HZ = 10;
	filter.In_MicChannels = 1; 
	filter.Out_MicChannels = 1;
	filter.Decimation = decimationF;
	filter.MaxVolume = 5;
	Open_PDM_Filter_Init(&filter);

	/* Operation */
	while(finished == 0){
		/* Grab 1ms data from stdin */
		dataCount = 0;
		while((dataCount < pdmBufLen/8) && (finished == 0)){
			ret = read(STDIN_FILENO, pdmBuf + dataCount, pdmBufLen/8-dataCount); // STDIN_FILENO : Standard input, pdmBuf + dataCount : 讀出資料的緩衝區, pdmBufLen/8-dataCount : 每次讀取的"位元組"數 , return值 : 返回讀出的位元組數
			if(ret < 0){
				fprintf(stderr, "Error reading from STDIN: %s\n", strerror(errno));
				exit(errno);
			}

			if(ret == 0){
				fprintf(stderr, "Decoding complete!\n");
				finished = 1;
			}

			dataCount += ret;
		}

		/* Decode PDM. Oldest PDM bit is MSB */
		switch(decimationF)
		{
			case 64 :  
			Open_PDM_Filter_64(pdmBuf, pcmBuf, 1, &filter);
			break;

			case 128 : 
			Open_PDM_Filter_128(pdmBuf, pcmBuf, 1, &filter);
			Count +=1;
			break;
		}

		

		/* Emit PCM decoded data to stdout */
		dataCount = 0;
		while(dataCount < sizeof(int16_t)*pcmBufLen){
			ret = write(STDOUT_FILENO, pcmBuf + dataCount, sizeof(int16_t)*pcmBufLen-dataCount);
			if(ret < 0){
				fprintf(stderr, "Error writing to STDOUT: %s\n", strerror(errno));
				exit(errno);
			}
		
			dataCount += ret;
		}
	}
	finish = clock(); 
	duration = (double)(finish - start) / CLOCKS_PER_SEC;   
    printf( "%f seconds\n", duration); 
	// Operation_Count();
	fprintf(stderr,"Count = %d times\n",Count);
	return 0;
}

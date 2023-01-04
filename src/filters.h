/*
 * filters.h
 *
 * Created: 3/17/2011 8:27:37 AM
 *  Author: waltj
 */ 


#ifndef FILTERS_H_
#define FILTERS_H_

#include "ISMatrix.h"
#include "data_sets.h"

//_____ D E F I N I T I O N S ______________________________________________



// Low Pass Filter Coefficients
//   Alpha = (Fc/Fs)/(1 + Fc/Fs)
//   Beta = 1.0 - Alpha
//   100Hz (10ms) Sample Rate
#define	ALPH_100SR_0p001CF	(0.0000099999f)		// 0.001Hz corner freq @ 100Hz sample rate
#define	BETA_100SR_0p001CF	(0.9999900001f)
#define	ALPH_100SR_0p01CF	(0.0000999900f)		// 0.01Hz corner freq @ 100Hz sample rate
#define	BETA_100SR_0p01CF	(0.9999000100f)
#define	ALPH_100SR_0p1CF	(0.0009990010f)		// 0.1Hz corner freq @ 100Hz sample rate
#define	BETA_100SR_0p1CF	(0.9990009990f)
#define	ALPH_100SR_1CF    	(0.0099009901f)		// 1Hz corner freq @ 100Hz sample rate
#define	BETA_100SR_1CF    	(0.9900990099f)
#define	ALPH_100SR_2CF    	(0.0196078431f)		// 2Hz corner freq @ 100Hz sample rate
#define	BETA_100SR_2CF    	(0.9803921569f)
#define	ALPH_100SR_5CF    	(0.0476190476f)		// 5Hz corner freq @ 100Hz sample rate
#define	BETA_100SR_5CF    	(0.9523809524f)
#define	ALPH_100SR_10CF   	(0.0909090909f)		// 10Hz corner freq @ 100Hz sample rate
#define	BETA_100SR_10CF   	(0.9090909091f)
#define	ALPH_100SR_20CF   	(0.1666666667f)		// 20Hz corner freq @ 100Hz sample rate
#define	BETA_100SR_20CF   	(0.8333333333f)
#define	ALPH_100SR_30CF   	(0.2307692308f)		// 30Hz corner freq @ 100Hz sample rate
#define	BETA_100SR_30CF   	(0.7692307692f)
#define	ALPH_100SR_50CF   	(0.3333333333f)		// 50Hz corner freq @ 100Hz sample rate
#define	BETA_100SR_50CF   	(0.6666666667f)
#define	ALPH_100SR_75CF   	(0.4285714286f)		// 75Hz corner freq @ 100Hz sample rate
#define	BETA_100SR_75CF   	(0.5714285714f)
#define	ALPH_100SR_100CF   	(0.5000000000f)		// 100Hz corner freq @ 100Hz sample rate
#define	BETA_100SR_100CF   	(0.5000000000f)


// Low Pass Filter Coefficients
//   Alpha = (Fc/Fs)/(1 + Fc/Fs)
//   Beta = 1.0 - Alpha
//   200Hz (5ms) Sample Rate
#define	ALPH_200SR_0p001CF	(0.0000050000f)		// 0.001Hz corner freq @ 200Hz sample rate
#define	BETA_200SR_0p001CF	(0.9999950000f)
#define	ALPH_200SR_0p01CF	(0.0000499975f)		// 0.01Hz corner freq @ 200Hz sample rate
#define	BETA_200SR_0p01CF	(0.9999500025f)
#define	ALPH_200SR_0p1CF	(0.0004997501f)		// 0.1Hz corner freq @ 200Hz sample rate
#define	BETA_200SR_0p1CF	(0.9995002499f)
#define	ALPH_200SR_1CF    	(0.0049751244f)		// 1Hz corner freq @ 200Hz sample rate
#define	BETA_200SR_1CF    	(0.9950248756f)
#define	ALPH_200SR_2CF    	(0.0099009901f)		// 2Hz corner freq @ 200Hz sample rate
#define	BETA_200SR_2CF    	(0.9900990099f)
#define	ALPH_200SR_5CF    	(0.0243902439f)		// 5Hz corner freq @ 200Hz sample rate
#define	BETA_200SR_5CF    	(0.9756097561f)
#define	ALPH_200SR_10CF   	(0.0476190476f)		// 10Hz corner freq @ 200Hz sample rate
#define	BETA_200SR_10CF   	(0.9523809524f)
#define	ALPH_200SR_20CF   	(0.0909090909f)		// 20Hz corner freq @ 200Hz sample rate
#define	BETA_200SR_20CF   	(0.9090909091f)
#define	ALPH_200SR_30CF   	(0.1304347826f)		// 30Hz corner freq @ 200Hz sample rate
#define	BETA_200SR_30CF   	(0.8695652174f)
#define	ALPH_200SR_50CF   	(0.2000000000f)		// 50Hz corner freq @ 200Hz sample rate
#define	BETA_200SR_50CF   	(0.8000000000f)
#define	ALPH_200SR_75CF   	(0.2727272727f)		// 75Hz corner freq @ 200Hz sample rate
#define	BETA_200SR_75CF   	(0.7272727273f)
#define	ALPH_200SR_100CF	(0.3333333333f)		// 100Hz corner freq @ 200Hz sample rate
#define	BETA_200SR_100CF	(0.6666666667f)
#define	ALPH_200SR_150CF   	(0.4285714286f)		// 150Hz corner freq @ 200Hz sample rate
#define	BETA_200SR_150CF   	(0.5714285714f)
#define	ALPH_200SR_200CF   	(0.5000000000f)		// 200Hz corner freq @ 200Hz sample rate
#define	BETA_200SR_200CF   	(0.5000000000f)


// Low Pass Filter Coefficients
//   Alpha = (Fc/Fs)/(1 + Fc/Fs)
//   Beta = 1.0 - Alpha
//   250Hz (4ms) Sample Rate
#define	ALPH_250SR_0p001CF	(0.0000040000f)		// 0.001Hz corner freq @ 250Hz sample rate
#define	BETA_250SR_0p001CF	(0.9999960000f)
#define	ALPH_250SR_0p01CF	(0.0000399984f)		// 0.01Hz corner freq @ 250Hz sample rate
#define	BETA_250SR_0p01CF	(0.9999600016f)
#define	ALPH_250SR_0p1CF	(0.0003998401f)		// 0.1Hz corner freq @ 250Hz sample rate
#define	BETA_250SR_0p1CF	(0.9996001599f)
#define	ALPH_250SR_1CF    	(0.0039840637f)		// 1Hz corner freq @ 250Hz sample rate
#define	BETA_250SR_1CF    	(0.9960159363f)
#define	ALPH_250SR_2CF    	(0.0079365079f)		// 2Hz corner freq @ 250Hz sample rate
#define	BETA_250SR_2CF    	(0.9920634921f)
#define	ALPH_250SR_5CF    	(0.0196078431f)		// 5Hz corner freq @ 250Hz sample rate
#define	BETA_250SR_5CF    	(0.9803921569f)
#define	ALPH_250SR_10CF   	(0.0384615385f)		// 10Hz corner freq @ 250Hz sample rate
#define	BETA_250SR_10CF   	(0.9615384615f)
#define	ALPH_250SR_20CF   	(0.0740740741f)		// 20Hz corner freq @ 250Hz sample rate
#define	BETA_250SR_20CF   	(0.9259259259f)
#define	ALPH_250SR_30CF   	(0.1071428571f)		// 30Hz corner freq @ 250Hz sample rate
#define	BETA_250SR_30CF   	(0.8928571429f)
#define	ALPH_250SR_50CF   	(0.1666666667f)		// 50Hz corner freq @ 250Hz sample rate
#define	BETA_250SR_50CF   	(0.8333333333f)
#define	ALPH_250SR_75CF   	(0.2307692308f)		// 75Hz corner freq @ 250Hz sample rate
#define	BETA_250SR_75CF   	(0.7692307692f)
#define	ALPH_250SR_100CF	(0.2857142857f)		// 100Hz corner freq @ 250Hz sample rate
#define	BETA_250SR_100CF	(0.7142857143f)
#define	ALPH_250SR_150CF	(0.3750000000f)		// 150Hz corner freq @ 250Hz sample rate
#define	BETA_250SR_150CF	(0.6250000000f)
#define	ALPH_250SR_187CF   	(0.4285714286f)		// 187Hz corner freq @ 250Hz sample rate
#define	BETA_250SR_187CF   	(0.5714285714f)
#define	ALPH_250SR_250CF   	(0.5000000000f)		// 250Hz corner freq @ 250Hz sample rate
#define	BETA_250SR_250CF   	(0.5000000000f)


// Low Pass Filter Coefficients
//   Alpha = (Fc/Fs)/(1 + Fc/Fs)
//   Beta = 1.0 - Alpha
//   333Hz (3ms) Sample Rate
#define	ALPH_333SR_0p001CF	(0.0000030000f)		// 0.001Hz corner freq @ 333Hz sample rate
#define	BETA_333SR_0p001CF	(0.9999970000f)
#define	ALPH_333SR_0p01CF	(0.0000299991f)		// 0.01Hz corner freq @ 333Hz sample rate
#define	BETA_333SR_0p01CF	(0.9999700009f)
#define	ALPH_333SR_0p1CF	(0.0002999100f)		// 0.1Hz corner freq @ 333Hz sample rate
#define	BETA_333SR_0p1CF	(0.9997000900f)
#define	ALPH_333SR_1CF    	(0.0029910269f)		// 1Hz corner freq @ 333Hz sample rate
#define	BETA_333SR_1CF    	(0.9970089731f)
#define	ALPH_333SR_2CF    	(0.0059642147f)		// 2Hz corner freq @ 333Hz sample rate
#define	BETA_333SR_2CF    	(0.9940357853f)
#define	ALPH_333SR_5CF    	(0.0147783251f)		// 5Hz corner freq @ 333Hz sample rate
#define	BETA_333SR_5CF    	(0.9852216749f)
#define	ALPH_333SR_10CF   	(0.0291262136f)		// 10Hz corner freq @ 333Hz sample rate
#define	BETA_333SR_10CF   	(0.9708737864f)
#define	ALPH_333SR_20CF   	(0.0566037736f)		// 20Hz corner freq @ 333Hz sample rate
#define	BETA_333SR_20CF   	(0.9433962264f)
#define	ALPH_333SR_30CF   	(0.0825688073f)		// 30Hz corner freq @ 333Hz sample rate
#define	BETA_333SR_30CF   	(0.9174311927f)
#define	ALPH_333SR_50CF   	(0.1304347826f)		// 50Hz corner freq @ 333Hz sample rate
#define	BETA_333SR_50CF   	(0.8695652174f)
#define	ALPH_333SR_75CF   	(0.1836734694f)		// 75Hz corner freq @ 333Hz sample rate
#define	BETA_333SR_75CF   	(0.8163265306f)
#define	ALPH_333SR_100CF	(0.2307692308f)		// 100Hz corner freq @ 333Hz sample rate
#define	BETA_333SR_100CF	(0.7692307692f)
#define	ALPH_333SR_150CF	(0.3103448276f)		// 150Hz corner freq @ 333Hz sample rate
#define	BETA_333SR_150CF	(0.6896551724f)
#define	ALPH_333SR_200CF	(0.3750000000f)		// 200Hz corner freq @ 333Hz sample rate
#define	BETA_333SR_200CF	(0.6250000000f)
#define	ALPH_333SR_300CF   	(0.4736842105f)		// 300Hz corner freq @ 333Hz sample rate
#define	BETA_333SR_300CF   	(0.5263157895f)
#define	ALPH_333SR_400CF   	(0.5454545455f)		// 400Hz corner freq @ 333Hz sample rate
#define	BETA_333SR_400CF   	(0.4545454545f)


// Low Pass Filter Coefficients
//   Alpha = (Fc/Fs)/(1 + Fc/Fs)
//   Beta = 1.0 - Alpha
//   400Hz (2.5ms) Sample Rate
#define	ALPH_400SR_0p001CF	(0.0000025000f)		// 0.001Hz corner freq @ 400Hz sample rate
#define	BETA_400SR_0p001CF	(0.9999975000f)
#define	ALPH_400SR_0p01CF	(0.0000249994f)		// 0.01Hz corner freq @ 400Hz sample rate
#define	BETA_400SR_0p01CF	(0.9999750006f)
#define	ALPH_400SR_0p1CF	(0.0002499375f)		// 0.1Hz corner freq @ 400Hz sample rate
#define	BETA_400SR_0p1CF	(0.9997500625f)
#define	ALPH_400SR_1CF    	(0.0024937656f)		// 1Hz corner freq @ 400Hz sample rate
#define	BETA_400SR_1CF    	(0.9975062344f)
#define	ALPH_400SR_2CF    	(0.0049751244f)		// 2Hz corner freq @ 400Hz sample rate
#define	BETA_400SR_2CF    	(0.9950248756f)
#define	ALPH_400SR_5CF    	(0.0123456790f)		// 5Hz corner freq @ 400Hz sample rate
#define	BETA_400SR_5CF    	(0.9876543210f)
#define	ALPH_400SR_10CF   	(0.0243902439f)		// 10Hz corner freq @ 400Hz sample rate
#define	BETA_400SR_10CF   	(0.9756097561f)
#define	ALPH_400SR_20CF   	(0.0476190476f)		// 20Hz corner freq @ 400Hz sample rate
#define	BETA_400SR_20CF   	(0.9523809524f)
#define	ALPH_400SR_30CF   	(0.0697674419f)		// 30Hz corner freq @ 400Hz sample rate
#define	BETA_400SR_30CF   	(0.9302325581f)
#define	ALPH_400SR_50CF   	(0.1111111111f)		// 50Hz corner freq @ 400Hz sample rate
#define	BETA_400SR_50CF   	(0.8888888889f)
#define	ALPH_400SR_75CF   	(0.1578947368f)		// 75Hz corner freq @ 400Hz sample rate
#define	BETA_400SR_75CF   	(0.8421052632f)
#define	ALPH_400SR_100CF	(0.2000000000f)		// 100Hz corner freq @ 400Hz sample rate
#define	BETA_400SR_100CF	(0.8000000000f)
#define	ALPH_400SR_150CF	(0.2727272727f)		// 150Hz corner freq @ 400Hz sample rate
#define	BETA_400SR_150CF	(0.7272727273f)
#define	ALPH_400SR_200CF	(0.3333333333f)		// 200Hz corner freq @ 400Hz sample rate
#define	BETA_400SR_200CF	(0.6666666667f)
#define	ALPH_400SR_300CF   	(0.4285714286f)		// 300Hz corner freq @ 400Hz sample rate
#define	BETA_400SR_300CF   	(0.5714285714f)
#define	ALPH_400SR_400CF   	(0.5000000000f)		// 400Hz corner freq @ 400Hz sample rate
#define	BETA_400SR_400CF   	(0.5000000000f)


// Low Pass Filter Coefficients
//   Alpha = (Fc/Fs)/(1 + Fc/Fs)
//   Beta = 1.0 - Alpha
//   500Hz (2ms) Sample Rate
#define	ALPH_500SR_0p001CF	(0.0000020000f)		// 0.001Hz corner freq @ 500Hz sample rate
#define	BETA_500SR_0p001CF	(0.9999980000f)
#define	ALPH_500SR_0p01CF	(0.0000199996f)		// 0.01Hz corner freq @ 500Hz sample rate
#define	BETA_500SR_0p01CF	(0.9999800004f)
#define	ALPH_500SR_0p1CF	(0.0001999600f)		// 0.1Hz corner freq @ 500Hz sample rate
#define	BETA_500SR_0p1CF	(0.9998000400f)
#define	ALPH_500SR_0p2CF	(0.0003999200f)		// 0.1Hz corner freq @ 500Hz sample rate
#define	BETA_500SR_0p2CF	(0.9996000800f)
#define	ALPH_500SR_1CF    	(0.0019960080f)		// 1Hz corner freq @ 500Hz sample rate
#define	BETA_500SR_1CF    	(0.9980039920f)
#define	ALPH_500SR_2CF    	(0.0039840637f)		// 2Hz corner freq @ 500Hz sample rate
#define	BETA_500SR_2CF    	(0.9960159363f)
#define	ALPH_500SR_5CF    	(0.0099009901f)		// 5Hz corner freq @ 500Hz sample rate
#define	BETA_500SR_5CF    	(0.9900990099f)
#define	ALPH_500SR_10CF   	(0.0196078431f)		// 10Hz corner freq @ 500Hz sample rate
#define	BETA_500SR_10CF   	(0.9803921569f)
#define	ALPH_500SR_20CF   	(0.0384615385f)		// 20Hz corner freq @ 500Hz sample rate
#define	BETA_500SR_20CF   	(0.9615384615f)
#define	ALPH_500SR_30CF   	(0.0566037736f)		// 30Hz corner freq @ 500Hz sample rate
#define	BETA_500SR_30CF   	(0.9433962264f)
#define	ALPH_500SR_50CF   	(0.0909090909f)		// 50Hz corner freq @ 500Hz sample rate
#define	BETA_500SR_50CF   	(0.9090909091f)
#define	ALPH_500SR_75CF   	(0.1304347826f)		// 75Hz corner freq @ 500Hz sample rate
#define	BETA_500SR_75CF   	(0.8695652174f)
#define	ALPH_500SR_100CF	(0.1666666667f)		// 100Hz corner freq @ 500Hz sample rate
#define	BETA_500SR_100CF	(0.8333333333f)
#define	ALPH_500SR_150CF	(0.2307692308f)		// 150Hz corner freq @ 500Hz sample rate
#define	BETA_500SR_150CF	(0.7692307692f)
#define	ALPH_500SR_200CF	(0.2857142857f)		// 200Hz corner freq @ 500Hz sample rate
#define	BETA_500SR_200CF	(0.7142857143f)
#define	ALPH_500SR_250CF	(0.3333333333f)		// 250Hz corner freq @ 500Hz sample rate
#define	BETA_500SR_250CF	(0.6666666667f)
#define	ALPH_500SR_375CF   	(0.4285714286f)		// 375Hz corner freq @ 500Hz sample rate
#define	BETA_500SR_375CF   	(0.5714285714f)
#define	ALPH_500SR_500CF   	(0.5000000000f)		// 500Hz corner freq @ 500Hz sample rate
#define	BETA_500SR_500CF   	(0.5000000000f)


// Low Pass Filter Coefficients
//   Alpha = (Fc/Fs)/(1 + Fc/Fs)
//   Beta = 1.0 - Alpha
//   1000Hz (1ms) Sample Rate
#define	ALPH_1000SR_0p001CF	(0.0000010000f)		// 0.001Hz corner freq @ 1000Hz sample rate
#define	BETA_1000SR_0p001CF	(0.9999990000f)
#define	ALPH_1000SR_0p01CF	(0.0000099999f)		// 0.01Hz corner freq @ 1000Hz sample rate
#define	BETA_1000SR_0p01CF	(0.9999900001f)
#define	ALPH_1000SR_0p1CF	(0.0000999900f)		// 0.1Hz corner freq @ 1000Hz sample rate
#define	BETA_1000SR_0p1CF	(0.9999000100f)
#define	ALPH_1000SR_1CF    	(0.0009990010f)		// 1Hz corner freq @ 1000Hz sample rate
#define	BETA_1000SR_1CF    	(0.9990009990f)
#define	ALPH_1000SR_2CF    	(0.0019960080f)		// 2Hz corner freq @ 1000Hz sample rate
#define	BETA_1000SR_2CF    	(0.9980039920f)
#define	ALPH_1000SR_5CF    	(0.0049751244f)		// 5Hz corner freq @ 1000Hz sample rate
#define	BETA_1000SR_5CF    	(0.9950248756f)
#define	ALPH_1000SR_10CF   	(0.0099009901f)		// 10Hz corner freq @ 1000Hz sample rate
#define	BETA_1000SR_10CF   	(0.9900990099f)
#define	ALPH_1000SR_20CF   	(0.0196078431f)		// 20Hz corner freq @ 1000Hz sample rate
#define	BETA_1000SR_20CF   	(0.9803921569f)
#define	ALPH_1000SR_30CF   	(0.0291262136f)		// 30Hz corner freq @ 1000Hz sample rate
#define	BETA_1000SR_30CF   	(0.9708737864f)
#define	ALPH_1000SR_50CF   	(0.0476190476f)		// 50Hz corner freq @ 1000Hz sample rate
#define	BETA_1000SR_50CF   	(0.9523809524f)
#define	ALPH_1000SR_75CF   	(0.0697674419f)		// 75Hz corner freq @ 1000Hz sample rate
#define	BETA_1000SR_75CF   	(0.9302325581f)
#define	ALPH_1000SR_100CF	(0.0909090909f)		// 100Hz corner freq @ 1000Hz sample rate
#define	BETA_1000SR_100CF	(0.9090909091f)
#define	ALPH_1000SR_150CF	(0.1304347826f)		// 150Hz corner freq @ 1000Hz sample rate
#define	BETA_1000SR_150CF	(0.8695652174f)
#define	ALPH_1000SR_200CF	(0.1666666667f)		// 200Hz corner freq @ 1000Hz sample rate
#define	BETA_1000SR_200CF	(0.8333333333f)
#define	ALPH_1000SR_250CF	(0.2000000000f)		// 250Hz corner freq @ 1000Hz sample rate
#define	BETA_1000SR_250CF	(0.8000000000f)
#define	ALPH_1000SR_400CF	(0.2857142857f)		// 400Hz corner freq @ 1000Hz sample rate
#define	BETA_1000SR_400CF	(0.7142857143f)
#define	ALPH_1000SR_500CF	(0.3333333333f)		// 500Hz corner freq @ 1000Hz sample rate
#define	BETA_1000SR_500CF	(0.6666666667f)
#define	ALPH_1000SR_750CF   (0.4285714286f)		// 750Hz corner freq @ 1000Hz sample rate
#define	BETA_1000SR_750CF   (0.5714285714f)
#define	ALPH_1000SR_1000CF  (0.50000000000f)	// 1000Hz corner freq @ 1000Hz sample rate
#define	BETA_1000SR_1000CF  (0.50000000000f)


// Low Pass Filter Coefficients
//   Alpha = (Fc/Fs)/(1 + Fc/Fs)
//   Beta = 1.0 - Alpha
//   2000Hz (0.5ms) Sample Rate
#define	ALPH_2000SR_0p001CF	(0.0000005000f)		// 0.001Hz corner freq @ 2000Hz sample rate
#define	BETA_2000SR_0p001CF	(0.9999995000f)
#define	ALPH_2000SR_0p01CF	(0.0000050000f)		// 0.01Hz corner freq @ 2000Hz sample rate
#define	BETA_2000SR_0p01CF	(0.9999950000f)
#define	ALPH_2000SR_0p1CF	(0.0000499975f)		// 0.1Hz corner freq @ 2000Hz sample rate
#define	BETA_2000SR_0p1CF	(0.9999500025f)
#define	ALPH_2000SR_1CF    	(0.0004997501f)		// 1Hz corner freq @ 2000Hz sample rate
#define	BETA_2000SR_1CF    	(0.9995002499f)
#define	ALPH_2000SR_2CF    	(0.0009990010f)		// 2Hz corner freq @ 2000Hz sample rate
#define	BETA_2000SR_2CF    	(0.9990009990f)
#define	ALPH_2000SR_5CF    	(0.0024937656f)		// 5Hz corner freq @ 2000Hz sample rate
#define	BETA_2000SR_5CF    	(0.9975062344f)
#define	ALPH_2000SR_10CF   	(0.0049751244f)		// 10Hz corner freq @ 2000Hz sample rate
#define	BETA_2000SR_10CF   	(0.9950248756f)
#define	ALPH_2000SR_20CF   	(0.0099009901f)		// 20Hz corner freq @ 2000Hz sample rate
#define	BETA_2000SR_20CF   	(0.9900990099f)
#define	ALPH_2000SR_30CF   	(0.0147783251f)		// 30Hz corner freq @ 2000Hz sample rate
#define	BETA_2000SR_30CF   	(0.9852216749f)
#define	ALPH_2000SR_50CF   	(0.0243902439f)		// 50Hz corner freq @ 2000Hz sample rate
#define	BETA_2000SR_50CF   	(0.9756097561f)
#define	ALPH_2000SR_75CF   	(0.0361445783f)		// 75Hz corner freq @ 2000Hz sample rate
#define	BETA_2000SR_75CF   	(0.9638554217f)
#define	ALPH_2000SR_100CF	(0.0476190476f)		// 100Hz corner freq @ 2000Hz sample rate
#define	BETA_2000SR_100CF	(0.9523809524f)
#define	ALPH_2000SR_150CF	(0.0697674419f)		// 150Hz corner freq @ 2000Hz sample rate
#define	BETA_2000SR_150CF	(0.9302325581f)
#define	ALPH_2000SR_200CF	(0.0909090909f)		// 200Hz corner freq @ 2000Hz sample rate
#define	BETA_2000SR_200CF	(0.9090909091f)
#define	ALPH_2000SR_250CF	(0.1111111111f)		// 250Hz corner freq @ 2000Hz sample rate
#define	BETA_2000SR_250CF	(0.8888888889f)
#define	ALPH_2000SR_400CF	(0.1666666667f)		// 400Hz corner freq @ 2000Hz sample rate
#define	BETA_2000SR_400CF	(0.8333333333f)
#define	ALPH_2000SR_500CF	(0.2000000000f)		// 500Hz corner freq @ 2000Hz sample rate
#define	BETA_2000SR_500CF	(0.8000000000f)
#define	ALPH_2000SR_1000CF	(0.3333333333f)		// 1000Hz corner freq @ 2000Hz sample rate
#define	BETA_2000SR_1000CF	(0.6666666667f)
#define	ALPH_2000SR_1500CF  (0.4285714286f)		// 1500Hz corner freq @ 2000Hz sample rate
#define	BETA_2000SR_1500CF  (0.5714285714f)
#define	ALPH_2000SR_2000CF  (0.50000000000f)	// 2000Hz corner freq @ 2000Hz sample rate
#define	BETA_2000SR_2000CF  (0.50000000000f)


// Low Pass Filter Coefficients
//   Alpha = (Fc/Fs)/(1 + Fc/Fs)
//   Beta = 1.0 - Alpha
//   4000Hz (0.25ms) Sample Rate
#define	ALPH_4000SR_0p001CF	(0.0000002500f)		// 0.001Hz corner freq @ 4000Hz sample rate
#define	BETA_4000SR_0p001CF	(0.9999997500f)
#define	ALPH_4000SR_0p01CF	(0.0000025000f)		// 0.01Hz corner freq @ 4000Hz sample rate
#define	BETA_4000SR_0p01CF	(0.9999975000f)
#define	ALPH_4000SR_0p1CF	(0.0000249994f)		// 0.1Hz corner freq @ 4000Hz sample rate
#define	BETA_4000SR_0p1CF	(0.9999750006f)
#define	ALPH_4000SR_1CF    	(0.0002499375f)		// 1Hz corner freq @ 4000Hz sample rate
#define	BETA_4000SR_1CF    	(0.9997500625f)
#define	ALPH_4000SR_2CF    	(0.0004997501f)		// 2Hz corner freq @ 4000Hz sample rate
#define	BETA_4000SR_2CF    	(0.9995002499f)
#define	ALPH_4000SR_5CF    	(0.0012484395f)		// 5Hz corner freq @ 4000Hz sample rate
#define	BETA_4000SR_5CF    	(0.9987515605f)
#define	ALPH_4000SR_10CF   	(0.0024937656f)		// 10Hz corner freq @ 4000Hz sample rate
#define	BETA_4000SR_10CF   	(0.9975062344f)
#define	ALPH_4000SR_20CF   	(0.0049751244f)		// 20Hz corner freq @ 4000Hz sample rate
#define	BETA_4000SR_20CF   	(0.9950248756f)
#define	ALPH_4000SR_30CF   	(0.0074441687f)		// 30Hz corner freq @ 4000Hz sample rate
#define	BETA_4000SR_30CF   	(0.9925558313f)
#define	ALPH_4000SR_50CF   	(0.0123456790f)		// 50Hz corner freq @ 4000Hz sample rate
#define	BETA_4000SR_50CF   	(0.9876543210f)
#define	ALPH_4000SR_75CF   	(0.0184049080f)		// 75Hz corner freq @ 4000Hz sample rate
#define	BETA_4000SR_75CF   	(0.9815950920f)
#define	ALPH_4000SR_100CF	(0.0243902439f)		// 100Hz corner freq @ 4000Hz sample rate
#define	BETA_4000SR_100CF	(0.9756097561f)
#define	ALPH_4000SR_150CF	(0.0361445783f)		// 150Hz corner freq @ 4000Hz sample rate
#define	BETA_4000SR_150CF	(0.9638554217f)
#define	ALPH_4000SR_200CF	(0.0476190476f)		// 200Hz corner freq @ 4000Hz sample rate
#define	BETA_4000SR_200CF	(0.9523809524f)
#define	ALPH_4000SR_250CF	(0.0588235294f)		// 250Hz corner freq @ 4000Hz sample rate
#define	BETA_4000SR_250CF	(0.9411764706f)
#define	ALPH_4000SR_400CF	(0.0909090909f)		// 400Hz corner freq @ 4000Hz sample rate
#define	BETA_4000SR_400CF	(0.9090909091f)
#define	ALPH_4000SR_500CF	(0.1111111111f)		// 500Hz corner freq @ 4000Hz sample rate
#define	BETA_4000SR_500CF	(0.8888888889f)
#define	ALPH_4000SR_1000CF	(0.2000000000f)		// 1000Hz corner freq @ 4000Hz sample rate
#define	BETA_4000SR_1000CF	(0.8000000000f)
#define	ALPH_4000SR_2000CF	(0.3333333333f)		// 2000Hz corner freq @ 4000Hz sample rate
#define	BETA_4000SR_2000CF	(0.6666666667f)
#define	ALPH_4000SR_3000CF  (0.4285714286f)		// 3000Hz corner freq @ 4000Hz sample rate
#define	BETA_4000SR_3000CF  (0.5714285714f)
#define	ALPH_4000SR_4000CF  (0.50000000000f)	// 4000Hz corner freq @ 4000Hz sample rate
#define	BETA_4000SR_4000CF  (0.50000000000f)


// Low Pass Filter Coefficients
//   Alpha = (Fc/Fs)/(1 + Fc/Fs)
//   Beta = 1.0 - Alpha
//   8000Hz (0.125ms) Sample Rate
#define	ALPH_8000SR_0p001CF	(0.0000001250f)		// 0.001Hz corner freq @ 8000Hz sample rate
#define	BETA_8000SR_0p001CF	(0.9999998750f)
#define	ALPH_8000SR_0p01CF	(0.0000012500f)		// 0.01Hz corner freq @ 8000Hz sample rate
#define	BETA_8000SR_0p01CF	(0.9999987500f)
#define	ALPH_8000SR_0p1CF	(0.0000124998f)		// 0.1Hz corner freq @ 8000Hz sample rate
#define	BETA_8000SR_0p1CF	(0.9999875002f)
#define	ALPH_8000SR_1CF    	(0.0001249844f)		// 1Hz corner freq @ 8000Hz sample rate
#define	BETA_8000SR_1CF    	(0.9998750156f)
#define	ALPH_8000SR_2CF    	(0.0002499375f)		// 2Hz corner freq @ 8000Hz sample rate
#define	BETA_8000SR_2CF    	(0.9997500625f)
#define	ALPH_8000SR_5CF    	(0.0006246096f)		// 5Hz corner freq @ 8000Hz sample rate
#define	BETA_8000SR_5CF    	(0.9993753904f)
#define	ALPH_8000SR_10CF   	(0.0012484395f)		// 10Hz corner freq @ 8000Hz sample rate
#define	BETA_8000SR_10CF   	(0.9987515605f)
#define	ALPH_8000SR_20CF   	(0.0024937656f)		// 20Hz corner freq @ 8000Hz sample rate
#define	BETA_8000SR_20CF   	(0.9975062344f)
#define	ALPH_8000SR_30CF   	(0.0037359900f)		// 30Hz corner freq @ 8000Hz sample rate
#define	BETA_8000SR_30CF   	(0.9962640100f)
#define	ALPH_8000SR_50CF   	(0.0062111801f)		// 50Hz corner freq @ 8000Hz sample rate
#define	BETA_8000SR_50CF   	(0.9937888199f)
#define	ALPH_8000SR_75CF   	(0.0092879257f)		// 75Hz corner freq @ 8000Hz sample rate
#define	BETA_8000SR_75CF   	(0.9907120743f)
#define	ALPH_8000SR_100CF	(0.0123456790f)		// 100Hz corner freq @ 8000Hz sample rate
#define	BETA_8000SR_100CF	(0.9876543210f)
#define	ALPH_8000SR_150CF	(0.0184049080f)		// 150Hz corner freq @ 8000Hz sample rate
#define	BETA_8000SR_150CF	(0.9815950920f)
#define	ALPH_8000SR_200CF	(0.0243902439f)		// 200Hz corner freq @ 8000Hz sample rate
#define	BETA_8000SR_200CF	(0.9756097561f)
#define	ALPH_8000SR_250CF	(0.0303030303f)		// 250Hz corner freq @ 8000Hz sample rate
#define	BETA_8000SR_250CF	(0.9696969697f)
#define	ALPH_8000SR_400CF	(0.0476190476f)		// 400Hz corner freq @ 8000Hz sample rate
#define	BETA_8000SR_400CF	(0.9523809524f)
#define	ALPH_8000SR_500CF	(0.0588235294f)		// 500Hz corner freq @ 8000Hz sample rate
#define	BETA_8000SR_500CF	(0.9411764706f)
#define	ALPH_8000SR_1000CF	(0.1111111111f)		// 1000Hz corner freq @ 8000Hz sample rate
#define	BETA_8000SR_1000CF	(0.8888888889f)
#define	ALPH_8000SR_2000CF	(0.2000000000f)		// 2000Hz corner freq @ 8000Hz sample rate
#define	BETA_8000SR_2000CF	(0.8000000000f)
#define	ALPH_8000SR_4000CF	(0.3333333333f)		// 4000Hz corner freq @ 8000Hz sample rate
#define	BETA_8000SR_4000CF	(0.6666666667f)
#define	ALPH_8000SR_6000CF  (0.4285714286f)		// 6000Hz corner freq @ 8000Hz sample rate
#define	BETA_8000SR_6000CF  (0.5714285714f)
#define	ALPH_8000SR_8000CF  (0.50000000000f)	// 8000Hz corner freq @ 8000Hz sample rate
#define	BETA_8000SR_8000CF  (0.50000000000f)




// #define IIR_BIT_SHIFT	8
// #define ADC_TO_IIR		256			// = 2^IIR_BIT_SHIFT
// #define IIR_TO_ADC		0.00390625	// = 1/ADC_TO_IIR
// #define I_ALPHA			9			// = TsFc/(1 + TsFc)*ALPHA_PLUS_BETA, sample period (Ts) and corner freq (Fc).  0.001*100/(1 + 0.001*100)*100 = 0.0909
// #define I_ALPHA_X		2304		// = I_ALPHA*ADC_TO_IIR
// #define I_BETA			91			// = ALPHA_PLUS_BETA - I_ALPHA
// #define ALPHA_PLUS_BETA	100

#define ACCUM_WORD_NBITS			32		// Bit size of IIR accumulator
#define MAX_NUMBER_IIR_CHANNELS		10

typedef struct
{   
	float Fs;						// (Hz) sample frequency
	float Fc;						// (Hz) corner frequency
	unsigned int n_channels;		// number of channels in filter (i.e. ADC channels)
	unsigned int input_size;		// input buffer length (= n_channels * n_input_samples)
	unsigned int sig_word_nbits;		// (bits) input signal resolution bit size
	
 	unsigned int bit_shift;			// bits to shift ADC int to IIR fixed point
// 	int gama_x;						// = alpha_x + beta_x
// 	unsigned int adc_to_iir;		// = 2^bit_shift
	float iir_to_adc;				// = 1/adc_to_iir
// 	unsigned int alpha;				// = TsFc/(1 + TsFc)*alpha_plus_beta, sample period (Ts) and corner freq (Fc).  0.001*100/(1 + 0.001*100)*100 = 0.0909
	int alpha_x;					// = alpha*adc_to_iir
	int beta;						// = alpha_plus_beta - alpha;
} iir_options_t;



typedef struct
{   
	iir_options_t opt;
	int accum[MAX_NUMBER_IIR_CHANNELS];
} iif_filter_t;


typedef struct  
{   
    int         count;              // Sample count
    double      mean;               // input average
} rmean_filter_t;


//_____ P R O T O T Y P E S ________________________________________________

void init_iir_filter(iif_filter_t *f);

void iir_filter_u16(iif_filter_t *f, unsigned short input[], float output[]);
void iir_filter_s16(iif_filter_t *f, short input[], float output[]);


/** 
 * \brief Running Average Filter
 *  A running average of the input array is collected in the mean array.  Filter
 *  is reset when sampleCount equals 0.
 *
 * \param mean          Average of input
 * \param input         Floating point value to be included in the average.
 * \param arraySize     Array length of mean and input arrays.
 * \param sampleCount   Sample number of input.  0 causes filter to be reset.
 */
void running_mean_filter( float mean[], float input[], int arraySize, int sampleCount );


/** 
 * \brief Running Average Filter (double)
 *  A running average of the input array is collected in the mean array.  Filter
 *  is reset when sampleCount equals 0.
 *
 * \param mean          Average of input
 * \param input         Double (float 64) value to be included in the average.
 * \param arraySize     Array length of mean and input arrays.
 * \param sampleCount   Sample number of input.  0 causes filter to be reset.
 */
void running_mean_filter_f64( double mean[], float input[], int arraySize, int sampleCount );

/**
 * \brief Recursive Moving Average and Variance Filter
 * Recursive computation of expected moving average and variance given their previous
 * values, new element in the set, number of elements in the set (window size) and
 * assuming that one of the elements in the set is removed when new one is
 * added (i.e. fixed window size).
 * Reference: http://math.stackexchange.com/questions/1063962/how-can-i-recursively-approximate-a-moving-average-and-standard-deviation
 *
 * \param mean          Moving average of the set
 * \param var           Moving variance of the set
 * \param input         Floating point value added to the set
  * \param sampleCount   Number of samples in the sliding window
*/
void recursive_moving_mean_var_filter(float *mean, float *var, float input, int sampleCount);


// Look for error in dual IMU data
void errorCheckImu3(imu3_t *di);

// Condense triple IMUs down to one IMU
int tripleToSingleImu(imu_t *result, const imu3_t *di);
int tripleToSingleImuExc(imu_t *result, const imu3_t *di, bool *exclude); // for individual IMU exclusion
void tripleToSingleImuAxis(imu_t* result, const imu3_t* di, bool exclude_gyro[3], bool exclude_acc[3], int iaxis);  // for individual gyro/accelerometer (per axis) exclusion

// Duplicate one IMU to triple IMUs
void singleToTripleImu(imu3_t *result, imu_t *imu);

// Convert integrated IMU to IMU. 0 on success, -1 on failure.
int preintegratedImuToIMU(imu_t *imu, const pimu_t *imuInt);
int imuToPreintegratedImu(pimu_t *pImu, const imu_t *imu, float dt);


/** 
 * \brief Compute coning and sculling integrals from gyro and accelerometer samples
 *
 * \param output			Coning and sculling integral
 * \param imu			Gyro and accelerometer sample.
 * \param imuLast		Previous gyro and accelerometer sample.
 */
void integratePimu( pimu_t *output, imu_t *imu, imu_t *imuLast );

// Set integral, time, and status to zero
void zeroPimu( pimu_t *pimu );

/** 
 * \brief Find alpha and beta parameters for single pole Low-Pass filter
 *
 * \param dt            (sec) Update period
 * \param cornerFreq    (Hz) Low-pass filter corner frequency
 * \param alpha         Filter alpha parameter (input gain)
 * \param beta          Filter beta parameter (memory gain)
 */
static __inline void lpf_alpha_beta( float dt, float cornerFreqHz, float *alpha, float *beta )
{
    float dc = dt * cornerFreqHz;
    *alpha  = dc / (1.0f + dc);
    *beta   = 1.0f - *alpha;
}


// LPF zero order:											val = beta*lastVal + alpha*input
#define O0_LP_FILTER(val,input,alph,beta)					(val = (((beta)*(val)) + ((alph)*(input))))

// LPF first order model coefficient:						c = beta*c + alph*((input - val) / dt)		(dt long)
// LPF input into state estimate:							val = beta*((val) + c*dt) + alph*input		(dt short)
#define O1_LP_FILTER(val,input,alph,beta,c,dt)				{ c = beta*c + alph*((input-val)/dt);   val = beta*(val + c*dt) + alph*input; }


// Propagate filter estimate when no input is available
#define O1X_LP_FILTER_NO_INPUT(val,input,c,dt)				{ val = val + c*dt; }						// (shorter dt)
	
// LPF first order model coefficient:						c = beta*c + alph*((input - val2) / dt2)
// LPF input into state estimate:							val = val2 = beta*((val2) + c*dt2) + alph*input
#define O1X_LP_FILTER(val,val2,input,alph,beta,c,dt2)		{ c = beta*c + alph*((input-val2)/dt2);   val = val2 = beta*(val2 + c*dt2) + alph*input; }


#define O0_LPF_VEC3(val,input,alph,beta)				   {O0_LP_FILTER(val[0],input[0],alph,beta); \
															O0_LP_FILTER(val[1],input[1],alph,beta); \
															O0_LP_FILTER(val[2],input[2],alph,beta);}
#define O1X_LPF_VEC3_NO_INPUT(val,input,c,dt)			   {O1X_LP_FILTER_NO_INPUT(val[0],input[0],c[0],dt); \
															O1X_LP_FILTER_NO_INPUT(val[1],input[1],c[1],dt); \
															O1X_LP_FILTER_NO_INPUT(val[2],input[2],c[2],dt);}
#define O1X_LPF_VEC3(val,val2,input,alph,beta,c,dt2)	   {O1X_LP_FILTER(val[0],val2[0],input[0],alph,beta,c[0],dt2); \
															O1X_LP_FILTER(val[1],val2[1],input[1],alph,beta,c[1],dt2); \
															O1X_LP_FILTER(val[2],val2[2],input[2],alph,beta,c[2],dt2);}

static __inline void lpf_filter(float *val, float input, float dt, float cornerFreqHz)
{
    float alph, beta;
	lpf_alpha_beta(dt, cornerFreqHz, &alph, &beta);
	O0_LP_FILTER(*val, input, alph, beta);
}


#if 0
/** 
 * \brief Find alpha and beta parameters for single pole Low-Pass filter
 *
 * \param val           Filter output and running value
 * \param input			Filter input
 * \param *alph			Filter alpha pointer (input gain)
 * \param *beta         Filter beta  pointer (memory gain)
 * \param *c1			Filter first order coefficient pointer
 * \param dt			Delta time between last and current input
 */
static __inline void O1_LP_filter( float *val, float input, float alph, float beta, float *c1, float dt )
{
	// Estimate next model coefficient:				d1 = (input - val) / dt
	// LPF this coefficient:					c1 = beta*c1 + alph*d1
	*c1 = beta*(*c1) + alph*((input - (*val)) / dt);
	
	// Current state estimate:						est = (last val) + c1*dt
	// LPF input into state estimate:				val = beta*est + alph*input
	(*val) = beta*((*val) + (*c1)*dt) + alph*input;
}
#endif





#endif /* FILTERS_H_ */

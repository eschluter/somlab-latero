#ifndef LATERO_TESTPATTERN_H 
#define LATERO_TESTPATTERN_H


// this is ported from parallel port example code: latero.c, latero.h and main.c
// TODO: to be merged together

#define LATERO_NB_PINS_X 8
#define LATERO_NB_PINS_Y 8
#define LATERO_NB_PINS (LATERO_NB_PINS_X*LATERO_NB_PINS_Y)

#define MILLISECPERSEC 1000
#define EPOCH 116444736000000000ULL

//Method to write the stimulus array
void latero_write_ether(latero_t* platero, double frame[LATERO_NB_PINS]);

//Moving top half to the right and bottom half to the left for 5 seconds
void TestSplit1(latero_t* platero);

//Moving top half to the left and bottom half to the right for 5 seconds
void TestSplit2(latero_t* platero);

//Running test pattern on first and last pins: upper-left and lower-right corners (5 seconds at 10 Hz)
void TestFirstlast(latero_t* platero);

//Running test pattern on all pins (10 seconds at 30 Hz)
void TestAllpin(latero_t* platero);

//Running test pattern on rows
void TestRow(latero_t* platero);

//Running test pattern on columns
void TestCol(latero_t* platero);

//	Please orient the Latero such that its cable is to the left.
void TestInit();

#endif
#ifndef FEATURES_H
#define FEATURES_H

#define WINDOW_SIZE 200
#define OVERLAP_SIZE 0

void extractFeatures(float* x, float* y, float* z, float* features, int size);
float findMean(float *arr, long n);
float findSTD(float *arr, long n);
float findRMS(float *arr, long n);
float findMax(float *arr, long n);
float findMin(float *arr, long n);
float findRange(float *arr, long n);
float findMedian(float *arr, long n);
float findIQR(float *arr, long n);
float findVar(float *arr, long n);
float findSkew(float *arr, long n);
float findKurtosis(float *arr, long n);
float findMobility(float *arr, long n);
float findComplexity(float *arr, long n);

#endif // FEATURES_H

//
// Created by robit on 21. 11. 3..
//

#ifndef MISSION_CONTROL_LIDAR_2D_PROCESSING_H
#define MISSION_CONTROL_LIDAR_2D_PROCESSING_H

#define PI 3.14159265359

#include <iostream>
#include <math.h>
#include <vector>

using namespace std;

typedef double tfloat;
typedef vector<tfloat> Ranges;

namespace ld2
{

void distBandPassFilter(Ranges &ranges, tfloat min, tfloat max);

void generateGaussianMask(Ranges &mask, double sigma,int size = -1);
//when size==-1 , it automatically decides size to be 4*sigma

void convolute(const Ranges &a, const Ranges &b, Ranges &out);

void diff(const Ranges &input, Ranges& output);

void findZeroCrossing(const Ranges &input, Ranges output);

void printRanges(const Ranges ranges);
}

#endif //MISSION_CONTROL_LIDAR_2D_PROCESSING_H

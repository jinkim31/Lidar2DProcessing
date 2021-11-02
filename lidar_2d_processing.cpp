//
// Created by robit on 21. 11. 3..
//

#include "lidar_2d_processing.h"

void ld2::distBandPassFilter(Ranges &ranges, tfloat min, tfloat max)
{
    Ranges::iterator iter;
    for (iter = ranges.begin(); iter != ranges.end(); iter++)
    {
        if(*iter < min) *iter = min;
        else if(max < *iter) *iter = max;
    }
}

void ld2::generateGaussianMask(Ranges &mask, double sigma, int size)
{
    if(size == -1)
    {
        mask.resize(sigma*4+1);
    }
    else
    {
        mask.resize(size);
    }

    Ranges::iterator iter;

    int i;
    double cumulative = 0;
    for (iter = mask.begin(), i =-mask.size()/2+1; iter != mask.end(); iter++, i++)
    {
        tfloat x = i;
        *iter = exp(-x*x/(2*sigma*sigma));
        cumulative += *iter;
    }
    for (iter = mask.begin(), i =-mask.size()/2+1; iter != mask.end(); iter++, i++)
    {
        *iter = *iter / cumulative;
    }
}

void ld2::convolute(const Ranges &a, const Ranges &b, Ranges &out)
{
    out.resize(a.size());
    Ranges::const_iterator iterJ;
    tfloat sum;
    int i, j;
    for (i=0; i<=a.size() - b.size(); i++)
    {
        sum = 0;
        for (iterJ = b.begin(), j=0; iterJ != b.end(); iterJ++, j++)
        {
            sum += a[i+j] * *iterJ;
        }
        out[i+b.size()/2] = sum;
    }
    for (i=0; i<=b.size()/2; i++)
    {
        out[i] = a[i];
    }
    for (i=a.size() - b.size()/2; i<=a.size(); i++)
    {
        out[i] = a[i];
    }
}

void ld2::diff(const Ranges &input, Ranges &output)
{
    output.resize(input.size());

    output[0] = 0;

    Ranges::const_iterator iter;
    int i;
    for(iter = output.begin()+1, i=1; iter != output.end(); iter++, i++)
    {
        output[i] = input[i] - input[i-1];
    }
}

void ld2::findZeroCrossing(const Ranges &input, Ranges output)
{

}

void ld2::printRanges(const Ranges ranges)
{
    Ranges::const_iterator iter;

    cout<<"[";
    for(iter = ranges.begin(); iter != ranges.end(); iter++)
    {
        cout<<*iter<<", ";
    }
    cout<<"]"<<endl;
}


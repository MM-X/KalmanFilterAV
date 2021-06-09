#include <iostream>
#include "KalmanFilter/Kalman.h"
using namespace std;

int main(int argc, char** argv)
{  
    float V_measure[25] = {0,1,3,4,7,10,16,13,17,14,18,13,17,19,16,24,26,25,27,30,23,17,11,7,3};
    float V_filter[25] = {0};
    
    Kalman kalmanfilter;
    kalmanfilter.setQvelocity(0.1f);
    kalmanfilter.setQacc(0.1f);
    kalmanfilter.setRmeasure(3.0f);
    kalmanfilter.setVelocity(0.0f);
    kalmanfilter.setAcc(1.0f);
    cout << "V_mesured" << "\t" << "V_filter" << endl;
    
    for(int i = 0; i < 25; ++i){
        V_filter[i] = kalmanfilter.getVelocity(V_measure[i], 1.0f);
        cout << V_measure[i] << "\t" << V_filter[i] << endl;
    }
    cout << "Finished!" << endl;
    cout << "Press 1 and enter to close" << endl;
    int j;
    cin >> j;
    return 0;
}
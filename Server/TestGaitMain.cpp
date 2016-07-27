#include <iostream>
#include <iomanip>
#include "NormalWalk.h"

using namespace std;

int main(int argc, char** argv)
{
    double legPos[18];
    NormalWalk::NormalWalker wker;
    
    wker.Initialize();

    for(int i = 0; i < 20000; i++)
    {
        double timeNow = i / 1000.0;
        
        if (i == 800)
        {
            wker.Start(timeNow);
        }
        if (i == 2000)
        {
            NormalWalk::GaitParameter adjParam;
            adjParam.stepHeight = 0.1;
            adjParam.velocity = 0.4;
            wker.AdjustGaitParam(timeNow, adjParam);
        }
        if (i == 16000)
        {
            wker.Stop(timeNow);
        }

        wker.GaitGenerator(timeNow, legPos);

        cout << std::fixed << std::setprecision(12) << timeNow << ' ';

        for(int j = 0; j < 6; j++)
        {
            for(int k = 0; k < 3; k++)
            {
                cout << std::fixed << std::setprecision(12) << legPos[j*3 + k] << ' ';
            }
        }
        cout << endl;
    }
    return 0;
}

#include "NormalWalk.h"
#include <iostream>

using Eigen::Vector3d;

namespace NormalWalk
{

NormalWalker::NormalWalker()
{
    m_state = GAIT_STATE::UNREADY;
}

int NormalWalker::Initialize()
{
    m_totalPeroid = 2.4;
    m_paramAdjustingTime = 2 * m_totalPeroid;
    m_currentParam.stepHeight = 0.08;
    m_currentParam.velocity = 0;
    m_startTimeLastStep = 0;

    m_standHeight = 0.85;
    m_initBodyPos.setZero();
    m_initBodyOri.setZero();

    m_initFootTipPos[0] << -0.24, -m_standHeight, -0.65;
    m_initFootTipPos[1] <<-0.48, -m_standHeight, -0.00;
    m_initFootTipPos[2] << -0.24, -m_standHeight,  0.65;
    m_initFootTipPos[3] <<  0.24, -m_standHeight, -0.65;
    m_initFootTipPos[4] << 0.48, -m_standHeight, -0.00;
    m_initFootTipPos[5] <<  0.24, -m_standHeight,  0.65;

    // positions of the leg group 1 and 2
    m_legGroupPos[0].setZero();
    m_legGroupPos[1].setZero();

    // the leg NO. IDs of each group
    m_legIDInGroup[0] << 0, 2, 4; // Group A
    m_legIDInGroup[1] << 1, 3, 5; // Group B

    m_state = GAIT_STATE::READY;

    return 0;
}

int NormalWalker::Start(double timeNow)
{
    if (m_state == GAIT_STATE::READY)
    {
        m_startTimeLastStep = timeNow;
        m_state = GAIT_STATE::STARTED;
    }
    return 0;
}

int NormalWalker::Stop(double timeNow)
{
    if (m_state == GAIT_STATE::STARTED)
    {
        m_stopRequestTime = timeNow;
        m_state = GAIT_STATE::STOPPING;
        m_originParam = m_currentParam;
        m_desireParam = m_currentParam;
        m_desireParam.velocity = 0;
    }
    return 0;
}

int NormalWalker::AdjustGaitParam(double timeNow, GaitParameter desireParam)
{
    if (m_state == GAIT_STATE::STARTED)
    {
        m_adjustStartTime = timeNow;
        m_state = GAIT_STATE::ADJUSTING;
        m_originParam = m_currentParam;
        m_desireParam = desireParam;
    }
    return 0;
}

int NormalWalker::GaitGenerator(double timeNow, double *legFootTipPosition)
{
    VirtualLegGaitGenerator(timeNow);

    CalculateFootTipPositions();

    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < 3; j++)
        {
            legFootTipPosition[i*3 + j] = m_footTipPos[i](j);
        }
    }
    return 0;
}

int NormalWalker::CalculateFootTipPositions()
{
    for(int i = 0; i < 3; i++)
    {
        // Calcualte the foot tip positions of A legs
        int legid = m_legIDInGroup[0](i);
        m_footTipPos[legid] = m_initFootTipPos[legid] + m_legGroupPos[0];
        
        // Calcualte the foot tip positions of B legs
        legid = m_legIDInGroup[1](i);
        m_footTipPos[legid] = m_initFootTipPos[legid] + m_legGroupPos[1];
    }

    return 0;
}

int NormalWalker::VirtualLegGaitGenerator(double timeNow)
{
    if (m_state == GAIT_STATE::READY || m_state == GAIT_STATE::UNREADY)
    {
        // stay at the init position
        return 0;
    }

    if (m_state == GAIT_STATE::ADJUSTING) // Gradually change the parameter, second-order differentiable must be kept
    {
        double timeRatio = (timeNow - m_adjustStartTime) / m_paramAdjustingTime;
        if (timeRatio > 1)
        {
            m_state = GAIT_STATE::STARTED; // return to STARTED after param adjustment
        }
        else 
        {
            GradualAdjust(timeRatio);
        }
    }

    bool waitForStepOverThenStop = false;
    if (m_state == GAIT_STATE::STOPPING)
    {
        double timeRatio = (timeNow - m_stopRequestTime) / m_paramAdjustingTime;
        if (timeRatio < 1)
        {
            GradualAdjust(timeRatio);
        }
        else 
        {
            waitForStepOverThenStop = true;
        }
    }

    double timeFromStart = timeNow - m_startTimeLastStep;
    double phase = timeFromStart/m_totalPeroid;

    if (phase >= 1.0) // Restart new step
    {
        phase -= 1;
        m_startTimeLastStep += m_totalPeroid;
        timeFromStart = timeNow - m_startTimeLastStep;

        if (waitForStepOverThenStop)
        {
            m_state = GAIT_STATE::STOPPED;
        }
    }

    if (phase > 0.5) // A swing, B support
    {
        HipTrjGenerator((phase - 0.5) * m_totalPeroid,
                m_currentParam.velocity, m_currentParam.stepHeight, m_legGroupPos[0], m_legGroupPos[1]);
    }
    else // B swing, A support
    {
        HipTrjGenerator(phase * m_totalPeroid,
                m_currentParam.velocity, m_currentParam.stepHeight, m_legGroupPos[1], m_legGroupPos[0]);
    }


    return 0;
}

int NormalWalker::TrjPatternGenerator(
        double timeRatio, 
        double stepLength, 
        double stepHeight, 
        double tacc, double tdec, 
        Eigen::Vector3d& trjPoint)
{
    // accel in the first tacc time, and deccel in the last tacc time
    double adjustcoefficient = 0.28;
    double theta = PivotGenerator(timeRatio, tacc, tdec) * M_PI;

    double height = stepHeight * sin(theta);
    double length = stepLength / 2 * (1 - cos(theta));

    if (length < stepLength / 2)
    {
        // adjust the shape of the first half trajectory
        double origindist = stepLength / 2 - length;
        double newdist = origindist * (1 - adjustcoefficient * (height / stepHeight));
        length = stepLength / 2 - newdist;
    }
    else
    {// adjust the shape of the second half trajectory
        double origindist = length - stepLength / 2;
        double newdist = origindist * (1 - adjustcoefficient * (height / stepHeight));
        length = stepLength / 2 + newdist;
    }
    trjPoint(2) = -length; // -Z axis is the forward direction
    trjPoint(1) = height;
    return 0;
}

double NormalWalker::PivotGenerator(double timeRatio, double Tacc, double Tdec)
{
    double Pratio = 0;
    if (Tacc + Tdec > 1.0)
        return 0;
    else
    {
        double Tcon = 1.0 - Tacc - Tdec;
        double Racc = Tacc*Tacc / Tdec / Tdec;
        double Ka = 1.0 / (1.0 / 3.0*Tacc*Tacc*Tacc
                + 1.0 / 2.0*Tacc*Tacc*(1 - Tacc - Tdec)
                + 1.0 / 3.0*Tacc*Tacc*Tdec);
        double Kd = Ka*Racc;
        if (timeRatio <= Tacc)
        {
            Pratio = 1.0 / 2.0*Ka*Tacc*timeRatio*timeRatio - 1.0 / 6.0*Ka*timeRatio*timeRatio*timeRatio;
        }
        else if (timeRatio>Tacc + Tcon)
        {
            Pratio = 1.0 / 3.0*Tacc*Tacc*Tacc*Ka
                + 1.0 / 2.0*Tacc*Tacc*Ka*(1 - Tacc - Tdec)
                - ((Ka*Tacc*Tacc)*(-1.0 + timeRatio + Tdec)
                        *(1 + timeRatio*timeRatio + 2.0*timeRatio*(-1 + Tdec) - 2.0*Tdec*(1 + Tdec)))
                / (6.0*Tdec*Tdec);
        }
        else
        {
            Pratio = 1.0 / 2.0*Ka*Tacc*Tacc*Tacc - 1.0 / 6.0*Ka*Tacc*Tacc*Tacc + 1.0 / 2.0*Ka*Tacc*Tacc*(timeRatio - Tacc);
        }
    }
    return Pratio;
}

int NormalWalker::HipTrjGenerator(
        double timeFromLastHalfStep, 
        double currentBodyVel, 
        double stepHeight, 
        Eigen::Vector3d& currentPosGSW,
        Eigen::Vector3d& currentPosGSP
        )
{
    double halfPeriod = m_totalPeroid / 2;
    double stepLength = currentBodyVel * m_totalPeroid;
    double timeRatio = timeFromLastHalfStep / halfPeriod;

    // Generate Swing leg group's trj
    TrjPatternGenerator(timeRatio, stepLength, stepHeight, 0.5, 0.5, currentPosGSW);
    // -Z axis is the forward direction
    currentPosGSW(2) -=  -timeFromLastHalfStep * currentBodyVel - halfPeriod * currentBodyVel / 2;

    // Generate Supporting leg group's trj
    // -Z axis is the forward direction
    currentPosGSP(2) = timeFromLastHalfStep * currentBodyVel - halfPeriod * currentBodyVel / 2;
    currentPosGSP(1) = 0;
    return 0;
}

int NormalWalker::GradualAdjust(double timeRatio)
{
    double adjRatio = (1 - cos(timeRatio * M_PI)) / 2;
    m_currentParam.velocity = m_originParam.velocity * (1 - adjRatio) + m_desireParam.velocity * adjRatio;
    m_currentParam.stepHeight = m_originParam.stepHeight * (1 - adjRatio) + m_desireParam.stepHeight * adjRatio;
    return 0;
}

}

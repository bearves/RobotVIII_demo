#ifndef NORMAL_WALK_H
#define NORMAL_WALK_H

#include <Eigen/Dense>

namespace NormalWalk
{

    class GaitParameter
    {
        public:
            double velocity;
            double stepHeight;
    };

    class NormalWalker
    {
        public:
            enum class GAIT_STATE
            {
                UNREADY   = 0,
                READY     = 1,
                STARTED   = 2,
                ADJUSTING = 3,
                STOPPING  = 4,
                STOPPED   = 5
            };

            NormalWalker();

            int Initialize();

            int Start(double timeNow);
            int Stop(double timeNow);

            int AdjustGaitParam(double timeNow, GaitParameter desireParam);

            int GaitGenerator(double timeNow, double *legFootTipPosition);

            Eigen::Vector3d* GetLegGroupPos()
            {
                return m_legGroupPos;
            }

            GAIT_STATE GetState()
            {
                return m_state;
            }

            GaitParameter GetCurrentParameter()
            {
                return m_currentParam;
            }

        private:
            int VirtualLegGaitGenerator(double timeNow);
            int CalculateFootTipPositions();
            int InverseKinematics();

            // Generate the trajectory pattern in the Cartesian space, w.r.t. ground
            int TrjPatternGenerator(
                    double timeRatio,
                    double stepLength,
                    double stepHeight,
                    double tacc, double tdec,
                    Eigen::Vector3d& trjPoint);

            // Generate the phase pivot for trajectory pattern
            double PivotGenerator(double timeRatio, double Tacc = 0.5, double Tdec = 0.5);

            // Generate the trajectory w.r.t. hip joint coordinate
            int HipTrjGenerator(
                    double timeFromLastHalfStep,
                    double currentBodyVel,
                    double stepHeight,
                    Eigen::Vector3d& currentPosGSW,
                    Eigen::Vector3d& currentPosGSP);

            // Gradually ajust the current gait param from the original to the desired one
            int GradualAdjust(double timeRatio);

            GAIT_STATE m_state;
            double m_startTimeLastStep;
            double m_stopRequestTime;

            double m_paramAdjustingTime;
            double m_adjustStartTime;

            double m_totalPeroid;
            GaitParameter m_currentParam;
            GaitParameter m_originParam;
            GaitParameter m_desireParam;

            // positions of the leg group 1 and 2
            Eigen::Vector3d m_legGroupPos[2];
            Eigen::Vector3i m_legIDInGroup[2];
            
            // positions of the initial foot tips w.r.t. body COM
            Eigen::Vector3d m_initFootTipPos[6];

            // init body position w.r.t. global coords
            double m_standHeight;
            Eigen::Matrix3d m_initBodyPos;
            Eigen::Matrix3d m_initBodyOri;

            // positions of the foot tips w.r.t. body COM
            Eigen::Vector3d m_footTipPos[6];
    };

}
#endif

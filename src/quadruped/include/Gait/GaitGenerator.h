#ifndef GAITGENERATOR_H
#define GAITGENERATOR_H

#include "Gait/WaveGenerator.h"
#include "Gait/FeetEndCal.h"


/*cycloid gait*/
class GaitGenerator{
public:
    GaitGenerator(CtrlComponents *ctrlComp);
    ~GaitGenerator();
    void setGait(Vec2 vxyGoalGlobal, float dYawGoal, float gaitHeight);
    void setGait_swimming(Vec2 vxyGoalGlobal, float dYawGoal, float dPitchGoal);

    void run(Vec34 &feetPos, Vec34 &feetVel);
    void run_swimming_F(Vec34 &feetPos, Vec34 &feetVel);
    void run_swimming_B(Vec34 &feetPos, Vec34 &feetVel);

    Vec3 getFootPos(int i);
    Vec3 getFootPos_swimming_F(int i);
    Vec3 getFootPos_swimming_B(int i);

    Vec3 getFootVel(int i);
    Vec3 getFootVel_swimming_F(int i);
    Vec3 getFootVel_swimming_B(int i);  

    void restart();
private:
    float cycloidXYPosition(float startXY, float endXY, float phase);
    float cycloidXYVelocity(float startXY, float endXY, float phase);
    float cycloidZPosition(float startZ, float height, float phase);
    float cycloidZVelocity(float height, float phase);

    WaveGenerator *_waveG;
    Estimator *_est;
    FeetEndCal *_feetCal;
    QuadrupedRobot *_robModel;
    LowlevelState *_state;
    float _gaitHeight;

    Vec2 _vxyGoal;

    float _dYawGoal;
    float _dPitchGoal;

    Vec4 *_phase, _phasePast;
    VecInt4 *_contact;
    Vec34 _startP, _endP, _idealP, _pastP;
    bool _firstRun;
};

#endif  // GAITGENERATOR_H
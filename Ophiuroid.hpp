//
//  Ophiuroid.hpp
//  Master
//
//  Created by 増田貴行 on 2017/10/31.
//  Copyright © 2017年 増田貴行. All rights reserved.
//

#ifndef Ophiuroid_hpp
#define Ophiuroid_hpp

#include <stdio.h>
#include "Starfish.h"
#include "GA2.h"
#include "Master.hpp"
#include "GAMaster.hpp"

class Ophiuroid : public Starfish {
    
private:
    int leg_state[NUM_LEGS];
    int turn_direction[NUM_LEGS];
    GAparameter m_param;
    
public:
    Ophiuroid(GAparameter);
    Ophiuroid(GAparameter, Starfish*);

    void idle();
    void idleDemo();
    bool checkState();
    void create();
    void initSF();
    float evalue();
    btRigidBody* createRigidBody(btScalar, const btTransform&, btCollisionShape*, int);
    void setMotorTarget(double);
    void setMotorTarget2(double);
    void calcMotorTarget(int, int = 0, float = 0);
    void setParam(GAparameter p) {m_param = p;}
};

#endif /* Ophiuroid_hpp */


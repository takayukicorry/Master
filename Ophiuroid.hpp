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

class Ophiuroid : public Starfish {

private:
    std::map<int, btTypedConstraint*> m_joints;
    std::map<int, btRigidBody*> m_bodies;
    std::map<int, btRotationalLimitMotor*> m_motor1;//handle
    std::map<int, btRotationalLimitMotor*> m_motor2;//wheel
    std::map<int, btCollisionShape*> m_shapes;
    int leg_state[NUM_LEGS];
    int turn_direction[NUM_LEGS];
    GAparameter m_param;
    
public:
    Ophiuroid(GAparameter);
    
    void idle();
    bool checkState();
    void create();
    float evalue();
    float evalue2();
    btRigidBody* createRigidBody(btScalar, const btTransform&, btCollisionShape*);
    void setMotorTarget(double);
    void setMotorTarget2(double);
    void setParam(GAparameter p) {m_param = p;}
    btTypedConstraint** GetJoints() {return &m_joints[0];}
    btRigidBody** GetBodies(){return &m_bodies[0];}
    btRotationalLimitMotor** GetMotor1() {return &m_motor1[0];}
    btRotationalLimitMotor** GetMotor2() {return &m_motor2[0];}
};

#endif /* Ophiuroid_hpp */

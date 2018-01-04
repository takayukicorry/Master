//
//  Ophiuroid3.hpp
//  Master
//
//  Created by 増田貴行 on 2018/01/03.
//  Copyright © 2018年 増田貴行. All rights reserved.
//

#ifndef Ophiuroid3_hpp
#define Ophiuroid3_hpp

#include <stdio.h>
#include "Starfish.h"
#include "Master.hpp"
#include "GAMaster.hpp"

class Ophiuroid3 : public Starfish {
    
private:
    std::map<int, btRotationalLimitMotor* > motor_tX;
    std::map<int, btRotationalLimitMotor* > motor_tY;
    std::map<int, btRotationalLimitMotor* > motor_tZ;
    std::map<int, btRotationalLimitMotor* > motor_to_groundY;
    std::map<int, btRotationalLimitMotor* > motor_to_groundZ;
    std::map<int, bool > motor_state;
    std::map<int, bool > TF_Contact;
    std::map<int, btRigidBody* > TF_object;
    std::map<int, int > dl_time;
    std::map<int, int> InitTime_tf;
    std::map<int, int> ResumeTime_ground;
    std::map<int, int> Init_tf;
    std::map<int, btGeneric6DofSpringConstraint*> TF_constraint;
    std::map<int, btUniversalConstraint*> TF_constraint_ground;
    std::map<int, btVector3> TF_axis_direction;
    std::map<int, btVector3> TF_pos;

    std::vector<btRigidBody* > bodies_tf;
    std::vector<btTypedConstraint* > constraints;

public:
    Ophiuroid3(GAparameter);
    
    void initSF();
    void create();
    void idle();
    void idleDemo();
    bool checkState();
    void motor();
    void contact();
    void setSpring(btGeneric6DofSpringConstraint*, int);
    btRigidBody* initTubefeet(btScalar*, const btTransform&);
    btRigidBody* createRigidBody(btScalar, const btTransform&, btCollisionShape*, int);
};

#endif /* Ophiuroid3_hpp */

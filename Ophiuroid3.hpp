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
    std::map<int, btRotationalLimitMotor* > motor_tY;
    std::map<int, btRotationalLimitMotor* > motor_tZ;
    std::map<int, btTranslationalLimitMotor* > motor_tran;
    std::map<int, btRotationalLimitMotor* > motor_to_groundY;
    std::map<int, btRotationalLimitMotor* > motor_to_groundZ;
public:
    Ophiuroid3(GAparameter);
    
    void initSF();
    void create();
    void idle();
    void idleDemo();
    bool checkState();
    void motor();
    void contact();
    btRigidBody* initTubefeet(btScalar*, const btVector3);
    btRigidBody* createRigidBody(btScalar, const btTransform&, btCollisionShape*, int);
};

#endif /* Ophiuroid3_hpp */

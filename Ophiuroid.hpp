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

public:
    Ophiuroid();
    
    void idle();
    bool checkState();
    void create();
    float evalue(GAparameter);
    float evalue2(GAparameter);
    btRigidBody* createRigidBody(btScalar, const btTransform&, btCollisionShape*);
    
};

#endif /* Ophiuroid_hpp */

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
 
public:
    Ophiuroid3(GAparameter);
    Ophiuroid3(Starfish*);

    void initSF();
    void create();
    void idle();
    void idleDemo();
    void idleNEAT();
    bool checkState();
    void checkLightDistance();
    void motor();
    void contact();
    void setDirection();
    void setDirection_NEAT();
    float evalue();
    float evalue_NEAT(NEAT::Network*);
    void setSpring(btGeneric6DofSpringConstraint*, int);
    btScalar calcMotorVel(int);
    btRigidBody* initTubefeet(btScalar*, const btTransform&);
    btRigidBody* createRigidBody(btScalar, const btTransform&, btCollisionShape*, int);
};

#endif /* Ophiuroid3_hpp */

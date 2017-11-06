//
//  main.cpp
//  Master
//
//  Created by 増田貴行 on 2017/10/31.
//  Copyright © 2017年 増田貴行. All rights reserved.
//

#include <stdio.h>

#if 0

#include "MasterStuff.hpp"
#include "Master.hpp"

int main (int argc, char** argv) {
    Starfish* oph2 = new Ophiuroid2();
    Master master = *new Master();
    
    master.setStarfish(oph2);
    mastermain(argc, argv, &master);
    
    return 0;
}

#else

#include "Source.h"

int main (int argc, char** argv) {
    Oophiuroid2 oph2;
    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
    
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
    
    btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();
    
    btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
    
    //world = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
    
    btDiscreteDynamicsWorld* world = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
    
    sourcemain(argc, argv, &oph2, world);
    
    return 0;
}
#endif

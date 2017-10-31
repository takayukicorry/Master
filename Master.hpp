//
//  Master.hpp
//  Master
//
//  Created by 増田貴行 on 2017/10/31.
//  Copyright © 2017年 増田貴行. All rights reserved.
//

#ifndef Master_hpp
#define Master_hpp

#include "Variables.h"
#include "Ophiuroid.hpp"
#include "Ophiuroid2.hpp"


class Master {
    
public:
    btDefaultCollisionConfiguration* collisionConfiguration;
    btCollisionDispatcher* dispatcher;
    btBroadphaseInterface* overlappingPairCache;
    btSequentialImpulseConstraintSolver* solver;
    btDiscreteDynamicsWorld* dynamicsWorld;
    btCollisionShape* groundShape;
    btAlignedObjectArray<btCollisionShape*> collisionShapes;
    int time_step;
    
    std::map<int, Starfish*> Starfishes;
    
public:
    Master();
    
    btDiscreteDynamicsWorld* getWorld() { return dynamicsWorld; }
    void Render();
    void CleanupBullet();
    void timer(int t) { time_step += t; }
    int getTime() { return time_step; }
    void setStarfish(int i, Starfish* s) { Starfishes[i] = s; }
    std::map<int, Starfish*> getStarfishes() { return Starfishes; }
    
};

#endif /* Master_hpp */

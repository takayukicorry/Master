//
//  Master.hpp
//  Master
//
//  Created by 増田貴行 on 2017/10/31.
//  Copyright © 2017年 増田貴行. All rights reserved.
//

#ifndef Master_hpp
#define Master_hpp

#include "Ophiuroid.hpp"
#include "Ophiuroid2.hpp"
#include "Ophiuroid3.hpp"


class Master {
    
private:
    static btDefaultCollisionConfiguration* collisionConfiguration;
    static btCollisionDispatcher* dispatcher;
    static btBroadphaseInterface* overlappingPairCache;
    static btSequentialImpulseConstraintSolver* solver;
    
    Starfish* starfish;
    GAparameter m_param;
    
public:
    static btDiscreteDynamicsWorld* dynamicsWorld;
    static btCollisionShape* groundShape;
    
    Master();
    void Render();
    void idle();
    void idleDemo();
    void InitBullet();
    void CleanupBullet();
    void init();
    void setStarfish(Starfish* s);
    Starfish* getStarfishes() { return starfish; }
    void createGround();
    void createStarfish();
    void CleanupStarfish();
    void checkStarfish();
    
};

#endif /* Master_hpp */

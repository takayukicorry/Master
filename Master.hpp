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


class Master {
    
private:
    btDiscreteDynamicsWorld* dynamicsWorld;
    Starfish* starfish;
    
public:
    static int time_step;
    static btCollisionShape* groundShape;
    static btAlignedObjectArray<btCollisionShape*> collisionShapes;
    
    Master(btDiscreteDynamicsWorld*);
    void Render();
    void idle();
    void InitBullet();
    void CleanupBullet();
    void init();
    void setStarfish(Starfish* s) { starfish = s; }
    Starfish* getStarfishes() { return starfish; }
    void createGround();
    void createStarfish();
    
};

#endif /* Master_hpp */

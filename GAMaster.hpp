//
//  GAMaster.hpp
//  Master
//
//  Created by 増田貴行 on 2017/12/29.
//  Copyright © 2017年 増田貴行. All rights reserved.
//

#ifndef GAMaster_hpp
#define GAMaster_hpp

#include "Variables.h"

class GAMaster {
public:
    static btDiscreteDynamicsWorld* createWorld();
    static void createGround(btDiscreteDynamicsWorld*);
    static void cleanupWorld(btDiscreteDynamicsWorld*);
};

#endif /* GAMaster_hpp */

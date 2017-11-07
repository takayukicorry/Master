//
//  Starfish.h
//  Master
//
//  Created by 増田貴行 on 2017/10/31.
//  Copyright © 2017年 増田貴行. All rights reserved.
//

#ifndef Starfish_h
#define Starfish_h

#include "Variables.h"

class Starfish {
    
public:
    btDiscreteDynamicsWorld* dynamicsWorld = 0;
    virtual void idle() = 0;
    virtual void create() = 0;
    virtual ~Starfish() {}
};

#endif /* Starfish_h */

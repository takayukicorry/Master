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
    std::map<int, btHingeConstraint*> m_joints_ankle;
    std::map<int, btUniversalConstraint*> m_joints_hip;
    std::map<int, btRigidBody*> m_bodies;
    std::map<int, btRotationalLimitMotor*> m_motor1;//handle
    std::map<int, btRotationalLimitMotor*> m_motor2;//wheel
    std::map<int, btCollisionShape*> m_shapes;

    virtual void idle() = 0;
    virtual bool checkState() = 0;
    virtual void create() = 0;
    virtual void initSF() = 0;
    virtual ~Starfish() {}
};

#endif /* Starfish_h */

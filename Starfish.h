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
#include "GA.h"

class Starfish {
public:
    std::map<int, btHingeConstraint*> m_joints_ankle;
    std::map<int, btUniversalConstraint*> m_joints_hip;
    std::map<int, btRigidBody*> m_bodies;
    std::map<int, btRotationalLimitMotor*> m_motor1;//handle
    std::map<int, btRotationalLimitMotor*> m_motor2;//wheel
    std::map<int, btCollisionShape*> m_shapes;
    btDynamicsWorld* m_ownerWorld;
    bool drawTF = true;
    int m_time_step;
    bool kCheck_first;
    int className;
    GAparameter m_param;

    virtual void idle() = 0;
    virtual bool checkState() = 0;
    virtual void create() = 0;
    virtual void initSF() = 0;
    virtual ~Starfish() {}
    
    void activateMotor(bool t) {
        for (auto itr = m_motor1.begin(); itr != m_motor1.end(); ++itr) {
            itr->second->m_enableMotor = t;
        }
        
        for (auto itr = m_motor2.begin(); itr != m_motor2.end(); ++itr) {
            itr->second->m_enableMotor = t;
        }
        
        for (auto itr = m_joints_ankle.begin(); itr != m_joints_ankle.end(); ++itr) {
            itr->second->enableMotor(t);
        }
    }
    
    void activateTwist(bool t) {
        for (auto itr = m_joints_hip.begin(); itr != m_joints_hip.end(); ++itr) {
            itr->second->setLowerLimit(-M_PI_2, 0);
            itr->second->setUpperLimit(M_PI_2, 0);
        }
    }
    
    void zeroFriction(bool t) {
        btScalar f = (t) ? 0 : 5;
        for (auto itr = m_bodies.begin(); itr != m_bodies.end(); ++itr) {
            itr->second->setFriction(f);
        }
    }
    void setWorld(btDynamicsWorld* ownerWorld) {m_ownerWorld = ownerWorld;}
};

#endif /* Starfish_h */

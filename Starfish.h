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
#include "network.hpp"

class Starfish {
public:
    std::map<int, btHingeConstraint*> m_joints_ankle;
    std::map<int, btUniversalConstraint*> m_joints_hip;
    std::map<int, btRigidBody*> m_bodies;
    std::map<int, btRotationalLimitMotor*> m_motor1;//handle
    std::map<int, btRotationalLimitMotor*> m_motor2;//wheel
    std::map<int, btCollisionShape*> m_shapes;
    
    std::map<int, btRotationalLimitMotor* > motor_tX;
    std::map<int, btRotationalLimitMotor* > motor_tY;
    std::map<int, btRotationalLimitMotor* > motor_tZ;
    std::map<int, btRotationalLimitMotor* > motor_to_groundY;
    std::map<int, btRotationalLimitMotor* > motor_to_groundZ;
    std::map<int, bool > motor_state;
    std::map<int, bool > TF_Contact;
    std::map<int, bool > TF_foward;
    std::map<int, btRigidBody* > TF_object;
    std::map<int, int > dl_time;
    std::map<int, int> InitTime_tf;
    std::map<int, int> ResumeTime_ground;
    std::map<int, int> Init_tf;
    std::map<int, btGeneric6DofSpringConstraint*> TF_constraint;
    std::map<int, btUniversalConstraint*> TF_constraint_ground;
    std::map<int, btVector3> TF_axis_direction;
    std::map<int, btVector3> TF_pos;
    
    std::vector<btRigidBody* > bodies_tf;
    std::vector<btTypedConstraint* > constraints;
    btDiscreteDynamicsWorld* m_ownerWorld;
    bool drawTF = true;
    int m_time_step;
    bool kCheck_first;
    int className;
    GAparameter m_param;
    NEAT::Network* m_net;
    bool hasNet;
    double m_value;

    virtual void idle() = 0;
    virtual void idleNEAT() = 0;
    virtual bool checkState() = 0;
    virtual void create() = 0;
    virtual void initSF() = 0;
    virtual float evalue() = 0;
    virtual float evalue_NEAT(NEAT::Network*) = 0;
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
    void setWorld(btDiscreteDynamicsWorld* ownerWorld) {m_ownerWorld = ownerWorld;}
    void setNet(NEAT::Network* net) {m_net = net; hasNet = true;}
    void setParam(GAparameter p) {m_param = p;}
};

#endif /* Starfish_h */

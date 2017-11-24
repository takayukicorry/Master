//
//  Ophiuroid2.hpp
//  Master
//
//  Created by 増田貴行 on 2017/10/31.
//  Copyright © 2017年 増田貴行. All rights reserved.
//

#ifndef Ophiuroid2_hpp
#define Ophiuroid2_hpp

#include <stdio.h>
#include "Starfish.h"
#include "Master.hpp"

class Ophiuroid2 : public Starfish {
    
private:
    std::map<int, bool> TF_contact;//tubefeet - ground (attach)
    std::map<int, int> TF_contact_times;//tubefeet - ground (attach times)
    std::map<int, btTypedConstraint*> TF_constraint_amp;//tubefeet - amp (constraint)
    std::map<int, btTypedConstraint*> TF_constraint_ground;//tubefeet - ground (constraint)
    std::map<int, btRigidBody*> TF_object;//tubefeet (object)
    std::map<int, btRigidBody*> TF_object_amp;//amp (object)
    std::map<int, btRotationalLimitMotor* > motor_tY;//tubefeet - amp (handle motor)
    std::map<int, btRotationalLimitMotor* > motor_tZ;//tubefeet - amp (wheel motor)
    std::map<int, btRotationalLimitMotor* > motor_to_groundY;//tubefeet - ground (handle motor)
    std::map<int, btRotationalLimitMotor* > motor_to_groundZ;//tubefeet - ground (wheel motor)
    std::map<int, btVector3> TF_axis_direction;//tubefeet - amp & tubefeet - ground (motor direction)
    std::map<int, btScalar> TF_axis_angle;//tubefeet - amp & tubefeet - ground (current motor angle to XZ)
    std::map<int, int> DeleteTime_tf;//time to delete tf - ground
    std::map<int, int> InitTime_tf;//first phase of tf
    std::map<int, int> ResumeTime_tf;//time to start swinging
    bool stay;

public:
    Ophiuroid2();
    
    void idle();
    bool checkState();
    void create();
    void initSF();
    btRigidBody* initAmp(btScalar, const btVector3);
    btRigidBody* initTubefeet(btScalar*, const btVector3);
    btRigidBody* createRigidBody(btScalar, const btTransform&, btCollisionShape*);
    void ContactAction();
    void ControllTubeFeet();
};

#endif /* Ophiuroid2_hpp */

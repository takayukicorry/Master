//
//  Source.h
//  Master
//
//  Created by 増田貴行 on 2017/10/31.
//  Copyright © 2017年 増田貴行. All rights reserved.
//

#ifndef Source_h
#define Source_h

//#include "btBulletDynamicsCommon.h"
#include <BulletDynamics/btBulletDynamicsCommon.h>
#include <stdio.h>
#include <stdlib.h>
//#include <GL/glut.h>
#include <GLUT/GLUT.h>
#include <OpenGL/OpenGL.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <map>
#include <random>
//#include <OpenGL/DemoApplication.h>
#include "Variables.h"

class Oophiuroid2 {
    
public:
    btDefaultCollisionConfiguration* collisionConfiguration;
    btCollisionDispatcher* dispatcher;
    btBroadphaseInterface* overlappingPairCache;
    btSequentialImpulseConstraintSolver* solver;
    btDiscreteDynamicsWorld* dynamicsWorld;
    btCollisionShape* groundShape;
    btAlignedObjectArray<btCollisionShape*> collisionShapes;
    
    //Userindex==int
    std::map<int, bool> TF_contact;//tubefeet - ground (attach)
    std::map<int, int> TF_contact_times;//tubefeet - ground (attach times)
    std::map<int, btTypedConstraint*> TF_constraint_amp;//tubefeet - amp (constraint)
    std::map<int, btTypedConstraint*> TF_constraint_ground;//tubefeet - ground (constraint)
    std::map<int, btRigidBody*> BODY_object;//arm (object)
    std::map<int, btRigidBody*> TF_object;//tubefeet (object)
    std::map<int, btRigidBody*> TF_object_amp;//amp (object)
    std::map<int, btRotationalLimitMotor* > motor_tY;//tubefeet - amp (handle motor)
    std::map<int, btRotationalLimitMotor* > motor_tZ;//tubefeet - amp (wheel motor)
    std::map<int, btRotationalLimitMotor* > motor_to_groundY;//tubefeet - ground (handle motor)
    std::map<int, btRotationalLimitMotor* > motor_to_groundZ;//tubefeet - ground (wheel motor)
    std::map<int, btVector3> TF_axis_direction;//tubefeet - amp & tubefeet - ground (motor direction)
    std::map<int, btScalar> TF_axis_angle;//tubefeet - amp & tubefeet - ground (current motor angle to XZ)
    std::map<int, int> DeleteTime_tf;//time to delete tf - ground
    std::map<int, int> ResumeTime_tf;//time to start swinging
    
    int time_step = 0;
    
    enum CollisionGroup{
        RX_COL_NOTHING = 0, // 0000
        RX_COL_GROUND = 1, // 0001
        RX_COL_BODY = 2,  // 0010
        RX_COL_TF = 4,  // 0100
        RX_COL_AMP = 8   // 1000
    };
    
    /*********************/
    //    function       //
    /*********************/
    btVector3 RotateY(const btVector3, double);
    btVector3 acrossb(btVector3, btVector3);
    void glutSolidCylinder(btScalar, btScalar, int, btVector3);
    btRigidBody::btRigidBodyConstructionInfo calcInertia(btScalar, btVector3);
    btRigidBody* getByUserIndex(int);
    void CreateGround();
    btRigidBody* initAmp(btScalar, const btVector3);
    btRigidBody* initBody(const btVector3, const btVector3);
    btRigidBody* initArm(const btVector3, const btVector3, const btQuaternion);
    btRigidBody* initTubefeet(btScalar*, const btVector3);
    void CreateStarfish();
    void ControllTubeFeet();
    void ContactAction();
    void init(void);
    void CleanupBullet();
    void InitBullet();

};

void sourcemain(int, char**, Oophiuroid2*);

#endif /* Source_h */

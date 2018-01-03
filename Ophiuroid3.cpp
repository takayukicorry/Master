//
//  Ophiuroid3.cpp
//  Master
//
//  Created by 増田貴行 on 2018/01/03.
//  Copyright © 2018年 増田貴行. All rights reserved.
//

#include "Ophiuroid3.hpp"

Ophiuroid3::Ophiuroid3(GAparameter p) {
    m_param = p;
    m_time_step = 0;
    className = 3;
}

void motorPreTickCallback3(btDynamicsWorld *world, btScalar timeStep) {
    Ophiuroid3* demo = (Ophiuroid3*)world->getWorldUserInfo();
    demo->idleDemo();
}

void Ophiuroid3::idle() {
    glutPostRedisplay();
}

void Ophiuroid3::idleDemo() {
    m_time_step++;
    
    motor();
    contact();
}

bool Ophiuroid3::checkState() {
    return true;
}

void Ophiuroid3::motor() {
    
}

void Ophiuroid3::contact() {
    
}

btScalar F_SIZE = FBODY_SIZE*2;
btScalar FLEG_SIZE = FBODY_SIZE/2.0;
btScalar FLEG_SIZE_WIDTH = FLEG_WIDTH/5.0;

void Ophiuroid3::initSF() {
    m_ownerWorld->setInternalTickCallback(motorPreTickCallback3, this, true);

    m_shapes[0] = new btCylinderShape(btVector3(F_SIZE,FLEG_SIZE_WIDTH,F_SIZE));
    int i, j, k;
    for (i = 0; i < NUM_LEGS; i++) {
        for (j = 1; j <= NUM_JOINT; j++) {
            m_shapes[NUM_JOINT*i+j] = new btCylinderShape(btVector3(FLEG_SIZE,FLEG_SIZE_WIDTH,FLEG_SIZE));
        }
    }
    //
    // Setup rigid bodies
    //
    btTransform transform;
    // root
    transform.setIdentity();
    transform.setOrigin(btVector3(btScalar(0.), btScalar(FHEIGHT), btScalar(0.)));
    m_bodies[0] = createRigidBody(btScalar(M_OBJ), transform, m_shapes[0], 10);
    // legs
    for (i = 0; i < NUM_LEGS; i++) {
        float fAngle = 2 * M_PI * i / NUM_LEGS;
        float fSin = sin(fAngle);
        float fCos = cos(fAngle);
        btVector3 point = btVector3(fCos*(F_SIZE+FLEG_SIZE),FHEIGHT,fSin*(F_SIZE+FLEG_SIZE));
        for (j = 1; j <= NUM_JOINT; j++) {
            transform.setIdentity();
            transform.setOrigin(point);
            transform.setRotation(btQuaternion(btVector3(0,1,0),-fAngle));
            //m_bodies[NUM_JOINT*i+j] = createRigidBody(btScalar(M_OBJ), transform, m_shapes[NUM_JOINT*i+j], 10+NUM_JOINT*i+j);
            
            point += btVector3(fCos*2*FLEG_SIZE,0,fSin*2*FLEG_SIZE);
        }
    }
    //
    // Setup some damping on the m_bodies
    //
    for (i = 0; i < m_bodies.size(); ++i) {
        m_bodies[i]->setDamping(0.05, 0.85);
        m_bodies[i]->setDeactivationTime(1000000000000.f);
        m_bodies[i]->setSleepingThresholds(1.6, 2.5);
        m_bodies[i]->setSleepingThresholds(0.5f, 0.5f);
        m_bodies[i]->forceActivationState(DISABLE_DEACTIVATION);
        
    }/*
    //
    // Setup the constraints
    //
    btVector3 axisA(0, 0, 1);
    btVector3 axisB(0, 0, 1);
    btVector3 pivotA(FLEG_SIZE,0,0);
    btVector3 pivotB(-FLEG_SIZE,0,0);
    for ( i=0; i<NUM_LEGS; i++) {
        float fAngle = 2 * M_PI * i / NUM_LEGS;
        float fSin = sin(fAngle);
        float fCos = cos(fAngle);
        // hip joints
        btVector3 parentAxis( fCos, 0, fSin);
        btVector3 childAxis( fSin, 0, -fCos);
        btVector3 anchor( fCos*F_SIZE, FHEIGHT, fSin*F_SIZE);
        btUniversalConstraint* univ = new btUniversalConstraint(*m_bodies[0],*m_bodies[NUM_JOINT*i+1],anchor,parentAxis,childAxis);
        univ->setLowerLimit(-M_PI_2, -M_PI_2);
        univ->setUpperLimit(M_PI_2, M_PI_2);
        univ->setLinearLowerLimit(btVector3(0,0,0));
        univ->setLinearUpperLimit(btVector3(0,0,0));
        m_joints_hip[i] = univ;
        m_ownerWorld->addConstraint(m_joints_hip[i], true);

        const int AXIS1_ID = 2;
        const int AXIS2_ID = 1;
        m_motor1[i] = univ->getRotationalLimitMotor(AXIS1_ID);
        m_motor2[i] = univ->getRotationalLimitMotor(AXIS2_ID);
        m_motor1[i]->m_maxMotorForce = MAX_MOTOR_TORQUE;
        m_motor2[i]->m_maxMotorForce = MAX_MOTOR_TORQUE;
        m_motor1[i]->m_enableMotor = true;
        m_motor2[i]->m_enableMotor = true;
        
        //ankle joints
        for (int j = 1; j < NUM_JOINT; j++) {
            btHingeConstraint* hin = new btHingeConstraint(*m_bodies[NUM_JOINT*i+j],*m_bodies[NUM_JOINT*i+j+1],pivotA,pivotB,axisA,axisB);
            hin->setLimit(-M_PI_2, M_PI_2);
            hin->setUserConstraintId(10);
            m_joints_ankle[(NUM_JOINT-1)*i+j-1] = hin;
            m_ownerWorld->addConstraint(m_joints_ankle[(NUM_JOINT-1)*i+j-1], true);
        }
    }*/
}

void Ophiuroid3::create() {
    
    std::vector<btRigidBody* > bodies_tf;
    std::vector<btTypedConstraint* > constraints;

    btScalar scale[] = {btScalar(RADIUS_TF), btScalar(LENGTH)};
    btVector3 pos_tf;
    btTransform tA, tB;
    
    int col, row;
    for (int i = 0; i < 4; i++) {
        col = i/2;
        row = i%2;
        int index = 100+i;
        //body
        pos_tf = btVector3(pow(-1, col)*F_SIZE/2.0,FHEIGHT-FLEG_SIZE_WIDTH/2.0-RADIUS_TF-LENGTH/2.0,pow(-1, row)*F_SIZE/2.0);
        btRigidBody* body_tf = initTubefeet(scale, pos_tf);
        body_tf->setUserIndex(100+i);
        bodies_tf.push_back(body_tf);
        //constraint
        tA.setIdentity(); tB.setIdentity();
        tA.setOrigin(btVector3(pow(-1, col)*F_SIZE/2.0,-FLEG_SIZE_WIDTH/2.0,pow(-1, row)*F_SIZE/2.0));
        tB.setOrigin(btVector3(0,RADIUS_TF+LENGTH/2.0,0));
        btGeneric6DofSpringConstraint* spring = new btGeneric6DofSpringConstraint(*m_bodies[0], *body_tf,tA,tB,true);
        spring->enableSpring(0, true);
        spring->enableSpring(1, true);
        spring->enableSpring(2, true);
        spring->setDamping(0, 1.f);
        spring->setDamping(1, 1.f);
        spring->setDamping(2, 1.f);
        spring->setStiffness(0, 1.f);
        spring->setStiffness(1, 1.f);
        spring->setStiffness(2, 1.f);
        spring->setLinearLowerLimit(btVector3(-2,-2,-2));
        spring->setLinearUpperLimit(btVector3(2,2,2));
        spring->setAngularLowerLimit(btVector3(-M_PI_2,-M_PI,-M_PI_2));
        spring->setAngularUpperLimit(btVector3(M_PI_2,M_PI,M_PI_2));
        constraints.push_back(spring);
        //motor
        btRotationalLimitMotor* motorRotY = spring->getRotationalLimitMotor(1);
        btRotationalLimitMotor* motorRotZ = spring->getRotationalLimitMotor(2);
        motorRotY->m_enableMotor = true;
        motorRotY->m_targetVelocity = M_PI_4;
        motorRotY->m_maxMotorForce = 1;
        motorRotZ->m_enableMotor = true;
        motorRotZ->m_targetVelocity = M_PI_4;
        motorRotZ->m_maxMotorForce = 1;
        motor_tY[index] = motorRotY;
        motor_tZ[index] = motorRotZ;
    }
    
    for (int i = 0; i < bodies_tf.size(); i++) {
        m_ownerWorld->addRigidBody(bodies_tf[i], RX_COL_TF, RX_COL_GROUND);
    }
    
    for (int i = 0; i < constraints.size(); i++) {
        m_ownerWorld->addConstraint(constraints[i]);
    }
}

btRigidBody* Ophiuroid3::initTubefeet(btScalar* scale, const btVector3 position)
{
    btCollisionShape* sBodyShape = new btCapsuleShape(scale[0], scale[1]);
    
    btScalar mass1(M_TF);
    bool isDynamic = (mass1 != 0.f);
    
    btVector3 localInertia1(0, 0, 0);
    if (isDynamic)
        sBodyShape->calculateLocalInertia(mass1, localInertia1);
    
    btTransform sBodyTransform;
    sBodyTransform.setIdentity();
    sBodyTransform.setOrigin(position);
    btDefaultMotionState* myMotionState1 = new btDefaultMotionState(sBodyTransform);
    
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass1, myMotionState1, sBodyShape, localInertia1);
    btRigidBody* body = new btRigidBody(rbInfo);
    
    return body;
}

btRigidBody* Ophiuroid3::createRigidBody(btScalar mass, const btTransform &startTransform, btCollisionShape *shape, int index) {
    bool isDynamic = (mass != 0.f);
    
    btVector3 localInertia(0,0,0);
    if (isDynamic)
        shape->calculateLocalInertia(mass,localInertia);
    
    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);
    body->setUserIndex(index);
    
    m_ownerWorld->addRigidBody(body, RX_COL_BODY, RX_COL_GROUND | RX_COL_BODY);
    
    return body;
}

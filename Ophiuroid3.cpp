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
    btTransform tra_body = m_bodies[0]->getWorldTransform();
    btVector3 pos_body = tra_body.getOrigin();
    tra_body.setIdentity();
    if (pos_body[1] < FHEIGHT) {
        //pos_body[1] = FHEIGHT;
    }
    tra_body.setOrigin(pos_body);
    m_bodies[0]->setCenterOfMassTransform(tra_body);
    
    
    for (auto itr = motor_tZ.begin(); itr != motor_tZ.end(); ++itr) {
        int index = itr->first;
        if (motor_state[index] && m_time_step > InitTime_tf[index]) {
            btRotationalLimitMotor* motor = itr->second;
            //
            //handle
            //
            btScalar angleY = motor_tY[index]->m_currentPosition;
            btScalar angle_target;
            if (TF_axis_direction[index][0] == 0) {
                if (TF_axis_direction[index][2] < 0) {
                    angle_target = -M_PI/2;
                } else if (TF_axis_direction[index][2] > 0) {
                    angle_target = M_PI/2;
                } else {
                    angle_target = 0;
                }
            } else {
                angle_target = atan(TF_axis_direction[index][2]/TF_axis_direction[index][0]);
            }
            motor_tY[index]->m_targetVelocity = (angle_target - angleY)/10;
            //
            //wheel
            //
            btScalar angleZ = motor->m_currentPosition;
            if (!Init_tf[index]) {
                Init_tf[index] = true;
                motor->m_targetVelocity = ANGLE_VELOCITY_TF;
            }
            
            if (angleZ > M_PI_4-0.1) {
                motor->m_targetVelocity = -ANGLE_VELOCITY_TF;
            } else if (angleZ < 0) {
                TF_object[index]->setFriction(0);
                if (angleZ < ANGLE_DETACH+0.1) {
                    motor->m_targetVelocity = ANGLE_VELOCITY_TF;
                    motor->m_maxMotorForce = 100000000;
                }
            }
        }
    }
    
    for (auto itr = motor_to_groundZ.begin(); itr != motor_to_groundZ.end(); ++itr) {
        btRotationalLimitMotor* motor = itr->second;
        motor->m_targetVelocity = -ANGLE_VELOCITY_GROUND;
    }
}

void Ophiuroid3::contact() {
    std::vector<int > contacts;
    
    int numManifolds = m_ownerWorld->getDispatcher()->getNumManifolds();
    for (int i = 0; i < numManifolds; i++)
    {
        btPersistentManifold* contactManifold =  m_ownerWorld->getDispatcher()->getManifoldByIndexInternal(i);
        const btCollisionObject* obA = contactManifold->getBody0();
        const btCollisionObject* obB = contactManifold->getBody1();
        btRigidBody* bodyA = btRigidBody::upcast(const_cast<btCollisionObject *>(obA));
        btRigidBody* bodyB = btRigidBody::upcast(const_cast<btCollisionObject *>(obB));
        btRigidBody* bodyC;
        btRigidBody* bodyG;

        int obIDA = obA->getUserIndex();
        int obIDB = obB->getUserIndex();

        //when a tubefeet attach ground
        if ((10 < obIDA && obIDA < 100) || (10 < obIDB && obIDB < 100)) {
            break;
        }
        
        if (0 < obIDA && obIDA <= NUM_GROUND*NUM_GROUND) {
            bodyC = bodyB;
            bodyG = bodyA;
        } else if (0 < obIDB && obIDB <= NUM_GROUND*NUM_GROUND) {
            bodyC = bodyA;
            bodyG = bodyB;
        } else {
            break;
        }
        int numContacts = contactManifold->getNumContacts();
        if (numContacts >= 1) {
            int k = -1;
            for (int j = 0; j < numContacts; j++) {
                btManifoldPoint& pt = contactManifold->getContactPoint(j);
                if (pt.getDistance() < 0.5f){k = j;break;}
            }
            if (k == -1) {break;}
        
            btManifoldPoint& pt = contactManifold->getContactPoint(k);
            const btVector3& ptB = pt.getPositionWorldOnB();
            int obIDC = bodyC->getUserIndex();
        
            if (!TF_Contact[obIDC]) {
                btScalar angle = motor_tZ[obIDC]->m_currentPosition;
                if (angle > ANGLE_ATTACH) {
                    TF_Contact[obIDC] = true;
                    dl_time[obIDC] = m_time_step + DL_TIME;
                    //change motor state
                    motor_state[obIDC] = false;
                    motor_tY[obIDC]->m_enableMotor = false;
                    motor_tZ[obIDC]->m_enableMotor = false;
                    //create tf - ground constraint
                    btUniversalConstraint* univ = new btUniversalConstraint(*bodyG, *bodyC, ptB+btVector3(0,RADIUS_TF,0), btVector3(0,1,0), btVector3(0,0,1));
                    univ->setLowerLimit(-M_PI, 0);
                    univ->setUpperLimit(M_PI, 0);
                    TF_constraint_ground[obIDC] = univ;
                    m_ownerWorld->addConstraint(univ);
                    //create tf - ground motor
                    btRotationalLimitMotor* motor1 = univ->getRotationalLimitMotor(1);//wheel
                    btRotationalLimitMotor* motor2 = univ->getRotationalLimitMotor(2);//handle
                    motor1->m_enableMotor = true;
                    motor1->m_targetVelocity = 0;
                    motor1->m_maxMotorForce = 10;
                    motor2->m_enableMotor = true;
                    motor2->m_targetVelocity = 0;
                    motor2->m_maxMotorForce = 10;
                    motor_to_groundZ[obIDC] = motor1;
                    motor_to_groundY[obIDC] = motor2;
                }
            } else {
                btScalar angle = motor_to_groundZ[obIDC]->m_currentPosition;
                if(angle < ANGLE_DETACH - ANGLE_ATTACH || dl_time[obIDC] < m_time_step) {
                    //remove tubefeet - ground (constraint & motor)
                    m_ownerWorld->removeConstraint(TF_constraint_ground[obIDC]);
                    motor_to_groundY.erase(obIDC);
                    motor_to_groundZ.erase(obIDC);
                    TF_Contact[obIDC] = false;
                    motor_state[obIDC] = true;
                    //activate tf motor
                    motor_tZ[obIDC]->m_enableMotor = true;
                    motor_tZ[obIDC]->m_targetVelocity = ANGLE_VELOCITY_TF;
                }
            }
        }
    }
}

btScalar F_SIZE = FBODY_SIZE*4;
btScalar FLEG_SIZE = F_SIZE/2.0;
btScalar FLEG_SIZE_WIDTH = FLEG_WIDTH/5.0;

void Ophiuroid3::initSF() {
    m_ownerWorld->setInternalTickCallback(motorPreTickCallback3, this, true);

    m_shapes[0] = new btCylinderShape(btVector3(F_SIZE,FLEG_SIZE_WIDTH,F_SIZE));
    int i, j;
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
            m_bodies[NUM_JOINT*i+j] = createRigidBody(btScalar(M_OBJ), transform, m_shapes[NUM_JOINT*i+j], 10+NUM_JOINT*i+j);
            
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
        
    }
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
    }
}

void Ophiuroid3::create() {
    
    std::random_device rnd;
    std::mt19937 mt(rnd());
    std::uniform_int_distribution<> rand100(0, 99);
    
    btScalar scale[] = {btScalar(RADIUS_TF), btScalar(LENGTH)};
    btVector3 pos_tf, pos_body;
    btTransform tA, tB;
    pos_body = btVector3(0,FHEIGHT,0);
    
    int col, row;
    for (int i = 0; i < 9; i++) {
        col = i/3;
        row = i%3;
        int index = 100+i;
        //body
        pos_tf = btVector3((col-1)*F_SIZE/2.0,FHEIGHT-FLEG_SIZE_WIDTH/2.0-RADIUS_TF-LENGTH/2.0,(row-1)*F_SIZE/2.0);
        btRigidBody* body_tf = initTubefeet(scale, pos_tf);
        body_tf->setUserIndex(100+i);
        bodies_tf.push_back(body_tf);
        TF_Contact[index] = false;
        TF_axis_direction[index] = btVector3(0, 0, 1);
        InitTime_tf[index] = 2*SECOND*( rand100(mt)/100.0 );
        TF_object[index] = body_tf;
        Init_tf[index] = false;
        //constraint
        tA.setIdentity(); tB.setIdentity();
        tA.setOrigin(pos_tf-pos_body+btVector3(0,RADIUS_TF+LENGTH/2.0,0));
        tB.setOrigin(btVector3(0,RADIUS_TF+LENGTH/2.0,0));
        btGeneric6DofSpringConstraint* spring = new btGeneric6DofSpringConstraint(*m_bodies[0], *body_tf,tA,tB,true);
        setSpring(spring, index);
    }
    
    for (int i = 0; i < bodies_tf.size(); i++) {
        m_ownerWorld->addRigidBody(bodies_tf[i], RX_COL_TF, RX_COL_GROUND);
    }
    
    for (int i = 0; i < constraints.size(); i++) {
        m_ownerWorld->addConstraint(constraints[i]);
    }
}

void Ophiuroid3::setSpring(btGeneric6DofSpringConstraint* spring, int index) {
    spring->enableSpring(0, true);
    spring->enableSpring(1, true);
    spring->setDamping(0, 10.f);
    spring->setDamping(1, 10.f);
    spring->setStiffness(0, 10.f);
    spring->setStiffness(1, 10.f);
    spring->setLinearLowerLimit(btVector3(-2,-2,0));
    spring->setLinearUpperLimit(btVector3(2,2,0));
    spring->setAngularLowerLimit(btVector3(0,0,-M_PI/3));
    spring->setAngularUpperLimit(btVector3(0,0,M_PI/3));
    TF_constraint[index] = spring;
    constraints.push_back(spring);
    //motor
    btRotationalLimitMotor* motorRotY = spring->getRotationalLimitMotor(1);
    btRotationalLimitMotor* motorRotZ = spring->getRotationalLimitMotor(2);
    motorRotY->m_enableMotor = true;
    motorRotY->m_targetVelocity = 0;
    motorRotY->m_maxMotorForce = 100000000;
    motorRotZ->m_enableMotor = true;
    motorRotZ->m_targetVelocity = 0;
    motorRotZ->m_maxMotorForce = 100000000;
    motor_tY[index] = motorRotY;
    motor_tZ[index] = motorRotZ;
    motor_state[index] = true;
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

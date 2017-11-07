//
//  Ophiuroid.cpp
//  Master
//
//  Created by 増田貴行 on 2017/10/31.
//  Copyright © 2017年 増田貴行. All rights reserved.
//

#include "Ophiuroid.hpp"

Ophiuroid::Ophiuroid() {
    
}

float Ophiuroid::evalue(GAparameter p) {
    return 0;
}

float Ophiuroid::evalue2(GAparameter p) {
    return 0;
}

void Ophiuroid::idle() {
    glutPostRedisplay();
}

bool Ophiuroid::checkState() {
    return true;
}

void Ophiuroid::create() {
    //
    // Setup geometry
    //
    float fBodySize = 5;
    float fLegLength = 8/NUM_JOINT;
    
    float fHeight = 15;//落ちる高さ
    float alpha = 37*M_PI/36;//thighとshinが何度開いてるか初期値
    float theta = M_PI - alpha;
    float fLegWidth = 3;
    GAmanager manager;;
    
    m_shapes[0] = new btCylinderShape(btVector3(fBodySize,fLegWidth,fBodySize));
    
    int i;
    
    for ( i=0; i<NUM_LEGS; i++)
    {
        m_shapes[1 + (NUM_JOINT+1)*i] = new btSphereShape(fLegWidth);
        for (int k = 1; k <= NUM_JOINT; k++)
            
        {
            m_shapes[k + 1 + (NUM_JOINT+1)*i] = new btCapsuleShape(btScalar(fLegWidth), btScalar(fLegLength));
        }
    }

    //
    // Setup rigid bodies
    //
    btTransform offset;
    offset.setIdentity();
    offset.setOrigin(btVector3(0,10,0));
    
    // root
    btVector3 vRoot = btVector3(btScalar(0.), btScalar(fHeight), btScalar(0.));
    btTransform transform;
    btTransform transformS;
    btTransform transformY;
    transform.setIdentity();
    transform.setOrigin(vRoot);
    m_bodies[0] = createRigidBody(btScalar(0), offset*transform, m_shapes[0]);
    
    // legs
    for ( i=0; i<NUM_LEGS; i++)
    {
        float fAngle = 2 * M_PI * i / NUM_LEGS;
        float fSin = sin(fAngle);
        float fCos = cos(fAngle);
        
        
        btVector3 vBoneOrigin = btVector3(btScalar(fCos*(fBodySize + fLegWidth + (0.5*fLegLength+2*fLegWidth)*cos(theta))), btScalar(fHeight + (0.5*fLegLength+2*fLegWidth)*sin(theta)), btScalar(fSin*(fBodySize + fLegWidth + (0.5*fLegLength+2*fLegWidth)*cos(theta))));
        btVector3 vToBone = (vBoneOrigin - vRoot).normalize();
        btVector3 vAxis = vToBone.cross(btVector3(0,1,0));
        btVector3 Point = vBoneOrigin;
        btVector3 spherePoint = btVector3(btScalar(fCos*(fBodySize + fLegWidth )), btScalar(fHeight), btScalar(fSin*(fBodySize + fLegWidth)));
        
        
        transformY.setIdentity();
        transformY.setOrigin(btVector3(0,0,0));
        transformY.setRotation(btQuaternion(btVector3(0,1,0), -fAngle));
        
        transformS.setIdentity();
        transformS.setOrigin(spherePoint);
        
        m_bodies[1+(NUM_JOINT+1)*i] = createRigidBody(btScalar(0.5), offset*transformS, m_shapes[1+(NUM_JOINT+1)*i]);
        
        for (int k = 1; k <= NUM_JOINT; k++)
        {            
            transform.setIdentity();
            transform.setOrigin(Point);
            transform.setRotation(btQuaternion(vAxis, M_PI_2 - theta * k));//垂直から下側が何度上がって行くか
            
            m_bodies[k+1+(NUM_JOINT+1)*i] = createRigidBody(btScalar(0.5), offset*transform/*transformY*/, m_shapes[k+1+(NUM_JOINT+1)*i]);//軸だけ回転してて、剛体の初期ポジが横向き疑惑→解決
            Point += btVector3(btScalar(fCos*(0.5*fLegLength+fLegWidth)*cos(k*theta)),btScalar(-(0.5*fLegLength+fLegWidth)*sin(k*theta)),btScalar(fSin*(0.5*fLegLength+fLegWidth)*cos(k*theta))) + btVector3(btScalar(fCos*(0.5*fLegLength+fLegWidth)*cos((k+1)*theta)),btScalar(-(0.5*fLegLength+fLegWidth)*sin((k+1)*theta)),btScalar(fSin*(0.5*fLegLength+fLegWidth)*cos((k+1)*theta)));
        }
        m_bodies[(NUM_JOINT+1)*(i+1)]->setFriction(5.0);//摩擦
    }
    
    //停止が続いてもsleeping状態にならないようにする//
    // Setup some damping on the m_bodies
    
    for (i = 0; i < m_bodies.size(); ++i)
    {
        m_bodies[i]->setDamping(0.05, 0.85);
        m_bodies[i]->setDeactivationTime(1000000000000.f);
        m_bodies[i]->setSleepingThresholds(1.6, 2.5);
        //m_bodies[i]->setSleepingThresholds(0.5f, 0.5f);
        m_bodies[i]->forceActivationState(DISABLE_DEACTIVATION);
        
    }
    /**/
    
    //
    // Setup the constraints
    //
    btHingeConstraint* joint2;//第1リンクと第2リンクの拘束
    btUniversalConstraint* hinge2C;
    
    btTransform localA, localB, localC;
    
    for ( i=0; i<NUM_LEGS; i++)
    {
        float fAngle = 2 * M_PI * i / NUM_LEGS;
        float fSin = sin(fAngle);
        float fCos = cos(fAngle);
        
        /*hinge2で作ってみよう*//*未完成*/
        
        // hip joints
        btVector3 parentAxis( fCos, 0, fSin);//第一ジョイント回転軸１（ワールド座標）-->ローカルz軸
        btVector3 childAxis( fSin, 0, -fCos);//第一ジョイント回転軸２（ワールド座標）-->ローカルy軸
        btVector3 anchor( fCos*fBodySize, 10+fHeight, fSin*fBodySize);//上の２軸の交点（ワールド座標）
        
        hinge2C = new btUniversalConstraint(*m_bodies[0], *m_bodies[1+(NUM_JOINT+1)*i],anchor, parentAxis, childAxis);
        hinge2C->setLinearLowerLimit(btVector3(0,0,0));
        hinge2C->setLinearUpperLimit(btVector3(0,0,0));
        hinge2C->setAngularLowerLimit(btVector3(0,-M_PI_2,-M_PI_2));
        hinge2C->setAngularUpperLimit(btVector3(0,M_PI_2,M_PI_2));
        m_joints[(NUM_JOINT+1)*i] = hinge2C;
        Master::dynamicsWorld->addConstraint(m_joints[(NUM_JOINT+1)*i], true);
        
        
        const int AXIS1_ID = 2;
        const int AXIS2_ID = 1;
        const float MAX_MOTOR_TORQUE = 10000000000;
        m_motor1[i] = hinge2C->getRotationalLimitMotor(AXIS1_ID);
        m_motor2[i] = hinge2C->getRotationalLimitMotor(AXIS2_ID);
        m_motor1[i]->m_maxMotorForce = MAX_MOTOR_TORQUE;
        m_motor2[i]->m_maxMotorForce = MAX_MOTOR_TORQUE;
        
        m_motor1[i]->m_enableMotor = true;
        m_motor2[i]->m_enableMotor = true;
        
        btVector3 JointPoint = btVector3(btScalar(fCos*(fBodySize+2*fLegWidth)), btScalar(0.), btScalar(fSin*(fBodySize+2*fLegWidth)));
        
        
        for (int k = 1; k <= NUM_JOINT; k++)
        {
            localA.setIdentity(); localB.setIdentity(); localC.setIdentity();
            localA.getBasis().setEulerZYX(0,-fAngle,0);
            localA.setOrigin(JointPoint);
            localB = m_bodies[k+(NUM_JOINT+1)*i]->getWorldTransform().inverse() * m_bodies[0]->getWorldTransform() * localA;
            localC = m_bodies[k+1+(NUM_JOINT+1)*i]->getWorldTransform().inverse() * m_bodies[0]->getWorldTransform() * localA;
            joint2 = new btHingeConstraint(*m_bodies[k+(NUM_JOINT+1)*i], *m_bodies[k+1+(NUM_JOINT+1)*i], localB, localC);
            joint2->setLimit(manager.pool[0].lowerlimit[(NUM_JOINT+2)*i + 1 + k], manager.pool[0].upperlimit[(NUM_JOINT+2)*i + 1 + k]);
            /*************  なんでこれ効かないのや？？  ****************/
            joint2->setMotorTarget(M_PI, M_PI/10);
            m_joints[k+(NUM_JOINT+1)*i] = joint2;
            Master::dynamicsWorld->addConstraint(m_joints[k+(NUM_JOINT+1)*i], true);
            JointPoint += btVector3(btScalar(fCos*(fLegLength+2*fLegWidth)*cos(k*theta)),btScalar(-(fLegLength+2*fLegWidth)*sin(k*theta)),btScalar(fSin*(fLegLength+2*fLegWidth)*cos(k*theta)));
            
        }
    }
}

btRigidBody* Ophiuroid::createRigidBody(btScalar mass, const btTransform &startTransform, btCollisionShape *shape) {
    bool isDynamic = (mass != 0.f);
    
    btVector3 localInertia(0,0,0);
    if (isDynamic)
        shape->calculateLocalInertia(mass,localInertia);
    
    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);
    
    Master::dynamicsWorld->addRigidBody(body, RX_COL_BODY, RX_COL_GROUND);
    
    return body;
}

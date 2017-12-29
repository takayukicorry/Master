//
//  GAMaster.cpp
//  Master
//
//  Created by 増田貴行 on 2017/12/29.
//  Copyright © 2017年 増田貴行. All rights reserved.
//

#include "GAMaster.hpp"

btDiscreteDynamicsWorld* GAMaster::createWorld() {
    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
    btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();
    btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
    btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
    
    return dynamicsWorld;
}

void GAMaster::createGround(btDiscreteDynamicsWorld* ownerWorld) {
    btVector3 gShape(btScalar(100.), btScalar(50.), btScalar(100.));
    btCollisionShape* groundShape = new btBoxShape(gShape);
    
    btTransform groundTransform;
    btScalar mass(0.);
    bool isDynamic = (mass != 0.f);
    btVector3 localInertia(0, 0, 0);
    if (isDynamic)
        groundShape->calculateLocalInertia(mass, localInertia);
    
    for (int i = 0; i < NUM_GROUND; i++) {
        for (int j = 0; j < NUM_GROUND; j++) {
            groundTransform.setIdentity();
            groundTransform.setOrigin(btVector3((NUM_GROUND-1-i*2)*gShape[0], -gShape[1], (NUM_GROUND-1-j*2)*gShape[2]));
            
            btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
            btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
            btRigidBody* body = new btRigidBody(rbInfo);
            body->setActivationState(DISABLE_DEACTIVATION);
            body->setUserIndex(1);
            
            ownerWorld->addRigidBody(body, RX_COL_GROUND, RX_COL_BODY | RX_COL_TF | RX_COL_AMP);
        }
    }
    groundTransform.setIdentity();
    groundTransform.setOrigin(btVector3((NUM_GROUND-1-(NUM_GROUND-1)*2)*gShape[0]+30, gShape[1], (NUM_GROUND-1-(NUM_GROUND-2)*2)*gShape[2]));
    
    btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);
    body->setActivationState(DISABLE_DEACTIVATION);
    body->setUserIndex(2);
    
    ownerWorld->addRigidBody(body, RX_COL_GROUND, RX_COL_BODY | RX_COL_TF | RX_COL_AMP);
}

void GAMaster::cleanupWorld(btDiscreteDynamicsWorld* dynamicsWorld) {
        for (int i = dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
        {
            btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
            btRigidBody* body = btRigidBody::upcast(obj);
            if (body && body->getMotionState())
            {
                delete body->getMotionState();
            }
            dynamicsWorld->removeCollisionObject(obj);
            delete obj;
        }
        
        for(int i = dynamicsWorld->getNumConstraints()-1; i>=0 ;i--){
            btTypedConstraint* constraint = dynamicsWorld->getConstraint(i);
            dynamicsWorld->removeConstraint(constraint);
            delete constraint;
        }
    
        delete dynamicsWorld;
}

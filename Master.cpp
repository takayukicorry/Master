//
//  Master.cpp
//  Master
//
//  Created by 増田貴行 on 2017/10/31.
//  Copyright © 2017年 増田貴行. All rights reserved.
//

#include "Master.hpp"

/*色など*/
GLfloat light0pos[] = { 300.0, 300.0, 300.0, 1.0 };
GLfloat light1pos[] = { -300.0, 300.0, 300.0, 1.0 };
struct MaterialStruct {
    GLfloat ambient[4];
    GLfloat diffuse[4];
    GLfloat specular[4];
    GLfloat shininess;
};
MaterialStruct ms_jade = {
    { 0.135, 0.2225, 0.1575, 1.0 },
    { 0.54, 0.89, 0.63, 1.0 },
    { 0.316228, 0.316228, 0.316228, 1.0 },
    12.8 };
MaterialStruct ms_ruby = {
    { 0.1745, 0.01175, 0.01175, 1.0 },
    { 0.61424, 0.04136, 0.04136, 1.0 },
    { 0.727811, 0.626959, 0.626959, 1.0 },
    76.8 };
GLfloat red[] = { 0.8, 0.2, 0.2, 1.0 };
GLfloat green[] = { 0.2, 0.8, 0.2, 1.0 };
GLfloat blue[] = { 0.2, 0.2, 0.8, 1.0 };
GLfloat yellow[] = { 0.8, 0.8, 0.2, 1.0 };
GLfloat white[] = { 1.0, 1.0, 1.0, 1.0 };
GLfloat shininess = 30.0;
enum CollisionGroup{
    RX_COL_NOTHING = 0, // 0000
    RX_COL_GROUND = 1, // 0001
    RX_COL_BODY = 2,  // 0010
    RX_COL_TF = 4,  // 0100
    RX_COL_AMP = 8   // 1000
};

void glutSolidCylinder(btScalar radius, btScalar height, int num, btVector3 position)
{
    glBegin(GL_POLYGON);
    
    for (int i = 0; i < num; i++) {
        glVertex3d(position[0] + radius * cos((M_PI*2/num)*i), position[1] + height, position[2] + radius * sin((M_PI*2/num)*i));
        glVertex3d(position[0] + radius * cos((M_PI*2/num)*(i+1)), position[1] + height, position[2] + radius * sin((M_PI*2/num)*(i+1)));
        glVertex3d(position[0] + radius * cos((M_PI*2/num)*(i+1)), position[1] - height, position[2] + radius * sin((M_PI*2/num)*(i+1)));
        glVertex3d(position[0] + radius * cos((M_PI*2/num)*i), position[1] - height, position[2] + radius * sin((M_PI*2/num)*i));
        
    }
    
    glEnd();
}
/*****************/
btCollisionShape* Master::groundShape = new btBoxShape(btVector3(btScalar(100.), btScalar(10.), btScalar(100.)));
btAlignedObjectArray<btCollisionShape*> Master::collisionShapes = *new btAlignedObjectArray<btCollisionShape*>();
int Master::time_step = 0;

Master::Master(btDiscreteDynamicsWorld* world) {
    dynamicsWorld = world;
    dynamicsWorld->setGravity(btVector3(0, -10, 0));
}

void Master::Render() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    //glLightfv(GL_LIGHT0, GL_POSITION, light0pos);
    
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    //glDisable(GL_LIGHT1);
    
    glPushMatrix();
    
    //draw each object
    for (int j = dynamicsWorld->getNumCollisionObjects() - 1; j >= 0; j--)
    {
        btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[j];
        btRigidBody* body = btRigidBody::upcast(obj);
        if (body && body->getMotionState())
        {
            btVector3 pos = body->getCenterOfMassPosition();
            int shape = body->getCollisionShape()->getShapeType();
            btScalar rot = body->getOrientation().getAngle() * RADIAN;
            btVector3 axis = body->getOrientation().getAxis();
            btVector3 halfExtent = static_cast<const btBoxShape*>(body->getCollisionShape())->getHalfExtentsWithMargin();
            
            glPushMatrix();
            glTranslatef(pos[0], pos[1], pos[2]);
            glRotated(rot, axis[0], axis[1], axis[2]);
            //ground
            if (j == 0 || j == 1 || j == 2 || j == 3)
            {
                glScaled(2 * halfExtent[0], 2 * halfExtent[1], 2 * halfExtent[2]);
                glMaterialfv(GL_FRONT, GL_AMBIENT, ms_jade.ambient);
                glMaterialfv(GL_FRONT, GL_DIFFUSE, ms_jade.diffuse);
                glMaterialfv(GL_FRONT, GL_SPECULAR, ms_jade.specular);
                glMaterialfv(GL_FRONT, GL_SHININESS, &ms_jade.shininess);
                glutSolidCube(1);
            }
            //box
            else if (shape == BOX_SHAPE_PROXYTYPE)
            {
                glScaled(2 * halfExtent[0], 2 * halfExtent[1], 2 * halfExtent[2]);
                glMaterialfv(GL_FRONT, GL_AMBIENT, ms_ruby.ambient);
                glMaterialfv(GL_FRONT, GL_DIFFUSE, ms_ruby.diffuse);
                glMaterialfv(GL_FRONT, GL_SPECULAR, ms_ruby.specular);
                glMaterialfv(GL_FRONT, GL_SHININESS, &ms_ruby.shininess);
                glutSolidCube(1);
            }
            //sphere
            else if (shape == SPHERE_SHAPE_PROXYTYPE)
            {
                glScaled(halfExtent[1], halfExtent[1], halfExtent[1]);
                glMaterialfv(GL_FRONT, GL_AMBIENT, ms_jade.ambient);
                glMaterialfv(GL_FRONT, GL_DIFFUSE, ms_jade.diffuse);
                glMaterialfv(GL_FRONT, GL_SPECULAR, ms_jade.specular);
                glMaterialfv(GL_FRONT, GL_SHININESS, &ms_jade.shininess);
                glutSolidSphere(1, 100, 100);
            }
            //capsule
            else if (shape == CAPSULE_SHAPE_PROXYTYPE)
            {
                glScaled(halfExtent[0], halfExtent[1], halfExtent[2]);
                glMaterialfv(GL_FRONT, GL_AMBIENT, ms_ruby.ambient);
                glMaterialfv(GL_FRONT, GL_DIFFUSE, ms_ruby.diffuse);
                glMaterialfv(GL_FRONT, GL_SPECULAR, ms_ruby.specular);
                glMaterialfv(GL_FRONT, GL_SHININESS, &ms_ruby.shininess);
                glutSolidCylinder(1, 1, 10, btVector3(0, 0, 0));
                
            }
            glPopMatrix();
        }
    }
    
    glPopMatrix();
    
    glDisable(GL_LIGHTING);
    
    glutSwapBuffers();
}

void Master::CleanupBullet() {
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
    for (int j = 0; j<Master::collisionShapes.size(); j++)
    {
        btCollisionShape* shape = Master::collisionShapes[j];
        Master::collisionShapes[j] = 0;
        delete shape;
    }
    for(int i = dynamicsWorld->getNumConstraints()-1; i>=0 ;i--){
        btTypedConstraint* constraint = dynamicsWorld->getConstraint(i);
        dynamicsWorld->removeConstraint(constraint);
        delete constraint;
    }
    delete dynamicsWorld;
    Master::collisionShapes.clear();
}

void Master::createGround() {
    btTransform groundTransform;
    btVector3 scale = btVector3(btScalar(100.), btScalar(10.), btScalar(100.));
    btScalar mass(0.);
    bool isDynamic = (mass != 0.f);
    btVector3 localInertia(0, 0, 0);
    if (isDynamic)
        Master::groundShape->calculateLocalInertia(mass, localInertia);
    
    for (int i = 0; i < NUM_GROUND; i++) {
        for (int j = 0; j < NUM_GROUND; j++) {
            Master::collisionShapes.push_back(Master::groundShape);
            groundTransform.setIdentity();
            groundTransform.setOrigin(btVector3((NUM_GROUND-1-i*2)*scale[0], -16, (NUM_GROUND-1-j*2)*scale[2]));
            
            btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
            btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, Master::groundShape, localInertia);
            btRigidBody* body = new btRigidBody(rbInfo);
            body->setActivationState(DISABLE_DEACTIVATION);
            body->setUserIndex(1+NUM_GROUND*i+j);
            
            dynamicsWorld->addRigidBody(body, RX_COL_GROUND, RX_COL_BODY | RX_COL_TF | RX_COL_AMP);
        }
    }
}

void Master::createStarfish() {
    starfish->create();
}

void Master::idle() {
    Master::time_step++;
    dynamicsWorld->stepSimulation(1.f / FPS, 10);
    starfish->idle();
}

void Master::init() {
    glClearColor(1.0, 1.0, 1.0, 1.0);
    glEnable(GL_DEPTH_TEST);
    //glEnable(GL_CULL_FACE);
    glLightfv(GL_LIGHT0, GL_POSITION, light0pos);
    glLightfv(GL_LIGHT1, GL_POSITION, light1pos);
    //glCullFace(GL_BACK);
    //glCullFace(GL_FRONT);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(30.0, (double)640 / (double)480, 0.1, 10000);
    gluLookAt(100,0,100, 0.0, 0, 0.0, 0.0, 1.0, 0.0);
}

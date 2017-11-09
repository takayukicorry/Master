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

void glutSolidCylinder(btScalar radius, btScalar halfheight, int num)
{
    glBegin(GL_POLYGON);
    
    for (int i = 0; i < num; i++) {
        glVertex3d(radius * cos((M_PI*2/num)*i), halfheight, radius * sin((M_PI*2/num)*i));
        glVertex3d(radius * cos((M_PI*2/num)*(i+1)), halfheight, radius * sin((M_PI*2/num)*(i+1)));
        glVertex3d(radius * cos((M_PI*2/num)*(i+1)), - halfheight, radius * sin((M_PI*2/num)*(i+1)));
        glVertex3d(radius * cos((M_PI*2/num)*i), - halfheight, radius * sin((M_PI*2/num)*i));
        
    }
    
    glEnd();
}

void glutSolidCapusule(btScalar radius, btScalar halfheight, int num)
{
    glutSolidCylinder(radius, halfheight, num);
    glTranslated(0, halfheight, 0);
    glPushMatrix();
    glScaled(radius, radius, radius);
    glutSolidSphere(1, 100, 100);
    glPopMatrix();
    glTranslated(0, -2*halfheight, 0);
    glScaled(radius, radius, radius);
    glutSolidSphere(1, 100, 100);
}

/***********************************/
/********                  *********/
/********                  *********/
/********                  *********/
/********                  *********/
/********                  *********/
/***********************************/

btDefaultCollisionConfiguration* Master::collisionConfiguration = new btDefaultCollisionConfiguration();
btCollisionDispatcher* Master::dispatcher = new btCollisionDispatcher(Master::collisionConfiguration);
btBroadphaseInterface* Master::overlappingPairCache = new btDbvtBroadphase();
btSequentialImpulseConstraintSolver* Master::solver = new btSequentialImpulseConstraintSolver;
btDiscreteDynamicsWorld* Master::dynamicsWorld = new btDiscreteDynamicsWorld(Master::dispatcher, Master::overlappingPairCache, Master::solver, Master::collisionConfiguration);
btCollisionShape* Master::groundShape = new btBoxShape(btVector3(btScalar(100.), btScalar(10.), btScalar(100.)));
int Master::time_step = 0;

Master::Master() {
    Master::dynamicsWorld->setGravity(btVector3(0, -10, 0));
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
    for (int j = Master::dynamicsWorld->getNumCollisionObjects() - 1; j >= 0; j--)
    {
        btCollisionObject* obj = Master::dynamicsWorld->getCollisionObjectArray()[j];
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
                glMaterialfv(GL_FRONT, GL_AMBIENT, ms_ruby.ambient);
                glMaterialfv(GL_FRONT, GL_DIFFUSE, ms_ruby.diffuse);
                glMaterialfv(GL_FRONT, GL_SPECULAR, ms_ruby.specular);
                glMaterialfv(GL_FRONT, GL_SHININESS, &ms_ruby.shininess);
                glutSolidCapusule(halfExtent[0], halfExtent[1], 10);

            }
            else if (shape == CYLINDER_SHAPE_PROXYTYPE)
            {
                glScaled(halfExtent[0], halfExtent[1], halfExtent[2]);
                glMaterialfv(GL_FRONT, GL_AMBIENT, ms_ruby.ambient);
                glMaterialfv(GL_FRONT, GL_DIFFUSE, ms_ruby.diffuse);
                glMaterialfv(GL_FRONT, GL_SPECULAR, ms_ruby.specular);
                glMaterialfv(GL_FRONT, GL_SHININESS, &ms_ruby.shininess);
                glutSolidCylinder(1, 1, 10);
            }
            glPopMatrix();
        }
    }
    
    glPopMatrix();
    
    glDisable(GL_LIGHTING);
    
    glutSwapBuffers();
}

void Master::CleanupStarfish() {
    for (int i = Master::dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
    {
        btCollisionObject* obj = Master::dynamicsWorld->getCollisionObjectArray()[i];
        if (obj->getUserIndex() < 100) {
            continue;
        }
        btRigidBody* body = btRigidBody::upcast(obj);
        if (body && body->getMotionState())
        {
            delete body->getMotionState();
        }
        Master::dynamicsWorld->removeCollisionObject(obj);
        delete obj;
    }
    
    for(int i = Master::dynamicsWorld->getNumConstraints()-1; i>=0 ;i--){
        btTypedConstraint* constraint = Master::dynamicsWorld->getConstraint(i);
        Master::dynamicsWorld->removeConstraint(constraint);
        delete constraint;
    }
}

void Master::CleanupBullet() {
    for (int i = Master::dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
    {
        btCollisionObject* obj = Master::dynamicsWorld->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(obj);
        if (body && body->getMotionState())
        {
            delete body->getMotionState();
        }
        Master::dynamicsWorld->removeCollisionObject(obj);
        delete obj;
    }
    
    for(int i = Master::dynamicsWorld->getNumConstraints()-1; i>=0 ;i--){
        btTypedConstraint* constraint = Master::dynamicsWorld->getConstraint(i);
        Master::dynamicsWorld->removeConstraint(constraint);
        delete constraint;
    }
    delete Master::dynamicsWorld;
    delete Master::solver;
    delete Master::overlappingPairCache;
    delete Master::dispatcher;
    delete Master::collisionConfiguration;
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
            groundTransform.setIdentity();
            groundTransform.setOrigin(btVector3((NUM_GROUND-1-i*2)*scale[0], -16, (NUM_GROUND-1-j*2)*scale[2]));
            
            btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
            btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, Master::groundShape, localInertia);
            btRigidBody* body = new btRigidBody(rbInfo);
            body->setActivationState(DISABLE_DEACTIVATION);
            body->setUserIndex(1+NUM_GROUND*i+j);
            
            Master::dynamicsWorld->addRigidBody(body, RX_COL_GROUND, RX_COL_BODY | RX_COL_TF | RX_COL_AMP);
        }
    }
}

void Master::createStarfish() {
    starfish->create();
}

void Master::checkStarfish() {
    if (!starfish->checkState()) {
        CleanupStarfish();
        Starfish* oph;
        if (strcmp("Ophiuroid2",typeid(starfish).name())) {
            oph= new Ophiuroid();
        } else {
            oph= new Ophiuroid2();
        }
        setStarfish(oph);
        createStarfish();
    }
}

void Master::idle() {
    Master::time_step++;
    Master::dynamicsWorld->stepSimulation(1.f / FPS, 10);
    
    starfish->idle();
    checkStarfish();
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
    gluLookAt(100,50,100, 0.0, 0, 0.0, 0.0, 1.0, 0.0);
}

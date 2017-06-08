//
// proj301
//

//#include "btBulletDynamicsCommon.h"
#include <BulletDynamics/btBulletDynamicsCommon.h>
#include <stdio.h>
#include <stdlib.h>
//#include <GL/glut.h>
#include <GLUT/GLUT.h>
#include <OpenGL/OpenGL.h>
#include <iostream>
#include <vector>
#include "Variables.h"
#include <math.h>
#include <map>
//#include <OpenGL/DemoApplication.h>

using namespace std;

btDefaultCollisionConfiguration* collisionConfiguration;
btCollisionDispatcher* dispatcher;
btBroadphaseInterface* overlappingPairCache;
btSequentialImpulseConstraintSolver* solver;
btDiscreteDynamicsWorld* dynamicsWorld;
btCollisionShape* groundShape;
btAlignedObjectArray<btCollisionShape*> collisionShapes;


GLfloat light0pos[] = { 300.0, 300.0, 300.0, 1.0 };
GLfloat light1pos[] = { -300.0, 300.0, 300.0, 1.0 };

//Userindex==intÇÃÅ`
map<int, bool> TF_contact;//ä«ë´Ç™ãzíÖÇµÇƒÇÈÇ©Ç«Ç§Ç©
map<int, bool> TF_light;//ä«ë´Ç™åıéÛêMÇµÇƒÇÈÇ«Ç§Ç©
map<int, btTypedConstraint*> TF_constraint_amp;//ä«ë´Ç∆ïrîXÇÃçSë©
map<int, btTypedConstraint*> TF_constraint_ground;//ä«ë´Ç∆ínñ ÇÃçSë©
map<int, btRigidBody*> BODY_object;//ì∑ëÃ(Rigid Body)
map<int, btRigidBody*> TF_object;//ä«ë´(Rigid Body)
map<int, btRigidBody*> TF_object_amp;//ä«ë´Ç∆åqÇ™Ç¡ÇƒÇÈïrîXÅiRigid BodyÅj
map<int, btRotationalLimitMotor* > motor_tY;//ä«ë´Ç∆ïrîXÇÃÉÇÅ[É^Å[ÅiÉnÉìÉhÉãÅj
map<int, btRotationalLimitMotor* > motor_tZ;//ä«ë´Ç∆ïrîXÇÃÉÇÅ[É^Å[Åié‘ó÷Åj
map<int, btRotationalLimitMotor* > motor_to_groundY;//ä«ë´Ç∆ínñ ÇÃÉÇÅ[É^Å[ÅiÉnÉìÉhÉãÅj
map<int, btRotationalLimitMotor* > motor_to_groundZ;//ä«ë´Ç∆ínñ ÇÃÉÇÅ[É^Å[Åié‘ó÷Åj
map<int, int> ResumeTime_tf;//ä«ë´Ç∆ïrîXÇÃÉÇÅ[É^Å[ÇÃäJénéûçè


int time_step = 0;

enum CollisionGroup{
    RX_COL_NOTHING = 0, // 0000
    RX_COL_GROUND = 1, // 0001
    RX_COL_BODY = 2,  // 0010
    RX_COL_TF = 4,  // 0100
    RX_COL_AMP = 8   // 1000
};

//----------------------------------------------------
// ï®éøéøä¥ÇÃíËã`
//----------------------------------------------------
struct MaterialStruct {
	GLfloat ambient[4];
	GLfloat diffuse[4];
	GLfloat specular[4];
	GLfloat shininess;
};
//jade(„≈êâ)
MaterialStruct ms_jade = {
	{ 0.135, 0.2225, 0.1575, 1.0 },
	{ 0.54, 0.89, 0.63, 1.0 },
	{ 0.316228, 0.316228, 0.316228, 1.0 },
	12.8 };
//ruby(ÉãÉrÅ[)
MaterialStruct ms_ruby = {
	{ 0.1745, 0.01175, 0.01175, 1.0 },
	{ 0.61424, 0.04136, 0.04136, 1.0 },
	{ 0.727811, 0.626959, 0.626959, 1.0 },
	76.8 };
//----------------------------------------------------
// êFÇÃíËã`ÇÃíËã`
//----------------------------------------------------
GLfloat red[] = { 0.8, 0.2, 0.2, 1.0 }; //ê‘êF
GLfloat green[] = { 0.2, 0.8, 0.2, 1.0 };//óŒêF
GLfloat blue[] = { 0.2, 0.2, 0.8, 1.0 };//ê¬êF
GLfloat yellow[] = { 0.8, 0.8, 0.2, 1.0 };//â©êF
GLfloat white[] = { 1.0, 1.0, 1.0, 1.0 };//îíêF
GLfloat shininess = 30.0;//åıëÚÇÃã≠Ç≥
//-----------------------------------------
//åvéZÇ∑ÇÈä÷êî
//-----------------------------------------

btVector3 RotateY(const btVector3 bef, double alpha)
{
    btVector3 aft = bef;
    aft[0] = bef[0] * cos(alpha) - bef[2] * sin(alpha);
    aft[2] = bef[0] * sin(alpha) + bef[2] * cos(alpha);
    
    return aft;
}

btVector3 acrossb(btVector3 r, btVector3 t)
{
    btVector3 b = {0, 0, 0};
    b[0]=r[1]*t[2]-r[2]*t[1];
    b[1]=r[2]*t[0]-r[0]*t[2];
    b[2]=r[0]*t[1]-r[1]*t[0];
    
    return b;
}

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

btRigidBody::btRigidBodyConstructionInfo calcInertia(btScalar mass, btVector3 position)
{

    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(position);
    
    // ê√ìIÇ»çÑëÃÇçÏÇËÇ‹Ç∑
    bool isDynamic = (mass != 0.f);
    
    btVector3 localInertia(0, 0, 0);
    if (isDynamic)
        groundShape->calculateLocalInertia(mass, localInertia);
        
    // ÉfÉtÉHÉãÉgÇÃÉÇÅ[ÉVÉáÉìÉXÉeÅ[ÉgÇçÏê¨
    btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
    
    return rbInfo;
}

btRigidBody* getByUserIndex(int index)
{
    for (int i = dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
    {
        btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
        if (obj->getUserIndex()==index) {
            btRigidBody* body = btRigidBody::upcast(obj);
            return body;
        }
    }
    
    assert(1);
    
    btCollisionShape* colShape = new btSphereShape(1);
    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(btVector3(0, -56, 0));
    btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(0, myMotionState, colShape, btVector3(0,0,0));
    btRigidBody* body = new btRigidBody(rbInfo);
    return body;
}
//---------------------------------------------

// ÉOÉâÉìÉhÇÃê∂ê¨
void CreateGround()
{
	// ÉVÉFÉCÉvÇÃê∂ê¨
    btVector3 scale = btVector3(btScalar(500.), btScalar(50.), btScalar(500.));
    groundShape = new btBoxShape(scale);
	collisionShapes.push_back(groundShape);
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, -56, 0));

	btScalar mass(0.);

	// ê√ìIÇ»çÑëÃÇçÏÇËÇ‹Ç∑
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		groundShape->calculateLocalInertia(mass, localInertia);

	// ÉfÉtÉHÉãÉgÇÃÉÇÅ[ÉVÉáÉìÉXÉeÅ[ÉgÇçÏê¨
	btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);

    body->setActivationState(DISABLE_DEACTIVATION);
    body->setUserIndex(1);
    
	// çÏê¨ÇµÇΩçÑëÃÇÉèÅ[ÉãÉhÇ÷ìoò^
    dynamicsWorld->addRigidBody(body, RX_COL_GROUND, RX_COL_BODY | RX_COL_TF | RX_COL_AMP);
    ///dynamicsWorld->addRigidBody(body);
}

// ãÖÇÃê∂ê¨
btRigidBody* initAmp(btScalar scale, const btVector3 position)
{
	btCollisionShape* colShape = new btSphereShape(scale);
	collisionShapes.push_back(colShape);

	// ÉgÉâÉìÉXÉtÉHÅ[ÉÄÇçÏê¨
	btTransform startTransform;
	startTransform.setIdentity();

	btScalar mass(0.f);

	// éøó Ç1Ç∆ÇµÇΩÇÃÇ≈É_ÉCÉiÉ~ÉbÉN
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		colShape->calculateLocalInertia(mass, localInertia);

	startTransform.setOrigin(position);

	// ÉÇÅ[ÉVÉáÉìÉXÉeÅ[ÉgÇçÏê¨
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);
    
    body->setActivationState(DISABLE_DEACTIVATION);

    return  body;
}

//ÉqÉgÉfÇÃì∑ëÃê∂ê¨
btRigidBody* initBody(const btVector3 scale, const btVector3 position)
{
    btCollisionShape* sBodyShape = new btBoxShape(scale);
    collisionShapes.push_back(sBodyShape);
    
    btTransform sBodyTransform;
    sBodyTransform.setIdentity();
    sBodyTransform.setOrigin(position);
    
    btScalar mass(M_BODY);
    
    bool isDynamic = (mass != 0.f);
    
    btVector3 localInertia(0, 0, 0);
    if (isDynamic)
        groundShape->calculateLocalInertia(mass, localInertia);
    
    // ÉfÉtÉHÉãÉgÇÃÉÇÅ[ÉVÉáÉìÉXÉeÅ[ÉgÇçÏê¨
    btDefaultMotionState* myMotionState = new btDefaultMotionState(sBodyTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, sBodyShape, localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);
    
    body->setActivationState(DISABLE_DEACTIVATION);
    
    return body;
}

//ÉqÉgÉfÇÃòrê∂ê¨
btRigidBody* initArm(const btVector3 scale, const btVector3 position, const btQuaternion rot)
{
	btCollisionShape* sBodyShape = new btBoxShape(scale);
	collisionShapes.push_back(sBodyShape);

	btScalar mass1(M_ARM);
	bool isDynamic = (mass1 != 0.f);

	btVector3 localInertia1(0, 0, 0);
	if (isDynamic)
		groundShape->calculateLocalInertia(mass1, localInertia1);

	// ÉfÉtÉHÉãÉgÇÃÉÇÅ[ÉVÉáÉìÉXÉeÅ[ÉgÇçÏê¨

	btTransform sBodyTransform;
	sBodyTransform.setIdentity();
	sBodyTransform.setOrigin(position);
    sBodyTransform.setRotation(rot);
	btDefaultMotionState* myMotionState1 = new btDefaultMotionState(sBodyTransform);

	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass1, myMotionState1, sBodyShape, localInertia1);
	btRigidBody* body = new btRigidBody(rbInfo);
    
    body->setActivationState(DISABLE_DEACTIVATION);
    
    return body;

}

//ÉqÉgÉfÇÃä«ë´ê∂ê¨
btRigidBody* initTubefeet(btScalar* scale, const btVector3 position)
{
    btCollisionShape* sBodyShape = new btCapsuleShape(scale[0], scale[1]);
    collisionShapes.push_back(sBodyShape);
    
    btScalar mass1(M_TF);
    bool isDynamic = (mass1 != 0.f);
    
    btVector3 localInertia1(0, 0, 0);
    if (isDynamic)
        groundShape->calculateLocalInertia(mass1, localInertia1);
    
    // ÉfÉtÉHÉãÉgÇÃÉÇÅ[ÉVÉáÉìÉXÉeÅ[ÉgÇçÏê¨
    
    btTransform sBodyTransform;
    sBodyTransform.setIdentity();
    sBodyTransform.setOrigin(position);
    btDefaultMotionState* myMotionState1 = new btDefaultMotionState(sBodyTransform);
    
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass1, myMotionState1, sBodyShape, localInertia1);
    btRigidBody* body = new btRigidBody(rbInfo);
    
    body->setActivationState(DISABLE_DEACTIVATION);
    
    return body;
}

// ÉqÉgÉfÇÃê∂ê¨
void CreateStarfish()
{
    vector<btRigidBody* > bodies_body;
    vector<btRigidBody* > bodies_tf;
    vector<btRigidBody* > bodies_amp;
    vector<btTypedConstraint* > constraints;
    
/***ëÃ***/
    /*btRigidBody* body_body = initBody(btVector3(RADIUS*2, LENGTH, RADIUS*2), btVector3(0, INIT_POS_Y-RADIUS*2, 0));
    BODY_object[0] = body_body;
    bodies_body.push_back(body_body);
    for (int i = 0; i < 5; i++) {
        btRigidBody* body_arm = initArm(btVector3(RADIUS*6, LENGTH, RADIUS*2), RotateY(btVector3(RADIUS*10, INIT_POS_Y-RADIUS*2, 0), M_PI*2*i/5), btQuaternion(btVector3(0, 1, 0), M_PI*2*i/5));
        BODY_object[i+1] = body_arm;
        bodies_body.push_back(body_arm);
    }
    
/***ä«ë´***/
    btScalar scale[] = {btScalar(RADIUS), btScalar(LENGTH)};
    btVector3 pos_tf, pos_amp;
    int col, row;
    int h = INIT_POS_Y-RADIUS*2-LENGTH/2;
    int from_x = RADIUS*2;
    for (int i = 0; i < 30; i++) {
        col = i % 2;
        row = i / 2;
        pos_tf = btVector3(from_x + row * RADIUS * 4, h, pow(-1, col) * RADIUS * 2);
        pos_amp = btVector3(from_x + row * RADIUS * 4, INIT_POS_Y, pow(-1, col) * RADIUS * 2);
        for (int j = 1; j <= 5; j++) {
            //body
            btRigidBody* body_amp = initAmp(btScalar(RADIUS), pos_amp);
            btRigidBody* body_tf = initTubefeet(scale, pos_tf);
            int index = 100 + 10 * i + j;
            body_tf->setUserIndex(index);
            TF_object[index] = body_tf;
            TF_object_amp[index] = body_amp;
            TF_contact[index] = false;
            bodies_tf.push_back(body_tf);
            bodies_amp.push_back(body_amp);
            //constraint
            btUniversalConstraint* univ = new btUniversalConstraint(*body_amp, *body_tf, pos_amp, btVector3(0, 1, 0), btVector3(0, 0, 1));//global
            univ->setLowerLimit(-ANGLE, -ANGLE);
            univ->setUpperLimit(ANGLE, ANGLE);
            TF_constraint_amp[index] = univ;
            constraints.push_back(univ);
            //motor
            btRotationalLimitMotor* motor1 = univ->getRotationalLimitMotor(1);//wheel
            btRotationalLimitMotor* motor2 = univ->getRotationalLimitMotor(2);//handle
            motor1->m_enableMotor = true;
            motor2->m_enableMotor = true;
            motor_tZ[index] = motor1;
            motor_tY[index] = motor2;
            ResumeTime_tf[index] = -SECOND*col;
            //for next
            pos_tf = RotateY(pos_tf, M_PI*2/5);
            pos_amp = RotateY(pos_amp, M_PI*2/5);
        }
    }

    
    ////ìoò^////
    
    for (int i = 0; i < bodies_body.size(); i++) {
        dynamicsWorld->addRigidBody(bodies_body[i], RX_COL_BODY, RX_COL_GROUND);
    }
    
    for (int i = 0; i < bodies_tf.size(); i++) {
        dynamicsWorld->addRigidBody(bodies_tf[i], RX_COL_TF, RX_COL_GROUND);
    }

    for (int i = 0; i < bodies_amp.size(); i++) {
        dynamicsWorld->addRigidBody(bodies_amp[i], RX_COL_AMP, RX_COL_GROUND);
    }
    
    for (int i = 0; i < constraints.size(); i++) {
        dynamicsWorld->addConstraint(constraints[i]);
    }

}

void ControllTubeFeet()
{
    
    btScalar velocity_all = 0;
    for (auto itr = TF_contact.begin(); itr != TF_contact.end(); ++itr) {
        
        int index = itr->first;
        btRigidBody* body = TF_object[index];
        
        if (body && body->getMotionState() && TF_contact[index])
        {
            btScalar velocity_x = body->getLinearVelocity()[0];
            
            if (velocity_x < velocity_all) {
                velocity_all = velocity_x;
            }
        }
    }
    
    //ñ{ëÃÇÃëOå„ÇÃìÆÇ´
    for (auto itr = BODY_object.begin(); itr != BODY_object.end(); ++itr) {
        btRigidBody* body = itr->second;
        
        if (body && body->getMotionState())
        {
            btVector3 pos = body->getCenterOfMassPosition();
            
            btTransform tran;
            tran.setIdentity();
            tran.setOrigin(btVector3(pos[0]+velocity_all*3.5/FPS, pos[1], pos[2]));
            
            body->setCenterOfMassTransform(tran);
        }
    }
    
    //ïrîXÇÃè„â∫ÅïëOå„ÇÃìÆÇ´
    for (auto itr = TF_object_amp.begin(); itr != TF_object_amp.end(); ++itr) {
        
        int index = itr->first;
        btRigidBody* body = itr->second;
    
        if (body && body->getMotionState() && !TF_contact[index])
        {
            btVector3 pos = body->getCenterOfMassPosition();
        
            btTransform tran;
            tran.setIdentity();
            tran.setOrigin(btVector3(pos[0]+velocity_all*3.5/FPS, INIT_POS_Y - (LENGTH/2 + 4) + (LENGTH/2 + 4)*sin(2*M_PI*(time_step%(SECOND*2))/(SECOND*2) + M_PI_2), pos[2]));
        
            body->setCenterOfMassTransform(tran);
        }
    }
    
    //ä«ë´ÇÃÉÇÅ[É^Å[
    for (auto itr = motor_tZ.begin(); itr != motor_tZ.end(); ++itr) {
        
        int index = itr->first;
        btRotationalLimitMotor* motor = itr->second;
        
        motor->m_maxMotorForce = 100000000;
        motor_tY[index]->m_maxMotorForce = 100000000;

        if (!TF_contact[index])
        {
            if (((time_step-ResumeTime_tf[index])/SECOND+1) / 2 % 2 == 0) {
                motor->m_targetVelocity = ANGLE_VELOCITY_TF;
            }else{
                motor->m_targetVelocity = -ANGLE_VELOCITY_TF;
            }
        }
        else
        {
            motor->m_targetVelocity = 0;
            
        }
    }
    
    //ínñ Ç∆ÇÃÉÇÅ[É^Å[
    for (auto itr = motor_to_groundZ.begin(); itr != motor_to_groundZ.end(); ++itr) {
        
        int index = itr->first;
        btRotationalLimitMotor* motor = itr->second;
        
        motor->m_maxMotorForce = 100000000;
        motor_to_groundY[index]->m_maxMotorForce = 100000000;
        
        motor->m_targetVelocity = -ANGLE_VELOCITY_GROUND;
    }
    

}

void ContactAction()
{
    vector<int > contacts;
    
    int numManifolds = dynamicsWorld->getDispatcher()->getNumManifolds();
    for (int i = 0; i < numManifolds; i++)
    {
        //ä«ë´è’ìÀèÛë‘çXêV
        btPersistentManifold* contactManifold =  dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
        const btCollisionObject* obA = contactManifold->getBody0();
        const btCollisionObject* obB = contactManifold->getBody1();
        btRigidBody* bodyA = btRigidBody::upcast(const_cast<btCollisionObject *>(obA));
        btRigidBody* bodyB = btRigidBody::upcast(const_cast<btCollisionObject *>(obB));

        int obID = obA->getUserIndex();
        
        //ínñ Ç∆ÇÃè’ìÀÇæÇ¡ÇΩéû
        if (obID==1) {
            
            //è’ìÀèÓïÒéÊìæ
            int numContacts = contactManifold->getNumContacts();
            
            for (int j = 0; j < numContacts; j++)
            {
                btManifoldPoint& pt = contactManifold->getContactPoint(j);
                if (pt.getDistance() < 0.5f)
                {
                    ///const btVector3& ptA = pt.getPositionWorldOnA();
                    const btVector3& ptB = pt.getPositionWorldOnB();
                    ///const btVector3& normalOnB = pt.m_normalWorldOnB;

                    int index = obB->getUserIndex();
                    
                    btVector3 euler;
                    bodyB->getWorldTransform().getBasis().getEulerZYX(euler[2], euler[1], euler[0]);
                    double angle = euler[2];
                    
                    //ãzíÖÇµÇƒÇ»Ç©Ç¡ÇΩèÍçá
                    if (!TF_contact[index] && angle<ANGLE_ATTACH) {
                        
                        dynamicsWorld->removeRigidBody(TF_object_amp[index]);
                        dynamicsWorld->removeConstraint(TF_constraint_amp[index]);
                        
                        TF_contact[index] = true;
                        
                        
                        //è’ìÀì_Ç…ínñ Ç∆ÇÃçSë©çÏê¨
                        btUniversalConstraint* univ = new btUniversalConstraint(*bodyA, *bodyB, btVector3(ptB[0],ptB[1]+RADIUS ,ptB[2] ), btVector3(0, 1, 0), btVector3(0, 0, 1));//ëSïîÉOÉçÅ[ÉoÉã
                        //univ->setLowerLimit(-ANGLE, -ANGLE);
                        //univ->setUpperLimit(ANGLE, ANGLE);
                        TF_constraint_ground[index] = univ;
                        dynamicsWorld->addConstraint(univ);
                    
                        //ÉÇÅ[É^Å[
                        btRotationalLimitMotor* motor1 = univ->getRotationalLimitMotor(1);//é‘ó÷
                        btRotationalLimitMotor* motor2 = univ->getRotationalLimitMotor(2);//ÉXÉeÉAÉäÉìÉO
                        motor1->m_enableMotor = true;
                        motor2->m_enableMotor = true;
                        motor_to_groundZ[index] = motor1;
                        motor_to_groundY[index] = motor2;
                    }
                    //ãzíÖÇµÇƒÇΩèÍçá
                    else if (TF_contact[index])
                    {
                        /**ínñ Ç©ÇÁó£íEÅ©äpìxîªíË**/
                        if (angle>ANGLE_DETACH)
                        {
                            /*ínñ Ç∆ÇÃçSë©çÌèú*/
                            dynamicsWorld->removeConstraint(TF_constraint_ground[index]);
                            motor_to_groundY.erase(index);
                            motor_to_groundZ.erase(index);
                            TF_contact[index] = false;
                            
                            /*ïrîXïúäà*/
                            btVector3 pos_tf = bodyB->getCenterOfMassPosition();
                            btVector3 pos_amp = btVector3(pos_tf[0]-(LENGTH/2+RADIUS*2)*sin(angle), pos_tf[1]+(LENGTH/2+RADIUS*2)*cos(angle), pos_tf[2]);
                            
                            btRigidBody* body_amp = initAmp(RADIUS, pos_amp);
                            TF_object_amp[index] = body_amp;
                            dynamicsWorld->addRigidBody(body_amp);
                            
                            /*ïrîXÇ∆ä«ë´ÇÃÉÇÅ[É^Å[*/
                            btUniversalConstraint* univ = new btUniversalConstraint(*body_amp, *TF_object[index], pos_amp, btVector3(-sin(angle), cos(angle), 0), btVector3(0, 0, 1));
                            univ->setLowerLimit(-ANGLE+angle, -ANGLE);
                            univ->setUpperLimit(ANGLE+angle, ANGLE);
                            TF_constraint_amp[index] = univ;
                            dynamicsWorld->addConstraint(univ);
                            
                            btRotationalLimitMotor* motor1 = univ->getRotationalLimitMotor(1);
                            btRotationalLimitMotor* motor2 = univ->getRotationalLimitMotor(2);
                            motor1->m_enableMotor = true;
                            motor2->m_enableMotor = true;
                            motor_tZ[index] = motor1;
                            motor_tY[index] = motor2;
                            ResumeTime_tf[index] = time_step;
                            
                        }
                    }
                }
            }
        }
    }
}

void Render()
{
    time_step++;
    dynamicsWorld->stepSimulation(1.f / FPS, 10);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//glLightfv(GL_LIGHT0, GL_POSITION, light0pos);

	glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    //glDisable(GL_LIGHT1);

	/* ÉÇÉfÉãÉrÉÖÅ[ïœä∑çsóÒÇÃï€ë∂ */
	glPushMatrix();
    
	// çÑëÃÇÃç¿ïWÇÉvÉäÉìÉg
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
            //ínñ 
            if (j == 0)
			{
				glScaled(2 * halfExtent[0], 2 * halfExtent[1], 2 * halfExtent[2]);
				glMaterialfv(GL_FRONT, GL_AMBIENT, ms_jade.ambient);
				glMaterialfv(GL_FRONT, GL_DIFFUSE, ms_jade.diffuse);
				glMaterialfv(GL_FRONT, GL_SPECULAR, ms_jade.specular);
				glMaterialfv(GL_FRONT, GL_SHININESS, &ms_jade.shininess);
				glutSolidCube(1);
			}
            //ì∑ëÃÅ@òr
			else if (shape == BOX_SHAPE_PROXYTYPE)
			{
				glScaled(2 * halfExtent[0], 2 * halfExtent[1], 2 * halfExtent[2]);
				glMaterialfv(GL_FRONT, GL_AMBIENT, ms_ruby.ambient);
				glMaterialfv(GL_FRONT, GL_DIFFUSE, ms_ruby.diffuse);
				glMaterialfv(GL_FRONT, GL_SPECULAR, ms_ruby.specular);
				glMaterialfv(GL_FRONT, GL_SHININESS, &ms_ruby.shininess);
				glutSolidCube(1);
			}
            //ïrîX
			else if (shape == SPHERE_SHAPE_PROXYTYPE)
			{/*
                glScaled(halfExtent[1], halfExtent[1], halfExtent[1]);
				glMaterialfv(GL_FRONT, GL_AMBIENT, ms_ruby.ambient);
				glMaterialfv(GL_FRONT, GL_DIFFUSE, ms_ruby.diffuse);
				glMaterialfv(GL_FRONT, GL_SPECULAR, ms_ruby.specular);
				glMaterialfv(GL_FRONT, GL_SHININESS, &ms_ruby.shininess);
				glutSolidSphere(1, 100, 100);*/
			}
            //ä«ë´
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

	/* ÉÇÉfÉãÉrÉÖÅ[ïœä∑çsóÒÇÃïúãA */
	glPopMatrix();

	glDisable(GL_LIGHTING);

	glutSwapBuffers();

}

void init(void)
{
	glClearColor(1.0, 1.0, 1.0, 1.0);
	glEnable(GL_DEPTH_TEST);
	//glEnable(GL_CULL_FACE);
	glLightfv(GL_LIGHT0, GL_POSITION, light0pos);
    glLightfv(GL_LIGHT1, GL_POSITION, light1pos);
    //glCullFace(GL_BACK);
	//glCullFace(GL_FRONT);

	glMatrixMode(GL_PROJECTION);//çsóÒÉÇÅ[ÉhÇÃê›íËÅiGL_PROJECTION : ìßéãïœä∑çsóÒÇÃê›íËÅAGL_MODELVIEWÅFÉÇÉfÉãÉrÉÖÅ[ïœä∑çsóÒÅj
	glLoadIdentity();//çsóÒÇÃèâä˙âª
	gluPerspective(30.0, (double)640 / (double)480, 0.1, 10000);
	gluLookAt(0, 700, 1000, 0.0, 0, 0.0, 0.0, 1.0, 0.0);
}

void idle(void)
{
    ContactAction();
	glutPostRedisplay();
    ControllTubeFeet();
}

// èIóπèàóù
void CleanupBullet()
{
    // çÑëÃÇçÌèú
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
    
    // è’ìÀÉVÉFÉCÉvÇçÌèú
    for (int j = 0; j<collisionShapes.size(); j++)
    {
        btCollisionShape* shape = collisionShapes[j];
        collisionShapes[j] = 0;
        delete shape;
    }
    
    for(int i = dynamicsWorld->getNumConstraints()-1; i>=0 ;i--){
        btTypedConstraint* constraint = dynamicsWorld->getConstraint(i);
        dynamicsWorld->removeConstraint(constraint);
        delete constraint;
    }
    
    // ÉèÅ[ÉãÉhÇçÌèú
    delete dynamicsWorld;
    
    // É\ÉãÉoÅ[ÇçÌèú
    delete solver;
    
    // è’ìÀåüímÇÃÉuÉçÅ[ÉhÉtÉFÉCÉYÇçÌèú
    delete overlappingPairCache;
    
    // ÉfÉBÉXÉpÉbÉ`ÉÉÇçÌèú
    delete dispatcher;
    
    delete collisionConfiguration;
    
    // ÉIÉvÉVÉáÉìÅAÇ»Ç≠ÇƒÇ‡ó«Ç¢
    collisionShapes.clear();
}

// èâä˙âª
void InitBullet()
{
    // ÉfÉtÉHÉãÉgÇÃè’ìÀåüímÉAÉãÉSÉäÉYÉÄ
    collisionConfiguration = new btDefaultCollisionConfiguration();
    
    // ÉfÉtÉHÉãÉgÇÃè’ìÀÉfÉBÉXÉpÉbÉ`ÉÉ
    dispatcher = new btCollisionDispatcher(collisionConfiguration);
    
    // äKëwìIÇ»è’ìÀåüímÉAÉãÉSÉäÉYÉÄ
    overlappingPairCache = new btDbvtBroadphase();
    
    // ÉVÅ[ÉPÉìÉVÉÉÉã(îÒï¿óÒ)Ç»ÉCÉìÉpÉãÉXâñ@
    solver = new btSequentialImpulseConstraintSolver;
    
    dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
    
    dynamicsWorld->setGravity(btVector3(0, -10, 0));
}



// ÉÅÉCÉì
int main(int argc, char** argv)
{
    // èâä˙âª
    InitBullet();

	// ÉOÉâÉìÉhÇÃçÏê¨
	CreateGround();

    //ÉqÉgÉfÇÃê∂ê¨
    CreateStarfish();
	
	glutInitWindowPosition(100, 100);
	glutInitWindowSize(640, 480);
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
	glutCreateWindow(argv[0]);
	glutDisplayFunc(Render);
	glutIdleFunc(idle);
	init();
	glutMainLoop();

	// èIóπèàóù
	CleanupBullet();
}

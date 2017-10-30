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
#include <random>
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

//Userindex==int
map<int, bool> TF_contact;//tubefeet - ground (attach)
map<int, int> TF_contact_times;//tubefeet - ground (attach times)
map<int, btTypedConstraint*> TF_constraint_amp;//tubefeet - amp (constraint)
map<int, btTypedConstraint*> TF_constraint_ground;//tubefeet - ground (constraint)
map<int, btRigidBody*> BODY_object;//arm (object)
map<int, btRigidBody*> TF_object;//tubefeet (object)
map<int, btRigidBody*> TF_object_amp;//amp (object)
map<int, btRotationalLimitMotor* > motor_tY;//tubefeet - amp (handle motor)
map<int, btRotationalLimitMotor* > motor_tZ;//tubefeet - amp (wheel motor)
map<int, btRotationalLimitMotor* > motor_to_groundY;//tubefeet - ground (handle motor)
map<int, btRotationalLimitMotor* > motor_to_groundZ;//tubefeet - ground (wheel motor)
map<int, btVector3> TF_axis_direction;//tubefeet - amp & tubefeet - ground (motor direction)
map<int, btScalar> TF_axis_angle;//tubefeet - amp & tubefeet - ground (current motor angle to XZ)
map<int, int> DeleteTime_tf;//time to delete tf - ground
map<int, int> ResumeTime_tf;//time to start swinging

int time_step = 0;

enum CollisionGroup{
    RX_COL_NOTHING = 0, // 0000
    RX_COL_GROUND = 1, // 0001
    RX_COL_BODY = 2,  // 0010
    RX_COL_TF = 4,  // 0100
    RX_COL_AMP = 8   // 1000
};

//----------------------------------------------------
// color definition
//----------------------------------------------------
struct MaterialStruct {
	GLfloat ambient[4];
	GLfloat diffuse[4];
	GLfloat specular[4];
	GLfloat shininess;
};
//jade
MaterialStruct ms_jade = {
	{ 0.135, 0.2225, 0.1575, 1.0 },
	{ 0.54, 0.89, 0.63, 1.0 },
	{ 0.316228, 0.316228, 0.316228, 1.0 },
	12.8 };
//ruby
MaterialStruct ms_ruby = {
	{ 0.1745, 0.01175, 0.01175, 1.0 },
	{ 0.61424, 0.04136, 0.04136, 1.0 },
	{ 0.727811, 0.626959, 0.626959, 1.0 },
	76.8 };
//----------------------------------------------------
// color definition
//----------------------------------------------------
GLfloat red[] = { 0.8, 0.2, 0.2, 1.0 }; //ê‘êF
GLfloat green[] = { 0.2, 0.8, 0.2, 1.0 };//óŒêF
GLfloat blue[] = { 0.2, 0.2, 0.8, 1.0 };//ê¬êF
GLfloat yellow[] = { 0.8, 0.8, 0.2, 1.0 };//â©êF
GLfloat white[] = { 1.0, 1.0, 1.0, 1.0 };//îíêF
GLfloat shininess = 30.0;//åıëÚÇÃã≠Ç≥
//-----------------------------------------
//private function
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
    
    btCollisionShape* colShape = new btSphereShape(1);
    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(btVector3(0, -56, 0));
    btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(0, myMotionState, colShape, btVector3(0,0,0));
    btRigidBody* body = new btRigidBody(rbInfo);
    return body;
}
//-----------------------------------------
//function
//-----------------------------------------

// create a ground
void CreateGround()
{
    btTransform groundTransform;
    btVector3 scale = btVector3(btScalar(100.), btScalar(10.), btScalar(100.));
    groundShape = new btBoxShape(scale);
    btScalar mass(0.);
    bool isDynamic = (mass != 0.f);
    btVector3 localInertia(0, 0, 0);
    if (isDynamic)
        groundShape->calculateLocalInertia(mass, localInertia);
    
    for (int i = 0; i < NUM_GROUND; i++) {
        for (int j = 0; j < NUM_GROUND; j++) {
            collisionShapes.push_back(groundShape);
            groundTransform.setIdentity();
            groundTransform.setOrigin(btVector3((NUM_GROUND-1-i*2)*scale[0], -16, (NUM_GROUND-1-j*2)*scale[2]));
            
            btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
            btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
            btRigidBody* body = new btRigidBody(rbInfo);
            body->setActivationState(DISABLE_DEACTIVATION);
            body->setUserIndex(1+NUM_GROUND*i+j);
            
            dynamicsWorld->addRigidBody(body, RX_COL_GROUND, RX_COL_BODY | RX_COL_TF | RX_COL_AMP);
        }
    }
}

// create an amp
btRigidBody* initAmp(btScalar scale, const btVector3 position)
{
	btCollisionShape* colShape = new btSphereShape(scale);
	collisionShapes.push_back(colShape);

	
	btTransform startTransform;
	startTransform.setIdentity();

	btScalar mass(0.f);

	
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		colShape->calculateLocalInertia(mass, localInertia);

	startTransform.setOrigin(position);

	
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);
    
    body->setActivationState(DISABLE_DEACTIVATION);

    return  body;
}

//create body
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
    
    
    btDefaultMotionState* myMotionState = new btDefaultMotionState(sBodyTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, sBodyShape, localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);
    
    body->setActivationState(DISABLE_DEACTIVATION);
    
    return body;
}

//create an arm
btRigidBody* initArm(const btVector3 scale, const btVector3 position, const btQuaternion rot)
{
	btCollisionShape* sBodyShape = new btBoxShape(scale);
	collisionShapes.push_back(sBodyShape);

	btScalar mass1(M_ARM);
	bool isDynamic = (mass1 != 0.f);

	btVector3 localInertia1(0, 0, 0);
	if (isDynamic)
		groundShape->calculateLocalInertia(mass1, localInertia1);

	
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

//create a tubefeet
btRigidBody* initTubefeet(btScalar* scale, const btVector3 position)
{
    btCollisionShape* sBodyShape = new btCapsuleShape(scale[0], scale[1]);
    collisionShapes.push_back(sBodyShape);
    
    btScalar mass1(M_TF);
    bool isDynamic = (mass1 != 0.f);
    
    btVector3 localInertia1(0, 0, 0);
    if (isDynamic)
        groundShape->calculateLocalInertia(mass1, localInertia1);
    
    
    btTransform sBodyTransform;
    sBodyTransform.setIdentity();
    sBodyTransform.setOrigin(position);
    btDefaultMotionState* myMotionState1 = new btDefaultMotionState(sBodyTransform);
    
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass1, myMotionState1, sBodyShape, localInertia1);
    btRigidBody* body = new btRigidBody(rbInfo);
    
    body->setActivationState(DISABLE_DEACTIVATION);
    
    return body;
}

// create starfish
void CreateStarfish()
{
    vector<btRigidBody* > bodies_body;
    vector<btRigidBody* > bodies_tf;
    vector<btRigidBody* > bodies_amp;
    vector<btTypedConstraint* > constraints;
    
/*** create body ***/
    btRigidBody* body_body = initBody(btVector3(RADIUS*2, LENGTH, RADIUS*2), btVector3(0, INIT_POS_Y-RADIUS*2, 0));
    BODY_object[0] = body_body;
    bodies_body.push_back(body_body);
    /*for (int i = 0; i < 5; i++) {
        btRigidBody* body_arm = initArm(btVector3(RADIUS*6, LENGTH, RADIUS*2), RotateY(btVector3(RADIUS*10, INIT_POS_Y-RADIUS*2, 0), M_PI*2*i/5), btQuaternion(btVector3(0, 1, 0), M_PI*2*i/5));
        BODY_object[i+1] = body_arm;
        bodies_body.push_back(body_arm);
    }*/
    
/*** create tubefeet & amp ***/
    std::random_device rnd;
    std::mt19937 mt(rnd());
    std::uniform_int_distribution<> rand100(0, 99);
    
    btScalar scale[] = {btScalar(RADIUS), btScalar(LENGTH)};
    btVector3 pos_tf, pos_amp;
    int col, row;
    for (int i = 0; i < 10; i++) {
        col = i % 2;
        row = i / 2;
        pos_tf = btVector3(RADIUS*2 + row * RADIUS * 4, INIT_POS_Y-(RADIUS*2+LENGTH/2), pow(-1, col) * RADIUS * 2);
        //pos_tf = btVector3(RADIUS*2 + row * RADIUS * 4 - (RADIUS*2+LENGTH/2)*sin(ANGLE_DETACH), INIT_POS_Y-(RADIUS*2+LENGTH/2)*cos(ANGLE_DETACH), pow(-1, col) * RADIUS * 2);
        pos_amp = btVector3(RADIUS*2 + row * RADIUS * 4, INIT_POS_Y, pow(-1, col) * RADIUS * 2);
        for (int j = 1; j <= 5; j++) {
            //tf - amp (object)
            btRigidBody* body_amp = initAmp(btScalar(RADIUS), pos_amp);
            btRigidBody* body_tf = initTubefeet(scale, pos_tf);
            int index = 100 + 10 * i + j;
            body_tf->setUserIndex(index);
            TF_object[index] = body_tf;
            TF_object_amp[index] = body_amp;
            TF_contact[index] = false;
            TF_axis_direction[index] = btVector3(1, 0, 0);
            TF_axis_angle[index] = M_PI/2;
            TF_contact_times[index] = 0;
            bodies_tf.push_back(body_tf);
            bodies_amp.push_back(body_amp);
            //tf - amp (constraint)
            btUniversalConstraint* univ = new btUniversalConstraint(*body_amp, *body_tf, pos_amp, btVector3(0, 1, 0), btVector3(0, 0, 1));//global
            univ->setLowerLimit(-ANGLE, -M_PI);
            univ->setUpperLimit(ANGLE, M_PI);
            TF_constraint_amp[index] = univ;
            constraints.push_back(univ);
            //tf - amp (motor)
            btRotationalLimitMotor* motor1 = univ->getRotationalLimitMotor(1);//wheel
            btRotationalLimitMotor* motor2 = univ->getRotationalLimitMotor(2);//handle
            motor1->m_enableMotor = true;
            motor1->m_targetVelocity = 0;
            motor2->m_enableMotor = true;
            motor2->m_targetVelocity = 0;
            motor_tZ[index] = motor1;
            motor_tY[index] = motor2;
            ResumeTime_tf[index] = 2*SECOND*( rand100(mt)/100.0 );
            //for next
            pos_tf = RotateY(pos_tf, M_PI*2/5);
            pos_amp = RotateY(pos_amp, M_PI*2/5);
        }
    }

    
    
    
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

//motion of tubefeet
void ControllTubeFeet()
{
    
    btScalar velocity_all_x = 0;
    btScalar velocity_all_z = 0;
    //calculate interaction of tf
    for (auto itr = TF_contact.begin(); itr != TF_contact.end(); ++itr) {
        
        int index = itr->first;
        btRigidBody* body = TF_object[index];
        
        if (body && body->getMotionState() && TF_contact[index])
        {
            btScalar velocity_x = body->getLinearVelocity()[0];
            btScalar velocity_z = body->getLinearVelocity()[1];
            
            if (velocity_x < velocity_all_x) {
                velocity_all_x = velocity_x;
                velocity_all_z = velocity_z;
            }
        }
    }
    
    //interacting of tf with body (X direction)/*********************X、Z軸両方向に関して影響与えねばならん************************/
    for (auto itr = BODY_object.begin(); itr != BODY_object.end(); ++itr) {
        btRigidBody* body = itr->second;
        
        if (body && body->getMotionState())
        {
            btVector3 pos = body->getCenterOfMassPosition();
            
            btTransform tran;
            tran.setIdentity();
            tran.setOrigin(btVector3(pos[0]+velocity_all_x*1.5/FPS, pos[1], pos[2]+velocity_all_z*1.5/FPS));
            
            body->setCenterOfMassTransform(tran);
        }
    }
    
    //interacting of tf with amp (X direction)/*********************X、Z軸両方向に関して影響与えねばならん************************/
    for (auto itr = TF_object_amp.begin(); itr != TF_object_amp.end(); ++itr) {
        
        int index = itr->first;
        btRigidBody* body = itr->second;
    
        if (body && body->getMotionState() && !TF_contact[index])
        {
            btVector3 pos = body->getCenterOfMassPosition();
        
            btTransform tran;
            tran.setIdentity();
            tran.setOrigin(btVector3(pos[0]+velocity_all_x/FPS, INIT_POS_Y - (LENGTH/2 + 4) + (LENGTH/2 + 4)*sin(2*M_PI*(time_step%(SECOND*2))/(SECOND*2) + M_PI_2), pos[2]+velocity_all_z/FPS));
        
            body->setCenterOfMassTransform(tran);
        }
    }
    
    //motion of tubefeet - amp (motor)
    for (auto itr = motor_tZ.begin(); itr != motor_tZ.end(); ++itr) {
        
        int index = itr->first;
        btRotationalLimitMotor* motor = itr->second;
        
        motor->m_maxMotorForce = 1000000000000000;
        motor_tY[index]->m_maxMotorForce = 1000000000000000;
        
        //handle motor/**********************どのくらいずつ振るか要調整**************************/
        btScalar angle_now = motor_tY[index]->m_currentPosition + TF_axis_angle[index];//relative angle with the start position
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
        motor_tY[index]->m_targetVelocity = (angle_target - angle_now)/2;//target angular velocity (rad/sec)
        
        //wheel motor
        if (!TF_contact[index] && ResumeTime_tf[index] < time_step)
        {
            double target_velocity = motor->m_targetVelocity;
            if (target_velocity == 0) {
                motor->m_targetVelocity = -ANGLE_VELOCITY_TF;
            }
            
            btScalar angle = motor->m_currentPosition;
            if (TF_contact_times[index]==0 && -(ANGLE - 0.1) >= angle && target_velocity <= 0) {
                motor->m_targetVelocity *= -1.0;
            } else if (0.1 >= angle && target_velocity <= 0) {
                motor->m_targetVelocity *= -1.0;
            }
            
            if (TF_contact_times[index]==0 && angle >= ANGLE-0.1  && target_velocity >= 0) {
                motor->m_targetVelocity *= -1.0;
            } else if (2*ANGLE - 0.1 <= angle && target_velocity >= 0) {
                motor->m_targetVelocity *= -1.0;
            }
        }
        else
        {
            motor->m_targetVelocity = 0;
        }
    }
    
    //motion of tubefeet - ground (wheel motor)
    for (auto itr = motor_to_groundZ.begin(); itr != motor_to_groundZ.end(); ++itr) {
        
        int index = itr->first;
        btRotationalLimitMotor* motor = itr->second;
        
        motor->m_maxMotorForce = 1000000000000000;
        motor_to_groundY[index]->m_maxMotorForce = 1000000000000000;
        
        motor->m_targetVelocity = -ANGLE_VELOCITY_GROUND;
    }
    

}

//action when tubefeet attach ground
void ContactAction()
{
    vector<int > contacts;
    
    int numManifolds = dynamicsWorld->getDispatcher()->getNumManifolds();
    for (int i = 0; i < numManifolds; i++)
    {
        btPersistentManifold* contactManifold =  dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
        const btCollisionObject* obA = contactManifold->getBody0();
        const btCollisionObject* obB = contactManifold->getBody1();
        btRigidBody* bodyA = btRigidBody::upcast(const_cast<btCollisionObject *>(obA));
        btRigidBody* bodyB = btRigidBody::upcast(const_cast<btCollisionObject *>(obB));

        int obID = obA->getUserIndex();
        
        //when a tubefeet attach ground
        if (obID==1 || obID==2 || obID==3 || obID==4) {
            
            int numContacts = contactManifold->getNumContacts();

            if (numContacts >= 1) {
                int k = -1;
                for (int j = 0; j < numContacts; j++) {
                    btManifoldPoint& pt = contactManifold->getContactPoint(j);

                    if (pt.getDistance() < 0.5f)
                    {
                        k = j;
                        break;
                    }
                }
                if (k == -1) {
                    break;
                }
                
                btManifoldPoint& pt = contactManifold->getContactPoint(k);
                const btVector3& ptB = pt.getPositionWorldOnB();

                int index = obB->getUserIndex();
                
                btScalar angle;
                if (TF_contact[index]) {
                    angle = motor_to_groundZ[index]->m_currentPosition;
                } else {
                    angle = motor_tZ[index]->m_currentPosition;
                }
                
                //when a tubefeet attempt to attach
                if ((TF_contact_times[index]==0 && angle>ANGLE_ATTACH) || (!TF_contact[index] && angle>ANGLE_ATTACH-ANGLE_DETACH)){
                    TF_axis_angle[index] += motor_tY[index]->m_currentPosition;
                    
                    //remove tubefeet - amp (object & constraint & motor)
                    dynamicsWorld->removeRigidBody(TF_object_amp[index]);
                    dynamicsWorld->removeConstraint(TF_constraint_amp[index]);
                    motor_tY.erase(index);
                    motor_tZ.erase(index);
                    TF_contact[index] = true;
                    TF_contact_times[index]++;
                    
                    //create tubefeet - ground (constraint)
                    btUniversalConstraint* univ = new btUniversalConstraint(*bodyA, *bodyB, btVector3(ptB[0],ptB[1]+RADIUS ,ptB[2] ), btVector3(0, 1, 0),btVector3(cos(TF_axis_angle[index]), 0, sin(TF_axis_angle[index])));//global
                    univ->setLowerLimit(-ANGLE, -M_PI);
                    univ->setUpperLimit(ANGLE, M_PI);
                    TF_constraint_ground[index] = univ;
                    dynamicsWorld->addConstraint(univ);
                    
                    //create tubefeet - ground (motor)
                    btRotationalLimitMotor* motor1 = univ->getRotationalLimitMotor(1);//wheel
                    btRotationalLimitMotor* motor2 = univ->getRotationalLimitMotor(2);//handle
                    motor1->m_enableMotor = true;
                    motor2->m_enableMotor = true;
                    motor_to_groundZ[index] = motor1;
                    motor_to_groundY[index] = motor2;
                    DeleteTime_tf[index] = time_step + int(double(ANGLE_ATTACH - ANGLE_DETACH)/double(ANGLE_VELOCITY_GROUND) * double(FPS) * 2.0);
                    
                    
                }
                else if (TF_contact[index])
                {
                    //delete the tubefeet attaching too long time & after that, create new one
                    if (time_step > DeleteTime_tf[index]) {
                         //remove tubefeet - ground (constraint & motor)
                         dynamicsWorld->removeConstraint(TF_constraint_ground[index]);
                         TF_constraint_ground.erase(index);
                         motor_to_groundY.erase(index);
                         motor_to_groundZ.erase(index);
                         TF_contact[index] = false;
                         TF_contact_times[index] = 0;
                         
                         //remove tubefeet
                         dynamicsWorld->removeRigidBody(TF_object[index]);
                         TF_object.erase(index);
                         
                         //create new one
                         btRigidBody* body_centor = BODY_object[0];
                         if (body_centor && body_centor->getMotionState())
                         {
                             btVector3 pos = body_centor->getCenterOfMassPosition();
                             std::random_device rnd;
                             std::mt19937 mt(rnd());
                             std::uniform_int_distribution<> rand100(0, 99);
                             btScalar scale[] = {btScalar(RADIUS), btScalar(LENGTH)};
                             int n = index % 10;//j
                             int m = (index-100)/10;//i
                             int col = m % 2;
                             int row = m / 2;
                             int h = INIT_POS_Y-RADIUS*2-LENGTH/2;
                             int from_x = RADIUS*2;
                             btVector3 pos_tf = btVector3(from_x + row * RADIUS * 4, h, pow(-1, col) * RADIUS * 2);
                             btVector3 pos_amp = btVector3(from_x + row * RADIUS * 4 , INIT_POS_Y, pow(-1, col) * RADIUS * 2);
                             pos_tf = RotateY(pos_tf, M_PI*(n-1)*2/5);
                             pos_amp = RotateY(pos_amp, M_PI*(n-1)*2/5);
                             pos_tf[0] += pos[0];
                             pos_tf[2] += pos[2];
                             pos_amp[0] += pos[0];
                             pos_amp[2] += pos[2];
                             //tf - amp (object)
                             btRigidBody* body_amp = initAmp(btScalar(RADIUS), pos_amp);
                             btRigidBody* body_tf = initTubefeet(scale, pos_tf);
                             body_tf->setUserIndex(index);
                             TF_object[index] = body_tf;
                             TF_object_amp[index] = body_amp;
                             TF_contact[index] = false;
                             dynamicsWorld->addRigidBody(body_tf, RX_COL_TF, RX_COL_GROUND);
                             dynamicsWorld->addRigidBody(body_amp, RX_COL_AMP, RX_COL_GROUND);
                             //tf - amp (constraint)
                             btUniversalConstraint* univ = new btUniversalConstraint(*body_amp, *body_tf, pos_amp, btVector3(0, 1, 0), btVector3(0, 0, 1));//global
                             univ->setLowerLimit(-ANGLE, -M_PI);
                             univ->setUpperLimit(ANGLE, M_PI);
                             TF_constraint_amp[index] = univ;
                             dynamicsWorld->addConstraint(univ);
                             //tf - amp (motor)
                             btRotationalLimitMotor* motor1 = univ->getRotationalLimitMotor(1);//wheel
                             btRotationalLimitMotor* motor2 = univ->getRotationalLimitMotor(2);//handle
                             motor1->m_enableMotor = true;
                             motor1->m_targetVelocity = 0;
                             motor2->m_enableMotor = true;
                             motor2->m_targetVelocity = 0;
                             motor_tZ[index] = motor1;
                             motor_tY[index] = motor2;
                             ResumeTime_tf[index] = 2*SECOND*( rand100(mt)/100.0 );
                         }
                    }
                    //when a tubefeet attempt to dettach
                     if (angle < ANGLE_DETACH - ANGLE_ATTACH)
                    {
                        TF_axis_angle[index] += motor_to_groundY[index]->m_currentPosition;
                        
                        //remove tubefeet - ground (constraint & motor)
                        dynamicsWorld->removeConstraint(TF_constraint_ground[index]);
                        motor_to_groundY.erase(index);
                        motor_to_groundZ.erase(index);
                        TF_contact[index] = false;
                        
                        //create amp (object)
                        btVector3 pos_tf = bodyB->getCenterOfMassPosition();
                        btVector3 pos_amp = btVector3(pos_tf[0]+(LENGTH/2+RADIUS*2)*sin(angle+ANGLE_ATTACH)*sin(TF_axis_angle[index]), pos_tf[1]+(LENGTH/2+RADIUS*2)*cos(angle+ANGLE_ATTACH), pos_tf[2]-(LENGTH/2+RADIUS*2)*sin(angle+ANGLE_ATTACH)*cos(TF_axis_angle[index]));
                        btRigidBody* body_amp = initAmp(RADIUS, pos_amp);
                        TF_object_amp[index] = body_amp;
                        dynamicsWorld->addRigidBody(body_amp);
                                
                        //create tubefeet - amp (constraint)
                        btUniversalConstraint* univ = new btUniversalConstraint(*body_amp, *TF_object[index], pos_amp, btVector3(pos_amp[0]-pos_tf[0], pos_amp[1]-pos_tf[1], pos_amp[2]-pos_tf[2]), btVector3(cos(TF_axis_angle[index]), 0, sin(TF_axis_angle[index])));//global
                        univ->setLowerLimit(0, -M_PI);
                        univ->setUpperLimit(2*ANGLE, M_PI);
                        TF_constraint_amp[index] = univ;
                        dynamicsWorld->addConstraint(univ);
                        
                        //create tubefeet - amp (motor)
                        btRotationalLimitMotor* motor1 = univ->getRotationalLimitMotor(1);//wheel
                        btRotationalLimitMotor* motor2 = univ->getRotationalLimitMotor(2);//handle
                        motor1->m_enableMotor = true;
                        motor1->m_targetVelocity = ANGLE_VELOCITY_TF;
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

//------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------

void init(void)
{
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
	gluLookAt(100,300,100, 0.0, 0, 0.0, 0.0, 1.0, 0.0);
}

void idle(void)
{
    ContactAction();
	glutPostRedisplay();
    ControllTubeFeet();
}

void CleanupBullet()
{
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
    
    delete dynamicsWorld;
    
    delete solver;
    
    delete overlappingPairCache;
    
    delete dispatcher;
    
    delete collisionConfiguration;
    
    collisionShapes.clear();
}

void InitBullet()
{
    collisionConfiguration = new btDefaultCollisionConfiguration();
    
    dispatcher = new btCollisionDispatcher(collisionConfiguration);
    
    overlappingPairCache = new btDbvtBroadphase();
    
    solver = new btSequentialImpulseConstraintSolver;
    
    dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
    
    dynamicsWorld->setGravity(btVector3(0, -10, 0));
}



int main(int argc, char** argv)
{
    InitBullet();

	CreateGround();

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

	CleanupBullet();
}

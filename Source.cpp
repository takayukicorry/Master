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
vector<btRotationalLimitMotor* > motor_tY;
vector<btRotationalLimitMotor* > motor_tZ;
vector<btRotationalLimitMotor* > motor_aY;
vector<btRotationalLimitMotor* > motor_aZ;
vector<btRotationalLimitMotor* > motor_to_groundY;
vector<btRotationalLimitMotor* > motor_to_groundZ;

GLfloat light0pos[] = { 300.0, 300.0, 300.0, 1.0 };
GLfloat light1pos[] = { -300.0, 300.0, 300.0, 1.0 };
map<int, bool> TF_contact;

int time_step = 0;

enum CollisionGroup{
    RX_COL_NOTHING = 0, // 0000
    RX_COL_GROUND = 1,  // 0001
    RX_COL_BODY = 2,  // 0010
    RX_COL_TF = 4   // 0100
};

//----------------------------------------------------
// ���������̒�`
//----------------------------------------------------
struct MaterialStruct {
	GLfloat ambient[4];
	GLfloat diffuse[4];
	GLfloat specular[4];
	GLfloat shininess;
};
//jade(�Ő�)
MaterialStruct ms_jade = {
	{ 0.135, 0.2225, 0.1575, 1.0 },
	{ 0.54, 0.89, 0.63, 1.0 },
	{ 0.316228, 0.316228, 0.316228, 1.0 },
	12.8 };
//ruby(���r�[)
MaterialStruct ms_ruby = {
	{ 0.1745, 0.01175, 0.01175, 1.0 },
	{ 0.61424, 0.04136, 0.04136, 1.0 },
	{ 0.727811, 0.626959, 0.626959, 1.0 },
	76.8 };
//----------------------------------------------------
// �F�̒�`�̒�`
//----------------------------------------------------
GLfloat red[] = { 0.8, 0.2, 0.2, 1.0 }; //�ԐF
GLfloat green[] = { 0.2, 0.8, 0.2, 1.0 };//�ΐF
GLfloat blue[] = { 0.2, 0.2, 0.8, 1.0 };//�F
GLfloat yellow[] = { 0.8, 0.8, 0.2, 1.0 };//���F
GLfloat white[] = { 1.0, 1.0, 1.0, 1.0 };//���F
GLfloat shininess = 30.0;//����̋���
//-----------------------------------------
//�v�Z����֐�
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
    
    // �ÓI�ȍ��̂����܂�
    bool isDynamic = (mass != 0.f);
    
    btVector3 localInertia(0, 0, 0);
    if (isDynamic)
        groundShape->calculateLocalInertia(mass, localInertia);
        
    // �f�t�H���g�̃��[�V�����X�e�[�g���쐬
    btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
    
    return rbInfo;
}
//---------------------------------------------

// �O�����h�̐���
void CreateGround()
{
	// �V�F�C�v�̐���
	btVector3 scale = btVector3(btScalar(500.), btScalar(50.), btScalar(500.));
	groundShape = new btBoxShape(scale);
	collisionShapes.push_back(groundShape);
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, -56, 0));

	btScalar mass(0.);

	// �ÓI�ȍ��̂����܂�
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		groundShape->calculateLocalInertia(mass, localInertia);

	// �f�t�H���g�̃��[�V�����X�e�[�g���쐬
	btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);

    body->setActivationState(DISABLE_DEACTIVATION);
    body->setUserIndex(1);
    
	// �쐬�������̂����[���h�֓o�^
    dynamicsWorld->addRigidBody(body, RX_COL_GROUND, RX_COL_BODY | RX_COL_TF);
    ///dynamicsWorld->addRigidBody(body);
}

// ���̐���
btRigidBody* CreateBall(btScalar scale, const btVector3 position)
{
	btCollisionShape* colShape = new btSphereShape(scale);
	collisionShapes.push_back(colShape);

	// �g�����X�t�H�[�����쐬
	btTransform startTransform;
	startTransform.setIdentity();

	btScalar mass(0.f);

	// ���ʂ�1�Ƃ����̂Ń_�C�i�~�b�N
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		colShape->calculateLocalInertia(mass, localInertia);

	startTransform.setOrigin(position);

	// ���[�V�����X�e�[�g���쐬
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);
    
    body->setActivationState(DISABLE_DEACTIVATION);

    return  body;
}

//�q�g�f�̓��̐���
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
    
    // �f�t�H���g�̃��[�V�����X�e�[�g���쐬
    btDefaultMotionState* myMotionState = new btDefaultMotionState(sBodyTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, sBodyShape, localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);
    
    return body;
}

//�q�g�f�̘r����
btRigidBody* initArm(const btVector3 scale, const btVector3 position, const btQuaternion rot)
{
	btCollisionShape* sBodyShape = new btBoxShape(scale);
	collisionShapes.push_back(sBodyShape);

	btScalar mass1(M_ARM);
	bool isDynamic = (mass1 != 0.f);

	btVector3 localInertia1(0, 0, 0);
	if (isDynamic)
		groundShape->calculateLocalInertia(mass1, localInertia1);

	// �f�t�H���g�̃��[�V�����X�e�[�g���쐬

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

//�q�g�f�̊Ǒ�����
btRigidBody* initTubefeet(btScalar* scale, const btVector3 position)
{
    btCollisionShape* sBodyShape = new btCapsuleShape(scale[0], scale[1]);
    collisionShapes.push_back(sBodyShape);
    
    btScalar mass1(M_TF);
    bool isDynamic = (mass1 != 0.f);
    
    btVector3 localInertia1(0, 0, 0);
    if (isDynamic)
        groundShape->calculateLocalInertia(mass1, localInertia1);
    
    // �f�t�H���g�̃��[�V�����X�e�[�g���쐬
    
    btTransform sBodyTransform;
    sBodyTransform.setIdentity();
    sBodyTransform.setOrigin(position);
    btDefaultMotionState* myMotionState1 = new btDefaultMotionState(sBodyTransform);
    
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass1, myMotionState1, sBodyShape, localInertia1);
    btRigidBody* body = new btRigidBody(rbInfo);
    
    body->setActivationState(DISABLE_DEACTIVATION);
    
    return body;
}

// �q�g�f�̐���
void CreateStarfish()
{
    vector<btRigidBody* > bodies_body;
    vector<btRigidBody* > bodies_tf;
    vector<btTypedConstraint* > constraints;
    
#if TUBEFEET_SIMULATION_MODE //�Ǒ���̃V�~�����[�V����
/***����***/
    
/***�Ǒ�***/
    btScalar scale[] = {btScalar(RADIUS), btScalar(LENGTH)};
    btRigidBody* body_amp = CreateBall(btScalar(RADIUS), btVector3(0, btScalar(INIT_POS_Y), 0));
    btRigidBody* body_tf = initTubefeet(scale, btVector3(0, INIT_POS_Y-RADIUS*2-LENGTH/2, 0));
    body_tf->setUserIndex(100);
    TF_contact[100] = false;
    bodies_tf.push_back(body_amp);
    bodies_tf.push_back(body_tf);
    //�S��
    btUniversalConstraint* univ = new btUniversalConstraint(*bodies_tf[0], *bodies_tf[1], btVector3(0, 24, 0), btVector3(0, 1, 0), btVector3(0, 0, 1));//�S���O���[�o��
    constraints.push_back(univ);
    //���[�^�[
    btRotationalLimitMotor* motor1 = univ->getRotationalLimitMotor(1);//�ԗ�
    btRotationalLimitMotor* motor2 = univ->getRotationalLimitMotor(2);//�X�e�A�����O
    motor1->m_enableMotor = true;
    motor2->m_enableMotor = true;
    motor_tZ.push_back(motor1);
    motor_tY.push_back(motor2);
    
    
#else
    /***������***/
    btVector3 scale_b(btScalar(25.), btScalar(5.), btScalar(25.));
    btVector3 position_b(0, 10, 0);
    bodies_body.push_back(initBody(scale_b, position_b));
	
/***���r***/
	btVector3 scale_a(btScalar(25.), btScalar(5.), btScalar(10.));
    btScalar dist = scale_a[0]+scale_b[0]+scale_b[1];

	bodies_body.push_back(initArm(scale_a, btVector3(dist, position_b[1], 0), btQuaternion(0, 0, 0, 1)));
	bodies_body.push_back(initArm(scale_a, btVector3(0, position_b[1], -dist), btQuaternion(0, 1/sqrt(2), 0, 1/sqrt(2))));
	bodies_body.push_back(initArm(scale_a, btVector3(-dist, position_b[1], 0), btQuaternion(0, 1, 0, 0)));
	bodies_body.push_back(initArm(scale_a, btVector3(0, position_b[1], dist), btQuaternion(0, -1/sqrt(2), 0, 1/sqrt(2))));
    
    //�S��
    btVector3 pivotInBody(scale_b[0], 0, 0);
    btVector3 axisInBody(0, 0, 1);
    btVector3 pivotInArm(-scale_a[0]-scale_b[1], 0, 0);
    btVector3 axisInArm = axisInBody;
    btHingeConstraint* hinge = new btHingeConstraint(*bodies_body[0], *bodies_body[1], pivotInBody, pivotInArm, axisInBody, axisInArm);//�S�����[�J��
    constraints.push_back(hinge);
    
    pivotInBody = btVector3(0, 0, -scale_b[0]);
    axisInBody = btVector3(1, 0, 0);
    hinge = new btHingeConstraint(*bodies_body[0], *bodies_body[2], pivotInBody, pivotInArm, axisInBody, axisInArm);
    constraints.push_back(hinge);
    
    pivotInBody = btVector3(-scale_b[0], 0, 0);
    axisInBody = btVector3(0, 0, -1);
    hinge = new btHingeConstraint(*bodies_body[0], *bodies_body[3], pivotInBody, pivotInArm, axisInBody, axisInArm);
    constraints.push_back(hinge);
    
    pivotInBody = btVector3(0, 0, scale_b[0]);
    axisInBody = btVector3(-1, 0, 0);
    hinge = new btHingeConstraint(*bodies_body[0], *bodies_body[4], pivotInBody, pivotInArm, axisInBody, axisInArm);
    constraints.push_back(hinge);
    
/***���Ǒ�***/
    btScalar scale_t[] = { btScalar(2.), btScalar(6.) };
    btVector3 position_t;
    int col, row;
    int h = position_b[1] - scale_b[1] - scale_t[0] - scale_t[1]/2;
    int from_x = dist - scale_a[0] + scale_t[0]*2;
    
    for (int i = 0; i < (scale_a[0]*2-scale_t[0]*4)*2/(scale_t[0]*3)+2; i++) {
        col = i % 2;
        row = i / 2;
        position_t = btVector3(from_x + row * scale_t[0] * 3, h, pow(-1, col) * scale_t[0] * 2);
        for (int j = 1; j <= 4; j++) {
            btRigidBody* body_t = initTubefeet(scale_t, position_t);
            bodies_tf.push_back(body_t);
            
            //�S��
            position_t[1] += scale_t[0] + scale_t[1]/2;
            btUniversalConstraint* univ = new btUniversalConstraint(*bodies_body[j], *body_t, position_t, btVector3(0, 1, 0), btVector3(sin(M_PI_2*(j-1)), 0, cos(M_PI_2*(j-1))));//�S���O���[�o��
            
            univ->setLowerLimit(-ANGLE, -ANGLE);
            univ->setUpperLimit(ANGLE, ANGLE);
            
            constraints.push_back(univ);

            //���[�^�[
            btRotationalLimitMotor* motor1 = univ->getRotationalLimitMotor(1);//�ԗ�
            btRotationalLimitMotor* motor2 = univ->getRotationalLimitMotor(2);//�X�e�A�����O
            motor1->m_enableMotor = true;
            motor2->m_enableMotor = true;
            
            motor_tZ.push_back(motor1);
            motor_tY.push_back(motor2);

            position_t[1] -= scale_t[0] + scale_t[1]/2;
            position_t = RotateY(position_t, M_PI_2);
            
        }
    }
#endif
    
////�o�^////
    
    for (int i = 0; i < bodies_body.size(); i++) {
        dynamicsWorld->addRigidBody(bodies_body[i], RX_COL_BODY, RX_COL_GROUND);
        ///dynamicsWorld->addRigidBody(bodies_body[i]);
    }
    
    for (int i = 0; i < bodies_tf.size(); i++) {
        dynamicsWorld->addRigidBody(bodies_tf[i], RX_COL_TF, RX_COL_GROUND);
        ///dynamicsWorld->addRigidBody(bodies_tf[i]);
    }

    for (int i = 0; i < constraints.size(); i++) {
        dynamicsWorld->addConstraint(constraints[i]);
    }

}

void ControllTubeFeet()
{
#if TUBEFEET_SIMULATION_MODE
    //�r�X�̏㉺
    btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[1];
    btRigidBody* body = btRigidBody::upcast(obj);
    int index = dynamicsWorld->getCollisionObjectArray()[2]->getUserIndex();
    
    if (body && body->getMotionState() && !TF_contact[index])
    {
        btVector3 pos = body->getCenterOfMassPosition();
        
        btTransform tran;
        tran.setIdentity();
        ////tran.setOrigin(btVector3(pos[0], INIT_POS_Y - (LENGTH/2 + 4)/2 + (LENGTH/2 + 4)/2*sin(2*M_PI*(time_step%(SECOND*2))/(SECOND*2) + M_PI_2), pos[2]));
        tran.setOrigin(btVector3(pos[0], INIT_POS_Y - (LENGTH/2 + 4) + (LENGTH/2 + 4)*sin(2*M_PI*(time_step%(SECOND*2))/(SECOND*2) + M_PI_2), pos[2]));
        
        body->setCenterOfMassTransform(tran);
    }
    
    //�Ǒ��̃��[�^�[
    for (int i = 0; i < motor_tZ.size(); i++) {
        motor_tZ[i]->m_maxMotorForce = 100000000;
        motor_tY[i]->m_maxMotorForce = 100000000;
        
        if (!TF_contact[index])
        {
            if ((time_step/SECOND+1) / 2 % 2 == 0) {
                motor_tZ[i]->m_targetVelocity = ANGLE*60/SECOND;
            }else{
                motor_tZ[i]->m_targetVelocity = -ANGLE*60/SECOND;
            }
            
        }
        else
        {
            motor_tZ[i]->m_targetVelocity = 0;
            
        }
    }
    
    //�n�ʂƂ̃��[�^�[
    for (int i = 0; i < motor_to_groundZ.size(); i++) {
        motor_to_groundZ[i]->m_maxMotorForce = 100000000;
        motor_to_groundY[i]->m_maxMotorForce = 100000000;
        
        motor_to_groundZ[i]->m_targetVelocity = -ANGLE;
    }
    
#else
    
    for (int i = 0; i < motor_tZ.size(); i++) {
        motor_tZ[i]->m_maxMotorForce = 100000000;
        motor_tY[i]->m_maxMotorForce = 100000000;
        
        if ((time_step/60+1) / 2 % 2 == 0) {
            motor_tZ[i]->m_targetVelocity = ANGLE;
        }else{
            motor_tZ[i]->m_targetVelocity = -ANGLE;
        }
        
    }
#endif
}

void ContactAction()
{
    vector<int > contacts;
    
    int numManifolds = dynamicsWorld->getDispatcher()->getNumManifolds();
    for (int i = 0; i < numManifolds; i++)
    {
        //�Ǒ��Փˏ�ԍX�V
        btPersistentManifold* contactManifold =  dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
        const btCollisionObject* obA = contactManifold->getBody0();
        const btCollisionObject* obB = contactManifold->getBody1();
        btRigidBody* bodyA = btRigidBody::upcast(const_cast<btCollisionObject *>(obA));
        btRigidBody* bodyB = btRigidBody::upcast(const_cast<btCollisionObject *>(obB));
        int index = obB->getUserIndex();

        if (obA->getUserIndex()==1) {
            
            //�Փˏ��擾
            int numContacts = contactManifold->getNumContacts();
            for (int j = 0; j < numContacts; j++)
            {
                btManifoldPoint& pt = contactManifold->getContactPoint(j);
                if (pt.getDistance() < 0.f)
                {
                    const btVector3& ptA = pt.getPositionWorldOnA();
                    const btVector3& ptB = pt.getPositionWorldOnB();
                    const btVector3& normalOnB = pt.m_normalWorldOnB;
                    
                    if (!TF_contact[index]) {
                        btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[1];
                        btRigidBody* body = btRigidBody::upcast(obj);
                        dynamicsWorld->removeRigidBody(body);
                        
                        TF_contact[index] = true;
                        
                        
                        //�Փ˓_�ɍS���쐬
                        btUniversalConstraint* univ = new btUniversalConstraint(*bodyA, *bodyB, btVector3(ptB[0],ptB[1]+RADIUS ,ptB[2] ), btVector3(0, 1, 0), btVector3(0, 0, 1));//�S���O���[�o��
                        univ->setLowerLimit(-ANGLE, -ANGLE);
                        univ->setUpperLimit(ANGLE, ANGLE);
                        dynamicsWorld->addConstraint(univ);
                    
                        //���[�^�[
                        btRotationalLimitMotor* motor1 = univ->getRotationalLimitMotor(1);//�ԗ�
                        btRotationalLimitMotor* motor2 = univ->getRotationalLimitMotor(2);//�X�e�A�����O
                        motor1->m_enableMotor = true;
                        motor2->m_enableMotor = true;
                        motor_to_groundZ.push_back(motor1);
                        motor_to_groundY.push_back(motor2);
                    }
                }
            }
        }
    }
}

void Render()
{
    time_step++;
    dynamicsWorld->stepSimulation(1.f / 60.f, 10);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//glLightfv(GL_LIGHT0, GL_POSITION, light0pos);

	glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    //glDisable(GL_LIGHT1);

	/* ���f���r���[�ϊ��s��̕ۑ� */
	glPushMatrix();

	// ���̂̍��W���v�����g
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
            //�n��
            if (j == 0)
			{
				glScaled(2 * halfExtent[0], 2 * halfExtent[1], 2 * halfExtent[2]);
				glMaterialfv(GL_FRONT, GL_AMBIENT, ms_jade.ambient);
				glMaterialfv(GL_FRONT, GL_DIFFUSE, ms_jade.diffuse);
				glMaterialfv(GL_FRONT, GL_SPECULAR, ms_jade.specular);
				glMaterialfv(GL_FRONT, GL_SHININESS, &ms_jade.shininess);
				glutSolidCube(1);
			}
            //���́@�r
			else if (shape == BOX_SHAPE_PROXYTYPE)
			{
				glScaled(2 * halfExtent[0], 2 * halfExtent[1], 2 * halfExtent[2]);
				glMaterialfv(GL_FRONT, GL_AMBIENT, ms_ruby.ambient);
				glMaterialfv(GL_FRONT, GL_DIFFUSE, ms_ruby.diffuse);
				glMaterialfv(GL_FRONT, GL_SPECULAR, ms_ruby.specular);
				glMaterialfv(GL_FRONT, GL_SHININESS, &ms_ruby.shininess);
				glutSolidCube(1);
			}
			else if (shape == SPHERE_SHAPE_PROXYTYPE)
			{
                glScaled(halfExtent[1], halfExtent[1], halfExtent[1]);
				glMaterialfv(GL_FRONT, GL_AMBIENT, ms_ruby.ambient);
				glMaterialfv(GL_FRONT, GL_DIFFUSE, ms_ruby.diffuse);
				glMaterialfv(GL_FRONT, GL_SPECULAR, ms_ruby.specular);
				glMaterialfv(GL_FRONT, GL_SHININESS, &ms_ruby.shininess);
				glutSolidSphere(1, 100, 100);
			}
            //�Ǒ�
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

	/* ���f���r���[�ϊ��s��̕��A */
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

	glMatrixMode(GL_PROJECTION);//�s�񃂁[�h�̐ݒ�iGL_PROJECTION : �����ϊ��s��̐ݒ�AGL_MODELVIEW�F���f���r���[�ϊ��s��j
	glLoadIdentity();//�s��̏�����
	gluPerspective(30.0, (double)640 / (double)480, 0.1, 1000);
	gluLookAt(0, 0, 300, 0.0, 0, 0.0, 0.0, 1.0, 0.0);
}

void idle(void)
{
    ContactAction();
	glutPostRedisplay();
    ControllTubeFeet();
}

// �I������
void CleanupBullet()
{
    // ���̂��폜
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
    
    // �Փ˃V�F�C�v���폜
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
    
    // ���[���h���폜
    delete dynamicsWorld;
    
    // �\���o�[���폜
    delete solver;
    
    // �Փˌ��m�̃u���[�h�t�F�C�Y���폜
    delete overlappingPairCache;
    
    // �f�B�X�p�b�`�����폜
    delete dispatcher;
    
    delete collisionConfiguration;
    
    // �I�v�V�����A�Ȃ��Ă��ǂ�
    collisionShapes.clear();
}

// ������
void InitBullet()
{
    // �f�t�H���g�̏Փˌ��m�A���S���Y��
    collisionConfiguration = new btDefaultCollisionConfiguration();
    
    // �f�t�H���g�̏Փ˃f�B�X�p�b�`��
    dispatcher = new btCollisionDispatcher(collisionConfiguration);
    
    // �K�w�I�ȏՓˌ��m�A���S���Y��
    overlappingPairCache = new btDbvtBroadphase();
    
    // �V�[�P���V����(�����)�ȃC���p���X��@
    solver = new btSequentialImpulseConstraintSolver;
    
    dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
    
    dynamicsWorld->setGravity(btVector3(0, -10, 0));
}



// ���C��
int main(int argc, char** argv)
{
    // ������
    InitBullet();

	// �O�����h�̍쐬
	CreateGround();

	// ���̍쐬
	//CreateBall();

	//�q�g�f�̐���
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

	// �I������
	CleanupBullet();
}

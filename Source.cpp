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
#include <math.h>

#define RADIAN 180/M_PI

using namespace std;

btDefaultCollisionConfiguration* collisionConfiguration;
btCollisionDispatcher* dispatcher;
btBroadphaseInterface* overlappingPairCache;
btSequentialImpulseConstraintSolver* solver;
btDiscreteDynamicsWorld* dynamicsWorld;
btCollisionShape* groundShape;
btAlignedObjectArray<btCollisionShape*> collisionShapes;

GLfloat light0pos[] = { 300.0, 300.0, 300.0, 1.0 };
GLfloat light1pos[] = { 5.0, 3.0, 0.0, 1.0 };

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

	// �쐬�������̂����[���h�֓o�^
	dynamicsWorld->addRigidBody(body);
}

// ���̐���
void CreateBall()
{
	btScalar scale = 5;
	btCollisionShape* colShape = new btSphereShape(scale);
	collisionShapes.push_back(colShape);

	// �g�����X�t�H�[�����쐬
	btTransform startTransform;
	startTransform.setIdentity();

	btScalar mass(10.f);

	// ���ʂ�1�Ƃ����̂Ń_�C�i�~�b�N
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		colShape->calculateLocalInertia(mass, localInertia);

	startTransform.setOrigin(btVector3(2, 15, 0));

	// ���[�V�����X�e�[�g���쐬
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);

	dynamicsWorld->addRigidBody(body);
}

//�q�g�f�̓��̐���
btRigidBody* initBody(const btVector3& scale, const btVector3& position)
{
    btCollisionShape* sBodyShape = new btBoxShape(scale);
    collisionShapes.push_back(sBodyShape);
    
    btTransform sBodyTransform;
    sBodyTransform.setIdentity();
    sBodyTransform.setOrigin(position);
    
    btScalar mass(0.);
    
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
btRigidBody* initArm(const btVector3& scale, const btVector3& position)
{
	btCollisionShape* sBodyShape = new btBoxShape(scale);
	collisionShapes.push_back(sBodyShape);

	btScalar mass1(1.);
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

    return body;

}

// �q�g�f�̐���
void CreateStarfish()
{
    vector<btRigidBody* > bodies;
    
/***������***/
    btVector3 scale_b = btVector3(btScalar(25.), btScalar(10.), btScalar(25.));
    btVector3 position_b = btVector3(0, 50, 0);
    bodies.push_back(initBody(scale_b, position_b));
	
/***���r***/
	btVector3 scale_a = btVector3(btScalar(25.), btScalar(10.), btScalar(25.));
    btScalar dist = scale_a[0]+scale_b[0]+scale_b[1]*2;

	bodies.push_back(initArm(scale_a, btVector3(dist, position_b[1], 0)));
	bodies.push_back(initArm(scale_a, btVector3(-dist, position_b[1], 0)));
	bodies.push_back(initArm(scale_a, btVector3(0, position_b[1], dist)));
	bodies.push_back(initArm(scale_a, btVector3(0, position_b[1], -dist)));
    
/***���Ǒ�***/
    
    
    
    for (int i = 0; i < bodies.size(); i++) {
        dynamicsWorld->addRigidBody(bodies[i]);
    }
    
//////�S��///////
    btVector3 pivotInBody = btVector3(scale_b[0], 0, 0);
    btVector3 axisInBody = btVector3(0, 0, 1);
    //btVector3 pivotInArm = btVector3(-scale_a[0]-scale_b[1]*2, 0, 0);
    btVector3 pivotInArm = btVector3(10, 10, 0);
    btVector3 axisInArm = axisInBody;
    
    //btHingeConstraint* hinge = new btHingeConstraint(*bodies[0], *bodies[1], pivotInBody, pivotInArm, axisInBody, axisInArm);
    btHingeConstraint* hinge = new btHingeConstraint(*bodies[1], pivotInArm, axisInArm);
    dynamicsWorld->addConstraint(hinge);

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

void Render()
{
#if 0
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); //�o�b�t�@�̏���

	//���f���r���[�ϊ��s��̐ݒ�--------------------------
	glMatrixMode(GL_MODELVIEW);//�s�񃂁[�h�̐ݒ�iGL_PROJECTION : �����ϊ��s��̐ݒ�AGL_MODELVIEW�F���f���r���[�ϊ��s��j
	glLoadIdentity();//�s��̏�����
	glViewport(0, 0, 640, 480);
	//----------------------------------------------

	//�A�eON-----------------------------
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);//����0�𗘗p
	//-----------------------------------

	//��
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_AMBIENT, ms_ruby.ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, ms_ruby.diffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, ms_ruby.specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, &ms_ruby.shininess);
	glTranslated(0.0, 0.0, 0.0);//���s�ړ��l�̐ݒ�
	glutSolidSphere(10.0, 20, 20);//�����F(���a, Z���܂��̕�����, Z���ɉ�����������)
	glPopMatrix();


	//�A�eOFF-----------------------------
	glDisable(GL_LIGHTING);
	//-----------------------------------

	glutSwapBuffers(); //glutInitDisplayMode(GLUT_DOUBLE)�Ń_�u���o�b�t�@�����O�𗘗p��
#else
	dynamicsWorld->stepSimulation(1.f / 60.f, 10);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glLightfv(GL_LIGHT0, GL_POSITION, light0pos);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
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
            btScalar rot = btScalar(body->getOrientation().getAngle() / RADIAN);
            btVector3 axis = body->getOrientation().getAxis();
            btVector3 halfExtent = static_cast<const btBoxShape*>(body->getCollisionShape())->getHalfExtentsWithMargin();
            
			glPushMatrix();
            glTranslatef(pos[0], pos[1], pos[2]);
            glRotated(rot, axis[0], axis[1], axis[2]);
			if (j == 0)
			{
				glScaled(2 * halfExtent[0], 2 * halfExtent[1], 2 * halfExtent[2]);
				glMaterialfv(GL_FRONT, GL_AMBIENT, ms_jade.ambient);
				glMaterialfv(GL_FRONT, GL_DIFFUSE, ms_jade.diffuse);
				glMaterialfv(GL_FRONT, GL_SPECULAR, ms_jade.specular);
				glMaterialfv(GL_FRONT, GL_SHININESS, &ms_jade.shininess);
				glutSolidCube(1);
			}
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
				glScaled(halfExtent[0], halfExtent[1], halfExtent[2]);
				glMaterialfv(GL_FRONT, GL_AMBIENT, ms_ruby.ambient);
				glMaterialfv(GL_FRONT, GL_DIFFUSE, ms_ruby.diffuse);
				glMaterialfv(GL_FRONT, GL_SPECULAR, ms_ruby.specular);
				glMaterialfv(GL_FRONT, GL_SHININESS, &ms_ruby.shininess);
				glutSolidSphere(1, 100, 100);
			}
			glPopMatrix();
		}
	}

	/* ���f���r���[�ϊ��s��̕��A */
	glPopMatrix();

	glDisable(GL_LIGHTING);

	glutSwapBuffers();
#endif
}

/// �����������
void resize(int w, int h)
{
	// �E�B���h�E�S�̂��r���[�|�[�g�ɂ���
	glViewport(0, 0, w, h);

	/* �����ϊ��s��̐ݒ� */
	//glMatrixMode(GL_PROJECTION);

	// �ϊ��s��̏����� 
	glLoadIdentity();
	
#if 0
	gluPerspective(30.0, (double)w / (double)h, 0.1, 1000);

	/* ���f���r���[�ϊ��s��̐ݒ� */
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	/* ���_�ʒu�Ǝ������� */
	//gluLookAt(300 , 10 , 0 , 0.0, 0 , 0.0, 0.0, 1.0, 0.0);
	gluLookAt(100, 0, 0, 0.0, 0, 0.0, 0.0, 1.0, 0.0);
#endif
}


void init(void)
{
	glClearColor(1.0, 1.0, 1.0, 1.0);
	glEnable(GL_DEPTH_TEST);
	//glEnable(GL_CULL_FACE);
	glLightfv(GL_LIGHT0, GL_POSITION, light0pos);
	//glCullFace(GL_BACK);
	//glCullFace(GL_FRONT);

	glMatrixMode(GL_PROJECTION);//�s�񃂁[�h�̐ݒ�iGL_PROJECTION : �����ϊ��s��̐ݒ�AGL_MODELVIEW�F���f���r���[�ϊ��s��j
	glLoadIdentity();//�s��̏�����
	gluPerspective(30.0, (double)640 / (double)480, 0.1, 1000);
	gluLookAt(0, 100, 300, 0.0, 0, 0.0, 0.0, 1.0, 0.0);
}

void idle(void)
{
	glutPostRedisplay();
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
	//glutReshapeFunc(resize);
	init();
	glutMainLoop();

	// �I������
	CleanupBullet();
}

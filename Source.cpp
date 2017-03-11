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
// 物質質感の定義
//----------------------------------------------------
struct MaterialStruct {
	GLfloat ambient[4];
	GLfloat diffuse[4];
	GLfloat specular[4];
	GLfloat shininess;
};
//jade(翡翠)
MaterialStruct ms_jade = {
	{ 0.135, 0.2225, 0.1575, 1.0 },
	{ 0.54, 0.89, 0.63, 1.0 },
	{ 0.316228, 0.316228, 0.316228, 1.0 },
	12.8 };
//ruby(ルビー)
MaterialStruct ms_ruby = {
	{ 0.1745, 0.01175, 0.01175, 1.0 },
	{ 0.61424, 0.04136, 0.04136, 1.0 },
	{ 0.727811, 0.626959, 0.626959, 1.0 },
	76.8 };
//----------------------------------------------------
// 色の定義の定義
//----------------------------------------------------
GLfloat red[] = { 0.8, 0.2, 0.2, 1.0 }; //赤色
GLfloat green[] = { 0.2, 0.8, 0.2, 1.0 };//緑色
GLfloat blue[] = { 0.2, 0.2, 0.8, 1.0 };//青色
GLfloat yellow[] = { 0.8, 0.8, 0.2, 1.0 };//黄色
GLfloat white[] = { 1.0, 1.0, 1.0, 1.0 };//白色
GLfloat shininess = 30.0;//光沢の強さ
//-----------------------------------------


// グランドの生成
void CreateGround()
{
	// シェイプの生成
	btVector3 scale = btVector3(btScalar(500.), btScalar(50.), btScalar(500.));
	groundShape = new btBoxShape(scale);
	collisionShapes.push_back(groundShape);
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, -56, 0));

	btScalar mass(0.);

	// 静的な剛体を作ります
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		groundShape->calculateLocalInertia(mass, localInertia);

	// デフォルトのモーションステートを作成
	btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);

	// 作成した剛体をワールドへ登録
	dynamicsWorld->addRigidBody(body);
}

// 球の生成
void CreateBall()
{
	btScalar scale = 5;
	btCollisionShape* colShape = new btSphereShape(scale);
	collisionShapes.push_back(colShape);

	// トランスフォームを作成
	btTransform startTransform;
	startTransform.setIdentity();

	btScalar mass(10.f);

	// 質量を1としたのでダイナミック
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		colShape->calculateLocalInertia(mass, localInertia);

	startTransform.setOrigin(btVector3(2, 15, 0));

	// モーションステートを作成
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);

	dynamicsWorld->addRigidBody(body);
}

//ヒトデの胴体生成
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
    
    // デフォルトのモーションステートを作成
    btDefaultMotionState* myMotionState = new btDefaultMotionState(sBodyTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, sBodyShape, localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);
    
    return body;
}

//ヒトデの腕生成
btRigidBody* initArm(const btVector3& scale, const btVector3& position)
{
	btCollisionShape* sBodyShape = new btBoxShape(scale);
	collisionShapes.push_back(sBodyShape);

	btScalar mass1(1.);
	bool isDynamic = (mass1 != 0.f);

	btVector3 localInertia1(0, 0, 0);
	if (isDynamic)
		groundShape->calculateLocalInertia(mass1, localInertia1);

	// デフォルトのモーションステートを作成

	btTransform sBodyTransform;
	sBodyTransform.setIdentity();
	sBodyTransform.setOrigin(position);
	btDefaultMotionState* myMotionState1 = new btDefaultMotionState(sBodyTransform);

	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass1, myMotionState1, sBodyShape, localInertia1);
	btRigidBody* body = new btRigidBody(rbInfo);

    return body;

}

// ヒトデの生成
void CreateStarfish()
{
    vector<btRigidBody* > bodies;
    
/***↓胴体***/
    btVector3 scale_b = btVector3(btScalar(25.), btScalar(10.), btScalar(25.));
    btVector3 position_b = btVector3(0, 50, 0);
    bodies.push_back(initBody(scale_b, position_b));
	
/***↓腕***/
	btVector3 scale_a = btVector3(btScalar(25.), btScalar(10.), btScalar(25.));
    btScalar dist = scale_a[0]+scale_b[0]+scale_b[1]*2;

	bodies.push_back(initArm(scale_a, btVector3(dist, position_b[1], 0)));
	bodies.push_back(initArm(scale_a, btVector3(-dist, position_b[1], 0)));
	bodies.push_back(initArm(scale_a, btVector3(0, position_b[1], dist)));
	bodies.push_back(initArm(scale_a, btVector3(0, position_b[1], -dist)));
    
/***↓管足***/
    
    
    
    for (int i = 0; i < bodies.size(); i++) {
        dynamicsWorld->addRigidBody(bodies[i]);
    }
    
//////拘束///////
    btVector3 pivotInBody = btVector3(scale_b[0], 0, 0);
    btVector3 axisInBody = btVector3(0, 0, 1);
    //btVector3 pivotInArm = btVector3(-scale_a[0]-scale_b[1]*2, 0, 0);
    btVector3 pivotInArm = btVector3(10, 10, 0);
    btVector3 axisInArm = axisInBody;
    
    //btHingeConstraint* hinge = new btHingeConstraint(*bodies[0], *bodies[1], pivotInBody, pivotInArm, axisInBody, axisInArm);
    btHingeConstraint* hinge = new btHingeConstraint(*bodies[1], pivotInArm, axisInArm);
    dynamicsWorld->addConstraint(hinge);

}

// 終了処理
void CleanupBullet()
{
	// 剛体を削除
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

	// 衝突シェイプを削除
	for (int j = 0; j<collisionShapes.size(); j++)
	{
		btCollisionShape* shape = collisionShapes[j];
		collisionShapes[j] = 0;
		delete shape;
	}

	// ワールドを削除
	delete dynamicsWorld;

	// ソルバーを削除
	delete solver;

	// 衝突検知のブロードフェイズを削除
	delete overlappingPairCache;

	// ディスパッチャを削除
	delete dispatcher;

	delete collisionConfiguration;

	// オプション、なくても良い
	collisionShapes.clear();
}

// 初期化
void InitBullet()
{
	// デフォルトの衝突検知アルゴリズム
	collisionConfiguration = new btDefaultCollisionConfiguration();

	// デフォルトの衝突ディスパッチャ
	dispatcher = new btCollisionDispatcher(collisionConfiguration);

	// 階層的な衝突検知アルゴリズム
	overlappingPairCache = new btDbvtBroadphase();

	// シーケンシャル(非並列)なインパルス解法
	solver = new btSequentialImpulseConstraintSolver;

	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);

	dynamicsWorld->setGravity(btVector3(0, -10, 0));
}

void Render()
{
#if 0
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); //バッファの消去

	//モデルビュー変換行列の設定--------------------------
	glMatrixMode(GL_MODELVIEW);//行列モードの設定（GL_PROJECTION : 透視変換行列の設定、GL_MODELVIEW：モデルビュー変換行列）
	glLoadIdentity();//行列の初期化
	glViewport(0, 0, 640, 480);
	//----------------------------------------------

	//陰影ON-----------------------------
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);//光源0を利用
	//-----------------------------------

	//球
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_AMBIENT, ms_ruby.ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, ms_ruby.diffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, ms_ruby.specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, &ms_ruby.shininess);
	glTranslated(0.0, 0.0, 0.0);//平行移動値の設定
	glutSolidSphere(10.0, 20, 20);//引数：(半径, Z軸まわりの分割数, Z軸に沿った分割数)
	glPopMatrix();


	//陰影OFF-----------------------------
	glDisable(GL_LIGHTING);
	//-----------------------------------

	glutSwapBuffers(); //glutInitDisplayMode(GLUT_DOUBLE)でダブルバッファリングを利用可
#else
	dynamicsWorld->stepSimulation(1.f / 60.f, 10);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glLightfv(GL_LIGHT0, GL_POSITION, light0pos);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	//glDisable(GL_LIGHT1);

	/* モデルビュー変換行列の保存 */
	glPushMatrix();

	// 剛体の座標をプリント
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

	/* モデルビュー変換行列の復帰 */
	glPopMatrix();

	glDisable(GL_LIGHTING);

	glutSwapBuffers();
#endif
}

/// 動作怪しいよ
void resize(int w, int h)
{
	// ウィンドウ全体をビューポートにする
	glViewport(0, 0, w, h);

	/* 透視変換行列の設定 */
	//glMatrixMode(GL_PROJECTION);

	// 変換行列の初期化 
	glLoadIdentity();
	
#if 0
	gluPerspective(30.0, (double)w / (double)h, 0.1, 1000);

	/* モデルビュー変換行列の設定 */
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	/* 視点位置と視線方向 */
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

	glMatrixMode(GL_PROJECTION);//行列モードの設定（GL_PROJECTION : 透視変換行列の設定、GL_MODELVIEW：モデルビュー変換行列）
	glLoadIdentity();//行列の初期化
	gluPerspective(30.0, (double)640 / (double)480, 0.1, 1000);
	gluLookAt(0, 100, 300, 0.0, 0, 0.0, 0.0, 1.0, 0.0);
}

void idle(void)
{
	glutPostRedisplay();
}


// メイン
int main(int argc, char** argv)
{
	// 初期化
	InitBullet();

	// グランドの作成
	CreateGround();

	// 球の作成
	//CreateBall();

	//ヒトデの生成
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

	// 終了処理
	CleanupBullet();
}

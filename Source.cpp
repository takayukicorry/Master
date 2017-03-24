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

GLfloat light0pos[] = { 300.0, 300.0, 300.0, 1.0 };
GLfloat light1pos[] = { -300.0, 300.0, 300.0, 1.0 };

int time_step = 0;

enum CollisionGroup{
    RX_COL_NOTHING = 0, // 0000
    RX_COL_GROUND = 1,  // 0001
    RX_COL_BODY = 2,  // 0010
    RX_COL_TF = 4   // 0100
};

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
//計算する関数
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
//---------------------------------------------

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

    body->setActivationState(DISABLE_DEACTIVATION);
    
	// 作成した剛体をワールドへ登録
    dynamicsWorld->addRigidBody(body, RX_COL_GROUND, RX_COL_BODY | RX_COL_TF);
    ///dynamicsWorld->addRigidBody(body);
}

// 球の生成
btRigidBody* CreateBall(btScalar scale, const btVector3 position)
{
	btCollisionShape* colShape = new btSphereShape(scale);
	collisionShapes.push_back(colShape);

	// トランスフォームを作成
	btTransform startTransform;
	startTransform.setIdentity();

	btScalar mass(0.f);

	// 質量を1としたのでダイナミック
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		colShape->calculateLocalInertia(mass, localInertia);

	startTransform.setOrigin(position);

	// モーションステートを作成
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);

    return  body;
}

//ヒトデの胴体生成
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
    
    // デフォルトのモーションステートを作成
    btDefaultMotionState* myMotionState = new btDefaultMotionState(sBodyTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, sBodyShape, localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);
    
    return body;
}

//ヒトデの腕生成
btRigidBody* initArm(const btVector3 scale, const btVector3 position, const btQuaternion rot)
{
	btCollisionShape* sBodyShape = new btBoxShape(scale);
	collisionShapes.push_back(sBodyShape);

	btScalar mass1(M_ARM);
	bool isDynamic = (mass1 != 0.f);

	btVector3 localInertia1(0, 0, 0);
	if (isDynamic)
		groundShape->calculateLocalInertia(mass1, localInertia1);

	// デフォルトのモーションステートを作成

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

//ヒトデの管足生成
btRigidBody* initTubefeet(btScalar* scale, const btVector3 position)
{
    btCollisionShape* sBodyShape = new btCapsuleShape(scale[0], scale[1]);
    collisionShapes.push_back(sBodyShape);
    
    btScalar mass1(M_TF);
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
    
    body->setActivationState(DISABLE_DEACTIVATION);
    
    return body;
}

// ヒトデの生成
void CreateStarfish()
{
    vector<btRigidBody* > bodies_body;
    vector<btRigidBody* > bodies_tf;
    vector<btTypedConstraint* > constraints;
    
#if TUBEFEET_SIMULATION_MODE //管足一個のシミュレーション
/***胴体***/
    
/***管足***/
    btScalar scale[] = {btScalar(RADIUS), btScalar(LENGTH)};
    bodies_tf.push_back(CreateBall(btScalar(RADIUS), btVector3(0, btScalar(INIT_POS_Y), 0)));
    bodies_tf.push_back(initTubefeet(scale, btVector3(0, INIT_POS_Y-RADIUS*2-LENGTH/2, 0)));
    //拘束
    btUniversalConstraint* univ = new btUniversalConstraint(*bodies_tf[0], *bodies_tf[1], btVector3(0, 24, 0), btVector3(0, 1, 0), btVector3(0, 0, 1));//全部グローバル
    constraints.push_back(univ);
    //モーター
    btRotationalLimitMotor* motor1 = univ->getRotationalLimitMotor(1);//車輪
    btRotationalLimitMotor* motor2 = univ->getRotationalLimitMotor(2);//ステアリング
    motor1->m_enableMotor = true;
    motor2->m_enableMotor = true;
    motor_tZ.push_back(motor1);
    motor_tY.push_back(motor2);
    
    
#else
    /***↓胴体***/
    btVector3 scale_b(btScalar(25.), btScalar(5.), btScalar(25.));
    btVector3 position_b(0, 10, 0);
    bodies_body.push_back(initBody(scale_b, position_b));
	
/***↓腕***/
	btVector3 scale_a(btScalar(25.), btScalar(5.), btScalar(10.));
    btScalar dist = scale_a[0]+scale_b[0]+scale_b[1];

	bodies_body.push_back(initArm(scale_a, btVector3(dist, position_b[1], 0), btQuaternion(0, 0, 0, 1)));
	bodies_body.push_back(initArm(scale_a, btVector3(0, position_b[1], -dist), btQuaternion(0, 1/sqrt(2), 0, 1/sqrt(2))));
	bodies_body.push_back(initArm(scale_a, btVector3(-dist, position_b[1], 0), btQuaternion(0, 1, 0, 0)));
	bodies_body.push_back(initArm(scale_a, btVector3(0, position_b[1], dist), btQuaternion(0, -1/sqrt(2), 0, 1/sqrt(2))));
    
    //拘束
    btVector3 pivotInBody(scale_b[0], 0, 0);
    btVector3 axisInBody(0, 0, 1);
    btVector3 pivotInArm(-scale_a[0]-scale_b[1], 0, 0);
    btVector3 axisInArm = axisInBody;
    btHingeConstraint* hinge = new btHingeConstraint(*bodies_body[0], *bodies_body[1], pivotInBody, pivotInArm, axisInBody, axisInArm);//全部ローカル
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
    
/***↓管足***/
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
            
            //拘束
            position_t[1] += scale_t[0] + scale_t[1]/2;
            btUniversalConstraint* univ = new btUniversalConstraint(*bodies_body[j], *body_t, position_t, btVector3(0, 1, 0), btVector3(sin(M_PI_2*(j-1)), 0, cos(M_PI_2*(j-1))));//全部グローバル
            
            univ->setLowerLimit(-ANGLE, -ANGLE);
            univ->setUpperLimit(ANGLE, ANGLE);
            
            constraints.push_back(univ);

            //モーター
            btRotationalLimitMotor* motor1 = univ->getRotationalLimitMotor(1);//車輪
            btRotationalLimitMotor* motor2 = univ->getRotationalLimitMotor(2);//ステアリング
            motor1->m_enableMotor = true;
            motor2->m_enableMotor = true;
            
            motor_tZ.push_back(motor1);
            motor_tY.push_back(motor2);

            position_t[1] -= scale_t[0] + scale_t[1]/2;
            position_t = RotateY(position_t, M_PI_2);
            
        }
    }
#endif
    
////登録////
    
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
    
    btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[1];
    btRigidBody* body = btRigidBody::upcast(obj);
    if (body && body->getMotionState())
    {
        btVector3 pos = body->getCenterOfMassPosition();
        
        btTransform tran;
        tran.setIdentity();
        tran.setOrigin(btVector3(pos[0], INIT_POS_Y - (LENGTH/2 + 4)/2 + (LENGTH/2 + 4)/2*sin(2*M_PI*(time_step%(SECOND*2))/(SECOND*2) + M_PI_2), pos[2]));
        
        body->setCenterOfMassTransform(tran);
    
    }
    
    for (int i = 0; i < motor_tZ.size(); i++) {
        motor_tZ[i]->m_maxMotorForce = 100000000;
        motor_tY[i]->m_maxMotorForce = 100000000;
        
        if ((time_step/SECOND+1) / 2 % 2 == 0) {
            motor_tZ[i]->m_targetVelocity = ANGLE;
        }else{
            motor_tZ[i]->m_targetVelocity = -ANGLE;
        }
        
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
            btScalar rot = body->getOrientation().getAngle() * RADIAN;
            btVector3 axis = body->getOrientation().getAxis();
            btVector3 halfExtent = static_cast<const btBoxShape*>(body->getCollisionShape())->getHalfExtentsWithMargin();
            
			glPushMatrix();
            glTranslatef(pos[0], pos[1], pos[2]);
            glRotated(rot, axis[0], axis[1], axis[2]);
            //地面
            if (j == 0)
			{
				glScaled(2 * halfExtent[0], 2 * halfExtent[1], 2 * halfExtent[2]);
				glMaterialfv(GL_FRONT, GL_AMBIENT, ms_jade.ambient);
				glMaterialfv(GL_FRONT, GL_DIFFUSE, ms_jade.diffuse);
				glMaterialfv(GL_FRONT, GL_SPECULAR, ms_jade.specular);
				glMaterialfv(GL_FRONT, GL_SHININESS, &ms_jade.shininess);
				glutSolidCube(1);
			}
            //胴体　腕
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
            //管足
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

	/* モデルビュー変換行列の復帰 */
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

	glMatrixMode(GL_PROJECTION);//行列モードの設定（GL_PROJECTION : 透視変換行列の設定、GL_MODELVIEW：モデルビュー変換行列）
	glLoadIdentity();//行列の初期化
	gluPerspective(30.0, (double)640 / (double)480, 0.1, 1000);
	gluLookAt(0, 0, 300, 0.0, 0, 0.0, 0.0, 1.0, 0.0);
}

void idle(void)
{
	glutPostRedisplay();
    ControllTubeFeet();
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
    
    for(int i = dynamicsWorld->getNumConstraints()-1; i>=0 ;i--){
        btTypedConstraint* constraint = dynamicsWorld->getConstraint(i);
        dynamicsWorld->removeConstraint(constraint);
        delete constraint;
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
	init();
	glutMainLoop();

	// 終了処理
	CleanupBullet();
}

//
//  Ophiuroid.cpp
//  Master
//
//  Created by 増田貴行 on 2017/10/31.
//  Copyright © 2017年 増田貴行. All rights reserved.
//

#include "Ophiuroid.hpp"
#include "Utils.hpp"

Ophiuroid::Ophiuroid(GAparameter p) {
    for (int i = 0; i < NUM_LEGS; i++) {
        leg_state[i] = 0;
    }
    m_param = p;
    m_time_step = 0;
    className = 1;
    swing_phase = -3;
    hasNet = false;
    check = true;
}

Ophiuroid::Ophiuroid(Starfish* sf) {
    for (int i = 0; i < NUM_LEGS; i++) {
        leg_state[i] = 0;
    }
    m_param = sf->m_param;
    m_time_step = 0;
    className = 1;
    swing_phase = -3;
    
    m_bodies = sf->m_bodies;
    m_shapes = sf->m_shapes;
    m_joints_hip = sf->m_joints_hip;
    m_joints_ankle = sf->m_joints_ankle;
    m_motor1 = sf->m_motor1;
    m_motor2 = sf->m_motor2;
    hasNet = false;
    check = true;
}

void Ophiuroid::idle() {
    glutPostRedisplay();
}

void Ophiuroid::idleDemo() {
    m_time_step++;
    checkState();
    setMotorTarget2(1);
}

void Ophiuroid::idleNEAT() {
    m_time_step++;
    checkState();
    setMotorTarget2_NEAT(1);
}

void motorPreTickCallback(btDynamicsWorld *world, btScalar timeStep) {
    Ophiuroid* demo = (Ophiuroid*)world->getWorldUserInfo();
    demo->idleDemo();
}

void motorPreTickCallback_NEAT(btDynamicsWorld *world, btScalar timeStep) {
    Ophiuroid* demo = (Ophiuroid*)world->getWorldUserInfo();
    demo->idleNEAT();
}

void Ophiuroid::ev() {
    btScalar value = swing_phase;
    btVector3 up = btVector3(0, 1, 0);
    btScalar val = swing_phase;
    if (swing_phase >= -1) {
        btTransform tr = m_bodies[0]->getWorldTransform();
        btVector3 vY = tr*up;
        btVector3 vOrigin = tr.getOrigin();
        btVector3 vY_O = vY - vOrigin;
        val = asin(vY_O[1])+M_PI_2;
    }
    if (value < val) {
        value = val;
    }
    
    std::cout << value << "  " << m_time_step << std::endl;
}

float Ophiuroid::evalue() {
    btDiscreteDynamicsWorld* dynamicsWorld = GAMaster::createWorld();
    dynamicsWorld->setGravity(btVector3(0, -10, 0));
    //dynamicsWorld->setInternalTickCallback(motorPreTickCallback, this, true);
    setWorld(dynamicsWorld);

    GAMaster::createGround(m_ownerWorld);
    initSF();
    create();
    btScalar value = swing_phase;
    btVector3 up = btVector3(0, 1, 0);
    for (int i = 0; i < SIMULATION_TIME_STEP; i++) {
        m_ownerWorld->stepSimulation(1.f / FPS);
        btScalar val = swing_phase;
        if (swing_phase >= -1) {
            btTransform tr = m_bodies[0]->getWorldTransform();
            btVector3 vY = tr*up;
            btVector3 vOrigin = tr.getOrigin();
            btVector3 vY_O = vY - vOrigin;
            val = asin(vY_O[1])+M_PI_2;
        
        }
        if (value < val) {
            value = val;
        }
    }
    
    GAMaster::cleanupWorld(m_ownerWorld);
    return value;
}

float Ophiuroid::evalue_NEAT(NEAT::Network* net) {
    btDiscreteDynamicsWorld* dynamicsWorld = GAMaster::createWorld();
    dynamicsWorld->setGravity(btVector3(0, -10, 0));
    //dynamicsWorld->setInternalTickCallback(motorPreTickCallback_NEAT, this, true);
    setWorld(dynamicsWorld);
    setNet(net);
    
    GAMaster::createGround(m_ownerWorld);
    initSF();
    create();
    btScalar value = swing_phase;
    btVector3 up = btVector3(0, 1, 0);
    for (int i = 0; i < SIMULATION_TIME_STEP; i++) {
        m_ownerWorld->stepSimulation(1.f / FPS);
        btScalar val = swing_phase;
        if (swing_phase >= -1) {
            btTransform tr = m_bodies[0]->getWorldTransform();
            btVector3 vY = tr*up;
            btVector3 vOrigin = tr.getOrigin();
            btVector3 vY_O = vY - vOrigin;
            val = asin(vY_O[1])+M_PI_2;
        }
        if (value < val) {
            value = val;
        }
    }
    
    GAMaster::cleanupWorld(m_ownerWorld);
    return value;
}

bool Ophiuroid::checkState() {
    btTransform tr = m_bodies[0]->getWorldTransform();
    btVector3 vY = tr*btVector3(0, 1, 0);
    btVector3 vOrigin = tr.getOrigin();
    
    btVector3 vY_O = vY - vOrigin;
    check = vY_O[1] < THRESH_TURN;
    return check;
}

void Ophiuroid::initSF() {
    
    float alpha = 37*M_PI/36;
    float theta = M_PI - alpha;
    
    m_shapes[0] = new btCylinderShape(btVector3(FBODY_SIZE,FLEG_WIDTH,FBODY_SIZE));
    int i;
    
    for ( i=0; i<NUM_LEGS; i++)
    {
        m_shapes[1 + (NUM_JOINT+1)*i] = new btSphereShape(FLEG_WIDTH);
        for (int k = 1; k <= NUM_JOINT; k++)
        {
            m_shapes[k + 1 + (NUM_JOINT+1)*i] = new btCapsuleShape(btScalar(FLEG_WIDTH), btScalar(FLEG_LENGTH));
        }
    }
    
    //
    // Setup rigid bodies
    //
    // root
    btVector3 vRoot = btVector3(btScalar(0.), btScalar(FHEIGHT), btScalar(0.));
    btTransform transform, transformY, transformS, transformSY, transformVersX, transformVersY, transformVersZ;
    transformVersX.setIdentity();
    transformVersX.setRotation(btQuaternion(btVector3(1,0,0),M_PI));
    transformVersY.setIdentity();
    transformVersY.setRotation(btQuaternion(btVector3(0,1,0),M_PI));
    transformVersZ.setIdentity();
    transformVersZ.setRotation(btQuaternion(btVector3(0,0,1),M_PI));
    transform.setIdentity();
    transform.setOrigin(vRoot);
    m_bodies[0] = createRigidBody(btScalar(M_OBJ0), transform*transformVersX, m_shapes[0], 50);
    
    // legs
    for ( i=0; i<NUM_LEGS; i++)
    {
        float fAngle = 2 * M_PI * i / NUM_LEGS;
        float fSin = sin(fAngle);//左ねじの方向に正
        float fCos = cos(fAngle);
        
        btVector3 vBoneOrigin = btVector3(btScalar(fCos*(FBODY_SIZE + FLEG_WIDTH + (0.5*FLEG_LENGTH+2*FLEG_WIDTH))), btScalar(FHEIGHT), btScalar(fSin*(FBODY_SIZE + FLEG_WIDTH + (0.5*FLEG_LENGTH+2*FLEG_WIDTH))));
        btVector3 Point = vBoneOrigin;
        btVector3 spherePoint = btVector3(btScalar(fCos*(FBODY_SIZE + FLEG_WIDTH)), btScalar(FHEIGHT), btScalar(fSin*(FBODY_SIZE + FLEG_WIDTH)));
        
        transformS.setIdentity();
        transformS.setOrigin(spherePoint);
        transformS.setRotation(btQuaternion(btVector3(0, 1, 0), -fAngle));//右ねじの方向に正
        transformSY.setIdentity();
        transformSY.setRotation(btQuaternion(btVector3(0, 0, 1), M_PI_2));
        
        m_bodies[1+(NUM_JOINT+1)*i] = createRigidBody(btScalar(M_OBJ), transformS*transformSY*transformVersY, m_shapes[1+(NUM_JOINT+1)*i], 50+1+(NUM_JOINT+1)*i);
        
        for (int k = 1; k <= NUM_JOINT; k++)
        {
            transform.setIdentity();
            transform.setOrigin(Point);
            transform.setRotation(btQuaternion(btVector3(0, 1, 0), -fAngle));
            transformY.setIdentity();
            transformY.setRotation(btQuaternion(btVector3(0, 0, 1), M_PI_2));
            
            m_bodies[k+1+(NUM_JOINT+1)*i] = createRigidBody(btScalar(M_OBJ), transform*transformY*transformVersY, m_shapes[k+1+(NUM_JOINT+1)*i], 50+k+1+(NUM_JOINT+1)*i);
            Point += btVector3(btScalar(fCos*(0.5*FLEG_LENGTH+FLEG_WIDTH)*2),btScalar(0),btScalar(fSin*(0.5*FLEG_LENGTH+FLEG_WIDTH))*2);
        }
        m_bodies[(NUM_JOINT+1)*(i+1)]->setFriction(5.0);
    }
    
    //停止が続いてもsleeping状態にならないようにする//
    // Setup some damping on the m_bodies
    
    for (i = 0; i < m_bodies.size(); ++i)
    {
        m_bodies[i]->setDamping(0.05, 0.85);
        m_bodies[i]->setDeactivationTime(1000000000000.f);
        m_bodies[i]->setSleepingThresholds(0.0f, 0.0f);
        m_bodies[i]->forceActivationState(DISABLE_DEACTIVATION);
        
    }
    
    //
    // Setup the constraints
    //
    btTransform localA, localB, localC, jP, objP;
    
    for ( i=0; i<NUM_LEGS; i++)
    {
        float fAngle = 2 * M_PI * i / NUM_LEGS;
        float fSin = sin(fAngle);
        float fCos = cos(fAngle);
        
        // hip joints
        btVector3 parentAxis( fCos, 0, fSin);//第一ジョイント回転軸１（ワールド座標）-->ローカルz軸(handle)
        btVector3 childAxis( fSin, 0, -fCos);//第一ジョイント回転軸２（ワールド座標）-->ローカルy軸(wheel)
        btVector3 anchor( fCos*(FBODY_SIZE+FLEG_WIDTH), FHEIGHT, fSin*(FBODY_SIZE+FLEG_WIDTH));//上の２軸の交点（ワールド座標）
        
        btUniversalConstraint* hinge2C = new btUniversalConstraint(*m_bodies[0], *m_bodies[1+(NUM_JOINT+1)*i], anchor, parentAxis, childAxis);
        hinge2C->setLinearLowerLimit(btVector3(0,0,0));
        hinge2C->setLinearUpperLimit(btVector3(0,0,0));
        hinge2C->setLowerLimit(-M_PI_2, -M_PI);//(wheel, handle)右ねじ
        hinge2C->setUpperLimit(M_PI_2, M_PI);
        hinge2C->setUserConstraintId(10);
        m_joints_hip[i] = hinge2C;
        m_ownerWorld->addConstraint(m_joints_hip[i], true);
        
        const int AXIS1_ID = 2;
        const int AXIS2_ID = 1;
        m_motor1[i] = hinge2C->getRotationalLimitMotor(AXIS1_ID);
        m_motor2[i] = hinge2C->getRotationalLimitMotor(AXIS2_ID);
        m_motor1[i]->m_maxMotorForce = MAX_MOTOR_TORQUE;
        m_motor2[i]->m_maxMotorForce = MAX_MOTOR_TORQUE;
        
        m_motor1[i]->m_enableMotor = true;
        m_motor2[i]->m_enableMotor = true;
        
        btVector3 JointPoint = btVector3(btScalar(fCos*(FBODY_SIZE+FLEG_WIDTH)), btScalar(0.), btScalar(fSin*(FBODY_SIZE+FLEG_WIDTH)));
        
        
        for (int k = 1; k <= NUM_JOINT; k++)
        {
            btVector3 axisA(0, 0, 1);
            btVector3 axisB(0, 0, 1);
            btVector3 pivotA(0, -(FLEG_WIDTH + FLEG_LENGTH/2), 0);
            btVector3 pivotB(0, FLEG_WIDTH + FLEG_LENGTH/2, 0);
            if(k==1){pivotA = btVector3(0, 0, 0); pivotB = btVector3(0, FLEG_WIDTH*2 + FLEG_LENGTH/2, 0);}
            btHingeConstraint* joint2 = new btHingeConstraint(*m_bodies[k+(NUM_JOINT+1)*i], *m_bodies[k+1+(NUM_JOINT+1)*i], pivotA, pivotB, axisA, axisB);
            
            //ここjoint2->setLimit(m_param.lowerlimit[(NUM_JOINT+2)*i + 1 + k], m_param.upperlimit[(NUM_JOINT+2)*i + 1 + k]);
            joint2->setLimit(-M_PI_2, M_PI_2);
            joint2->setUserConstraintId(10);
            m_joints_ankle[k-1+NUM_JOINT*i] = joint2;
            m_ownerWorld->addConstraint(m_joints_ankle[k-1+NUM_JOINT*i], true);
            JointPoint += btVector3(btScalar(fCos*(FLEG_LENGTH+2*FLEG_WIDTH)*cos(k*theta)),btScalar(-(FLEG_LENGTH+2*FLEG_WIDTH)*sin(k*theta)),btScalar(fSin*(FLEG_LENGTH+2*FLEG_WIDTH)*cos(k*theta)));
            
        }
    }
}

void Ophiuroid::create() {
    if (!hasNet) {
        m_ownerWorld->setInternalTickCallback(motorPreTickCallback, this, true);
    } else {
        m_ownerWorld->setInternalTickCallback(motorPreTickCallback_NEAT, this, true);
    }
    //activateMotor(true);
    //activateTwist(true);
    zeroFriction(false);
    
    /*if (!kCheck_first)*/ return;

    std::random_device rnd;
    std::mt19937 mt(rnd());
    std::uniform_int_distribution<> rand100(0, 99);
    
    btScalar scale[] = {btScalar(RADIUS_TF), btScalar(LENGTH)};
    btScalar dis_tf = FLEG_LENGTH/(btScalar)(NUM_TF_UNIT / 2 - 1);
    btVector3 pos_tf, pos_body;
    btTransform tA, tB, tBody, tTF_rel, tTF;
    pos_body = btVector3(0,FHEIGHT,0);
    
    int col, row, num;
    for (int i = 0; i < NUM_TF; i++) {
        col = i % 2;
        row = (i / 2) % (NUM_TF_UNIT / 2);
        num = i / NUM_TF_UNIT;
        
        tTF_rel.setIdentity();
        tTF_rel.setOrigin(btVector3(-RADIUS_TF-LENGTH/2.0, FLEG_LENGTH/2.0-dis_tf*row, pow(-1, col) * FLEG_WIDTH/2.0));
        tTF_rel.setRotation(btQuaternion(btVector3(0,0,1), -M_PI_2));
        
        for (int j = 1; j <= NUM_LEGS; j++) {
            int index = 100 + NUM_LEGS * i + j;
            int m_num = 2+num+(NUM_JOINT+1)*(j-1);
            tBody = m_bodies[m_num]->getWorldTransform();
            tTF = tBody*tTF_rel;
            
            //body
            btRigidBody* body_tf = initTubefeet(scale, tTF);
            body_tf->setUserIndex(index);
            bodies_tf.push_back(body_tf);
            TF_Contact[index] = false;
            TF_axis_direction[index] = btVector3(0, 0, 1);
            InitTime_tf[index] = 2*SECONDS*( rand100(mt)/100.0 );
            ResumeTime_ground[index] = InitTime_tf[index];
            TF_object[index] = body_tf;
            TF_object[index]->setFriction(0);
            Init_tf[index] = false;
            TF_foward[index] = true;
            //constraint
            tA.setIdentity(); tB.setIdentity();
            tA.setOrigin(tTF_rel.getOrigin()+btVector3(RADIUS_TF+LENGTH/2.0,0,0));
            tB.setOrigin(btVector3(0,RADIUS_TF+LENGTH/2.0,0));
            btGeneric6DofSpringConstraint* spring = new btGeneric6DofSpringConstraint(*m_bodies[m_num], *body_tf,tA,tB,true);
            setSpring(spring, index);
            //for next
            pos_tf = RotateY(pos_tf, M_PI*2/5);
        }
    }
    
    for (int i = 0; i < bodies_tf.size(); i++) {
        m_ownerWorld->addRigidBody(bodies_tf[i], RX_COL_TF, RX_COL_GROUND);
    }
    
    for (int i = 0; i < constraints.size(); i++) {
        m_ownerWorld->addConstraint(constraints[i]);
    }
}

btRigidBody* Ophiuroid::createRigidBody(btScalar mass, const btTransform &startTransform, btCollisionShape *shape, int index) {
    bool isDynamic = (mass != 0.f);
    
    btVector3 localInertia(0,0,0);
    if (isDynamic)
        shape->calculateLocalInertia(mass,localInertia);

    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);
    body->setUserIndex(index);
    
    m_ownerWorld->addRigidBody(body, RX_COL_BODY, RX_COL_GROUND | RX_COL_BODY);
    
    return body;
}

void Ophiuroid::setMotorTarget(double delta) {
    for (int i = 0; i<BODYPART_COUNT; i++){
        m_bodies[i]->activate(true);
    }
    
    for (int i = 0; i < NUM_LEGS; i++) {
        
        if (m_param.turn < NUM_TURN) {
            if(m_param.turn==1) swing_phase=-2;;
            if (leg_state[i] == 0) {
                
                calcMotorTarget(i);
                
                btRigidBody* rigid1 = m_bodies[(NUM_JOINT+1)*(i+1)];
                btTransform tran;
                tran.setIdentity();
                rigid1->getMotionState()->getWorldTransform(tran);
                btScalar y = tran.getOrigin().getY();
                btVector3 vY = tran*btVector3(1, 0, 0);
                btVector3 vOrigin = tran.getOrigin();
                btVector3 vY_O = vY - vOrigin;
                btScalar rot = btScalar(acos(vY_O[1])*RADIAN);
                
                if(rot<60){
                    if (y < 0.6){
                        m_param.turn += 1;
                        leg_state[i] = 1;
                        turn_direction[i] = 1;//右ねじの正
                        m_param.turn_pattern[i] = 1;
                    }
                }
            }
        } else {
            swing_phase = -1;
            int state = leg_state[(i+NUM_LEGS-1)%NUM_LEGS] + leg_state[(i+1)%NUM_LEGS];
            //支脚以外の動き//
            if (leg_state[i] == 0){
                //not支脚隣//
                if (state == 0){
                    calcMotorTarget(i);
                }
                
                //片方or両方支脚隣
                //蹴りだすor持ち上げる
                if (state == 1 || state == 2){
                    calcMotorTarget(i, 1);
                }
            }
            
            //支脚//
            if (leg_state[i] == 1){
                
            }
        }
    }
}
void Ophiuroid::setMotorTarget2_NEAT(double delta) {
    //動かなくならんように//
    for (int i = 0; i<BODYPART_COUNT; i++){
        m_bodies[i]->activate(true);
    }
    
    std::map<int, double> f;
    int i = 0;
    if (m_param.turn >= NUM_TURN) {
        double tp[NUM_LEGS];
        for (int j = 0; j < NUM_LEGS; j++) {
            tp[j] = m_param.turn_pattern[j];
        }
        m_net->load_sensors(tp);
        if (!(m_net->activate())) { m_value = -1; return; }
        for (auto itr = m_net->outputs.begin(); itr != m_net->outputs.end(); ++itr) {
            f[i] = (*itr)->activation;
            i++;
        }
    }
    
    
    //脚動き//
    for (int i=0; i<NUM_LEGS; i++){
        //第一段階（turn本数待ち）//
        if(m_param.turn < NUM_TURN){
            if(m_param.turn==1) swing_phase = -2;
            //支脚以外の動き//
            if (leg_state[i] == 0){
                
                calcMotorTarget(i);
                
                btRigidBody* rigid1 = m_bodies[(NUM_JOINT+1)*(i+1)];
                btTransform tran;
                tran.setIdentity();
                rigid1->getMotionState()->getWorldTransform(tran);
                btVector3 ttt = tran*btVector3(0,-FLEG_LENGTH/2.0,0);
                btScalar y = ttt[1];
                btVector3 vY = tran*btVector3(1, 0, 0);
                btVector3 vOrigin = tran.getOrigin();
                btVector3 vY_O = vY - vOrigin;
                btScalar rot = btScalar(acos(vY_O[1]));
                
                if(rot < THRESHOLD_ROT){
                    if (y < FLEG_WIDTH){
                        if (m_param.turn_pattern[i]==1){
                            m_param.turn += 1;
                            m_param.ee = 1;
                            leg_state[i] = 1;
                            m_param.turn_pattern[i] = 1;
                            turn_direction[i] = 1;//右ねじ正
                            rigid1->setFriction(FRICTION);
                            rigid1->setGravity(btVector3(0,-50.0,0));
                        }
                    }
                }
            }
        }
        //第二段階（turn後）//
        if (m_param.turn >= NUM_TURN){
            swing_phase = -1;
            activateMotor(true);
            //支脚以外//
            if (leg_state[i] == 0){
                
                calcMotorTarget(i, 2, f[i]);
            }
            
            //支脚
            if (leg_state[i] == 1){
                
            }
        }
    }
    
}
void Ophiuroid::setMotorTarget2(double delta) {
    //動かなくならんように//
    for (int i = 0; i<BODYPART_COUNT; i++){
        m_bodies[i]->activate(true);
    }
    
    //turn_pattern獲得後のため、センサからの入力値はわかってる。for文の外で出力値まで計算しちゃう。
    int mid[NUM_LEGS];
    int out[NUM_LEGS];
    if (m_param.turn >= NUM_TURN) {
        //中間層
        for (int i = 0; i<NUM_LEGS; i++){
            mid[i] = 0;
            for (int k = 0; k<NUM_LEGS; k++){
                mid[i] += m_param.turn_pattern[k]*m_param.conect[NUM_LEGS*i + k];
            }
            
            if(mid[i] >= 0){
                mid[i] = 1;
            }else{
                mid[i] = 0;
            }
        }
        //出力層
        for (int i = 0; i<NUM_LEGS; i++){
            out[i] = 0;
            for (int k = 0; k<NUM_LEGS; k++){
                out[i] += mid[k]*m_param.conect[NUM_LEGS*(NUM_LEGS+i) + k];
            }
        }
    }
    
    //脚動き//
    for (int i=0; i<NUM_LEGS; i++){
        //第一段階（turn本数待ち）//
        if(m_param.turn < NUM_TURN){
            if(m_param.turn==1) swing_phase = -2;
            //支脚以外の動き//
            if (leg_state[i] == 0){
                
                calcMotorTarget(i);
                
                btRigidBody* rigid1 = m_bodies[(NUM_JOINT+1)*(i+1)];
                btTransform tran;
                tran.setIdentity();
                rigid1->getMotionState()->getWorldTransform(tran);
                btScalar y = tran.getOrigin().getY();
                btVector3 vY = tran*btVector3(1, 0, 0);
                btVector3 vOrigin = tran.getOrigin();
                btVector3 vY_O = vY - vOrigin;
                btScalar rot = btScalar(acos(vY_O[1]));
                
                if(rot < THRESHOLD_ROT){
                    if (y < FLEG_WIDTH){
                        if (m_param.turn_pattern[i]==1){
                            m_param.turn += 1;
                            m_param.ee = 1;
                            leg_state[i] = 1;
                            m_param.turn_pattern[i] = 1;
                            turn_direction[i] = 1;//右ねじ正
                            rigid1->setFriction(FRICTION);
                            rigid1->setGravity(btVector3(0,-15.0,0));
                        }
                    }
                }
            }
        }
        //第二段階（turn後）//
        if (m_param.turn >= NUM_TURN){
            swing_phase = -1;
            activateMotor(true);
            //支脚以外//
            if (leg_state[i] == 0){
                float a = m_param.a[i];//シグモイド関数のパラメータ
                float f = 1/(1+exp(-a*out[i]));//出力層からの出力値（シグモイド関数[0,1]）
                calcMotorTarget(i, 2, f);
            }
            
            //支脚
            if (leg_state[i] == 1){
                
            }
        }
    }
    
}

void Ophiuroid::calcMotorTarget(int i, int sW, float f) {
    btUniversalConstraint* hinge2 = m_joints_hip[i];
    //handle
    btScalar fCurAngle1 = hinge2->getAngle1();
    btScalar fTargetPercent_hip1 = m_param.targetpercent[(NUM_JOINT + 2)*i] + (int(m_time_step ) % int(m_param.cycle)) / m_param.cycle;
    btScalar fTargetAngle_hip1 = 0.5 * (1 + sin(2* M_PI * fTargetPercent_hip1));
    btScalar fTargetLimitAngle1;
    switch (sW) {
        case 2: fTargetLimitAngle1 = (m_param.swing[i] - 4) * M_PI_4; break;
        default: fTargetLimitAngle1 = m_param.lowerlimit[(NUM_JOINT + 2)*i] + fTargetAngle_hip1 * (m_param.upperlimit[(NUM_JOINT + 2)*i] - m_param.lowerlimit[(NUM_JOINT + 2)*i]); break;
    }
    if (!check) fTargetLimitAngle1=0;
    btScalar fAngleError1;
    switch (sW) {
        case 1: fAngleError1 = -fCurAngle1; break;
        default: fAngleError1 = fTargetLimitAngle1 - fCurAngle1; break;
    }
    btScalar fDesiredAngularVel1 = fAngleError1*6;//*FPS;
    m_motor1[i]->m_targetVelocity = fDesiredAngularVel1;
    
    //wheel
    btScalar fCurAngle2 = hinge2->getAngle2();
    btScalar fTargetPercent_hip2 = m_param.targetpercent[(NUM_JOINT + 2)*i + 1] + (int(m_time_step ) % int(m_param.cycle)) / m_param.cycle;
    btScalar fTargetAngle_hip2 = 0.5 * (1 + sin(2* M_PI * fTargetPercent_hip2));
    btScalar fTargetLimitAngle2;
    switch (sW) {
        case 1: fTargetLimitAngle2 = -M_PI_2 + fTargetAngle_hip2 * M_PI; break;
        case 2: fTargetLimitAngle2 = fTargetAngle_hip2 * SWING_ANGLE; break;
        default: fTargetLimitAngle2 = m_param.lowerlimit[(NUM_JOINT + 2)*i + 1] + fTargetAngle_hip2 * (m_param.upperlimit[(NUM_JOINT + 2)*i +1] - m_param.lowerlimit[(NUM_JOINT + 2)*i +1]); break;
    }
    if (!check) fTargetLimitAngle2=0;
    btScalar fAngleError2 = fTargetLimitAngle2  - fCurAngle2;
    btScalar fDesiredAngularVel2 = fAngleError2*6;//*FPS;
    switch (sW) {
        case 2: m_motor2[i]->m_targetVelocity = fDesiredAngularVel2 * f; break;
        default:m_motor2[i]->m_targetVelocity = fDesiredAngularVel2; break;
    }
    //ankle
    for (int k = 1; k<=NUM_JOINT; k++){
        btHingeConstraint* hingeC2 = m_joints_ankle[k-1+NUM_JOINT*i];
        btScalar fCurAngle_ankle = hingeC2->getHingeAngle();
        btScalar fTargetPercent_ankle;
        switch (sW) {
            case 1: fTargetPercent_ankle = m_param.targetpercent[(NUM_JOINT + 2)*i + 1] + (int(m_time_step) % int(m_param.cycle)) / m_param.cycle; break;
            case 2: fTargetPercent_ankle = m_param.targetpercent[(NUM_JOINT + 2)*i + 1] + (int(m_time_step) % int(m_param.cycle*3)) / (m_param.cycle*3.0); break;
            default: fTargetPercent_ankle = m_param.targetpercent[(NUM_JOINT + 2)*i + 1 + k] + (int(m_time_step) % int(m_param.cycle)) / m_param.cycle; break;
        }
        btScalar fTargetAngle_ankle = 0.5 * (1 + sin(2 * M_PI * fTargetPercent_ankle));
        btScalar fTargetLimitAngle_ankle;
        switch (sW) {
            case 1: fTargetLimitAngle_ankle = -M_PI_4 + fTargetAngle_ankle * M_PI_4; break;
            case 2: fTargetLimitAngle_ankle = fTargetAngle_ankle * M_PI_4; break;
            default: fTargetLimitAngle_ankle = m_param.lowerlimit[(NUM_JOINT + 2)*i + 1 + k] + fTargetAngle_ankle * (m_param.upperlimit[(NUM_JOINT + 2)*i + 1 + k] - m_param.lowerlimit[(NUM_JOINT + 2)*i + 1 + k]); break;
        }
        if (!check) fTargetLimitAngle_ankle=0;
        btScalar fAngleError_ankle  = fTargetLimitAngle_ankle - fCurAngle_ankle;
        btScalar fDesiredAngularVel_ankle = fAngleError_ankle*6;
        switch (sW) {
            case 2: hingeC2->enableAngularMotor(true, fDesiredAngularVel_ankle * f, MAX_MOTOR_TORQUE); break;
            default: hingeC2->enableAngularMotor(true, fDesiredAngularVel_ankle, MAX_MOTOR_TORQUE); break;
        }
    }
}

btRigidBody* Ophiuroid::initTubefeet(btScalar* scale, const btTransform &startTransform)
{
    btCollisionShape* sBodyShape = new btCapsuleShape(scale[0], scale[1]);
    
    btScalar mass1(M_TF);
    bool isDynamic = (mass1 != 0.f);
    
    btVector3 localInertia1(0, 0, 0);
    if (isDynamic)
        sBodyShape->calculateLocalInertia(mass1, localInertia1);
    
    btDefaultMotionState* myMotionState1 = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass1, myMotionState1, sBodyShape, localInertia1);
    btRigidBody* body = new btRigidBody(rbInfo);
    
    return body;
}

void Ophiuroid::setSpring(btGeneric6DofSpringConstraint* spring, int index) {
    spring->enableSpring(0, true);
    spring->enableSpring(1, true);
    spring->setDamping(0, 10.f);
    spring->setDamping(1, 10.f);
    spring->setStiffness(0, 10.f);
    spring->setStiffness(1, 10.f);
    spring->setLinearLowerLimit(btVector3(-2,-2,0));
    spring->setLinearUpperLimit(btVector3(2,2,0));
    spring->setAngularLowerLimit(btVector3(0,-M_PI_2,M_PI_2+m_param.lowerlimit2[index-101]));
    spring->setAngularUpperLimit(btVector3(0,M_PI_2,M_PI_2+m_param.upperlimit2[index-101]));
    TF_constraint[index] = spring;
    constraints.push_back(spring);
    //motor
    btRotationalLimitMotor* motorRotX = spring->getRotationalLimitMotor(0);
    btRotationalLimitMotor* motorRotY = spring->getRotationalLimitMotor(1);
    btRotationalLimitMotor* motorRotZ = spring->getRotationalLimitMotor(2);
    motorRotX->m_enableMotor = true;
    motorRotX->m_targetVelocity = 0;
    motorRotX->m_maxMotorForce = 100000000;
    motorRotY->m_enableMotor = true;
    motorRotY->m_targetVelocity = 0;
    motorRotY->m_maxMotorForce = 100000000;
    motorRotZ->m_enableMotor = true;
    motorRotZ->m_targetVelocity = 0;
    motorRotZ->m_maxMotorForce = 100000000;
    motor_tX[index] = motorRotX;
    motor_tY[index] = motorRotY;
    motor_tZ[index] = motorRotZ;
    motor_state[index] = true;
}

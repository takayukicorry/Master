//
//  Ophiuroid.cpp
//  Master
//
//  Created by 増田貴行 on 2017/10/31.
//  Copyright © 2017年 増田貴行. All rights reserved.
//

#include "Ophiuroid.hpp"

Ophiuroid::Ophiuroid(GAparameter p) {
    /************  create() で変数は初期化される  **************/
    for (int i = 0; i < NUM_LEGS; i++) {
        leg_state[i] = 0;
    }
    m_param = p;
}

Ophiuroid::Ophiuroid(GAparameter p, Starfish* sf) {
    /************  create() で変数は初期化される  **************/
    for (int i = 0; i < NUM_LEGS; i++) {
        leg_state[i] = 0;
    }
    m_param = p;
    
    m_bodies = sf->m_bodies;
    m_shapes = sf->m_shapes;
    m_joints_hip = sf->m_joints_hip;
    m_joints_ankle = sf->m_joints_ankle;
    m_motor1 = sf->m_motor1;
    m_motor2 = sf->m_motor2;
}

float Ophiuroid::evalue() {
    btDiscreteDynamicsWorld* dynamicsWorld = GAMaster::createWorld();
    dynamicsWorld->setGravity(btVector3(0, -10, 0));
    setWorld(dynamicsWorld);

    GAMaster::createGround(dynamicsWorld);
    initSF();
    create();
    for (int i = 0; i < SIMULATION_TIME_STEP; i++) {
        dynamicsWorld->stepSimulation(1.f / FPS);
        
        /***********************************/
        /*******   なんかしらする  ************/
        /***********************************/
    }
    
    btTransform tr = m_bodies[0]->getWorldTransform();
    btVector3 vY = tr*btVector3(0, 1, 0);
    btVector3 vOrigin = tr.getOrigin();
    btVector3 vY_O = vY - vOrigin;
    
    GAMaster::cleanupWorld(dynamicsWorld);
    return -vY_O[1];
}

void Ophiuroid::idle() {
    /***********    turn_pattern獲得前後でここ違う   ***************/
    setMotorTarget(1);
    glutPostRedisplay();
    
}

bool Ophiuroid::checkState() {
    btTransform tr = m_bodies[0]->getWorldTransform();
    btVector3 vY = tr*btVector3(0, 1, 0);
    btVector3 vOrigin = tr.getOrigin();
    
    btVector3 vY_O = vY - vOrigin;

    return vY_O[1] > sin(2*M_PI/5);
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
    btTransform transform, transformY, transformS, transformSY;
    transform.setIdentity();
    transform.setOrigin(vRoot);
    m_bodies[0] = createRigidBody(btScalar(M_OBJ), transform, m_shapes[0], 10);
    
    // legs
    for ( i=0; i<NUM_LEGS; i++)
    {
        float fAngle = 2 * M_PI * i / NUM_LEGS;
        float fSin = sin(fAngle);//左ねじの方向に正
        float fCos = cos(fAngle);
        
        btVector3 vBoneOrigin = btVector3(btScalar(fCos*(FBODY_SIZE + FLEG_WIDTH + (0.5*FLEG_LENGTH+2*FLEG_WIDTH)*cos(theta))), btScalar(FHEIGHT - (0.5*FLEG_LENGTH+2*FLEG_WIDTH)*sin(theta)), btScalar(fSin*(FBODY_SIZE + FLEG_WIDTH + (0.5*FLEG_LENGTH+2*FLEG_WIDTH)*cos(theta))));
        btVector3 Point = vBoneOrigin;
        btVector3 spherePoint = btVector3(btScalar(fCos*(FBODY_SIZE + FLEG_WIDTH)), btScalar(FHEIGHT), btScalar(fSin*(FBODY_SIZE + FLEG_WIDTH)));
        
        transformS.setIdentity();
        transformS.setOrigin(spherePoint);
        transformS.setRotation(btQuaternion(btVector3(0, 1, 0), -fAngle));//右ねじの方向に正
        transformSY.setIdentity();
        transformSY.setRotation(btQuaternion(btVector3(0, 0, 1), M_PI_2));
        
        m_bodies[1+(NUM_JOINT+1)*i] = createRigidBody(btScalar(M_OBJ), transformS*transformSY, m_shapes[1+(NUM_JOINT+1)*i], 10+1+(NUM_JOINT+1)*i);
        
        for (int k = 1; k <= NUM_JOINT; k++)
        {
            transform.setIdentity();
            transform.setOrigin(Point);
            transform.setRotation(btQuaternion(btVector3(0, 1, 0), -fAngle));
            transformY.setIdentity();
            transformY.setRotation(btQuaternion(btVector3(0, 0, 1), M_PI_2 - theta * k));
            
            m_bodies[k+1+(NUM_JOINT+1)*i] = createRigidBody(btScalar(M_OBJ), transform*transformY, m_shapes[k+1+(NUM_JOINT+1)*i], 10+k+1+(NUM_JOINT+1)*i);
            Point += btVector3(btScalar(fCos*(0.5*FLEG_LENGTH+FLEG_WIDTH)*cos(k*theta)),btScalar(-(0.5*FLEG_LENGTH+FLEG_WIDTH)*sin(k*theta)),btScalar(fSin*(0.5*FLEG_LENGTH+FLEG_WIDTH)*cos(k*theta))) + btVector3(btScalar(fCos*(0.5*FLEG_LENGTH+FLEG_WIDTH)*cos((k+1)*theta)),btScalar(-(0.5*FLEG_LENGTH+FLEG_WIDTH)*sin((k+1)*theta)),btScalar(fSin*(0.5*FLEG_LENGTH+FLEG_WIDTH)*cos((k+1)*theta)));
        }
        m_bodies[(NUM_JOINT+1)*(i+1)]->setFriction(5.0);
    }
    
    //停止が続いてもsleeping状態にならないようにする//
    // Setup some damping on the m_bodies
    
    for (i = 0; i < m_bodies.size(); ++i)
    {
        m_bodies[i]->setDamping(0.05, 0.85);
        m_bodies[i]->setDeactivationTime(1000000000000.f);
        m_bodies[i]->setSleepingThresholds(1.6, 2.5);
        m_bodies[i]->setSleepingThresholds(0.5f, 0.5f);
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
        hinge2C->setLowerLimit(-M_PI_2, -M_PI_2);//(wheel, handle)右ねじ
        hinge2C->setUpperLimit(M_PI_2, M_PI_2);
        hinge2C->setUserConstraintId(10);
        m_joints_hip[i] = hinge2C;
        m_ownerWorld->addConstraint(m_joints_hip[i], true);
        
        const int AXIS1_ID = 2;
        const int AXIS2_ID = 1;
        const float MAX_MOTOR_TORQUE = 10000000000;//出力[W] ＝ ( 2 * M_PI / 60 ) × T[N・m] × θ[rad/min]
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
            
            //****************joint2->setLimit(m_param.lowerlimit[(NUM_JOINT+2)*i + 1 + k], m_param.upperlimit[(NUM_JOINT+2)*i + 1 + k]);
            joint2->setUserConstraintId(10);
            m_joints_ankle[k-1+NUM_JOINT*i] = joint2;
            m_ownerWorld->addConstraint(m_joints_ankle[k-1+NUM_JOINT*i], true);
            JointPoint += btVector3(btScalar(fCos*(FLEG_LENGTH+2*FLEG_WIDTH)*cos(k*theta)),btScalar(-(FLEG_LENGTH+2*FLEG_WIDTH)*sin(k*theta)),btScalar(fSin*(FLEG_LENGTH+2*FLEG_WIDTH)*cos(k*theta)));
            
        }
    }
}

void Ophiuroid::create() {
    activateMotor(true);
    activateTwist(true);
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
            if (leg_state[i] == 0) {
                
                calcMotorTarget(i);
                
                btRigidBody* rigid1 = m_bodies[(NUM_JOINT+1)*(i+1)];
                btTransform tran;
                tran.setIdentity();
                rigid1->getMotionState()->getWorldTransform(tran);
                btScalar y = tran.getOrigin().getY();
                btVector3 pos = rigid1->getCenterOfMassPosition();
                btScalar rot = btScalar(rigid1->getOrientation().getAngle()*RADIAN);
                
                if(rot>120){
                    if (y < 0.6){
                        m_param.turn += 1;
                        leg_state[i] = 1;
                        turn_direction[i] = 1;//右ねじの正
                        m_param.turn_pattern[i] = 1;
                    }
                }
                if(rot<-120){
                    if (y < 0.6){
                        m_param.turn += 1;
                        leg_state[i] = 1;
                        turn_direction[i] = -1;//右ねじの負
                        m_param.turn_pattern[i] = 1;
                    }
                }
            }
        }
        //turn済//
        int state = leg_state[(i+NUM_LEGS-1)%NUM_LEGS] + leg_state[(i+1)%NUM_LEGS];
        
        if (m_param.turn >= NUM_TURN){
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

void Ophiuroid::setMotorTarget2(double delta) {
    //動かなくならんように//
    for (int i = 0; i<BODYPART_COUNT; i++){
        m_bodies[i]->activate(true);
    }
    
    //turn_pattern獲得後のため、センサからの入力値はわかってる。for文の外で出力値まで計算しちゃう。
    int mid[NUM_LEGS];
    int out[NUM_LEGS];
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
    
    //脚動き//
    for (int i=0; i<NUM_LEGS; i++){
        //第一段階（turn本数待ち）//
        if(m_param.turn < NUM_TURN){
            //支脚以外の動き//
            if (leg_state[i] == 0){
                
                calcMotorTarget(i);
                
                //turn判定//
                btRigidBody* rigid1 = m_bodies[(NUM_JOINT+1)*(i+1)];
                btTransform tran;
                tran.setIdentity();
                rigid1->getMotionState()->getWorldTransform(tran);
                btScalar y = tran.getOrigin().getY();
                btTransform tr = rigid1->getWorldTransform();
                btVector3 vY = tr*btVector3(0,1,0);
                btVector3 v_origin = tr.getOrigin();
                
                btVector3 vY1 = vY - v_origin;
                float vY2[3] = {vY1[0],0,vY1[2]};
                float mv;
                if (vY1[1] >= 0)
                {
                    mv = M_PI_2 - acos((vY1[0]*vY2[0]+vY1[1]*vY2[1]+vY1[2]*vY2[2])/sqrtf((vY1[0]*vY1[0]+vY1[1]*vY1[1]+vY1[2]*vY1[2])*(vY2[0]*vY2[0]+vY2[1]*vY2[1]+vY2[2]*vY2[2])));
                    
                }else
                {
                    mv = M_PI_2 + acos((vY1[0]*vY2[0]+vY1[1]*vY2[1]+vY1[2]*vY2[2])/sqrtf((vY1[0]*vY1[0]+vY1[1]*vY1[1]+vY1[2]*vY1[2])*(vY2[0]*vY2[0]+vY2[1]*vY2[1]+vY2[2]*vY2[2])));
                    
                }
                if(mv > THRESHOLD){
                    if (y < 0.6){
                        if (m_param.turn_pattern[i]==1){
                            m_param.turn += 1;
                            m_param.ee = 1;
                            leg_state[i] = 1;
                            turn_direction[i] = 1;//右ねじ正
                            rigid1->setFriction(FRICTION);
                            rigid1->setGravity(btVector3(0,-15.0,0));
                        }
                    }
                }
                if(mv < -THRESHOLD){
                    if (y < 0.6){
                        if (m_param.turn_pattern[i]==1){
                            m_param.turn += 1;
                            m_param.ee = 1;
                            leg_state[i] = 1;
                            turn_direction[i] = -1;//右ねじ負
                            rigid1->setFriction(FRICTION);
                            rigid1->setGravity(btVector3(0,-15.0,0));
                            
                        }
                    }
                }
            }
            
        }
        //第二段階（turn後）//
        
        if (m_param.turn >= NUM_TURN){
            
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
    btScalar fTargetPercent_hip1 = m_param.targetpercent[(NUM_JOINT + 2)*i] + (int(Master::time_step ) % int(m_param.cycle)) / m_param.cycle;
    btScalar fTargetAngle_hip1 = 0.5 * (1 + sin(2* M_PI * fTargetPercent_hip1));
    btScalar fTargetLimitAngle1;
    switch (sW) {
        case 2: fTargetLimitAngle1 = (m_param.swing[i] - 4) * M_PI_4; break;
        default: fTargetLimitAngle1 = m_param.lowerlimit[(NUM_JOINT + 2)*i] + fTargetAngle_hip1 * (m_param.upperlimit[(NUM_JOINT + 2)*i] - m_param.lowerlimit[(NUM_JOINT + 2)*i]); break;
    }
    btScalar fAngleError1;
    switch (sW) {
        case 1: fAngleError1 = -fCurAngle1; break;
        default: fAngleError1 = fTargetLimitAngle1 - fCurAngle1; break;
    }
    btScalar fDesiredAngularVel1 = fAngleError1*FPS/10;
    m_motor1[i]->m_targetVelocity = fDesiredAngularVel1;
    
    //wheel
    btScalar fCurAngle2 = hinge2->getAngle2();
    btScalar fTargetPercent_hip2 = m_param.targetpercent[(NUM_JOINT + 2)*i + 1] + (int(Master::time_step ) % int(m_param.cycle)) / m_param.cycle;
    btScalar fTargetAngle_hip2 = 0.5 * (1 + sin(2* M_PI * fTargetPercent_hip2));
    btScalar fTargetLimitAngle2;
    switch (sW) {
        case 1: fTargetLimitAngle2 = -M_PI_2 + fTargetAngle_hip2 * M_PI; break;
        case 2: fTargetLimitAngle2 = fTargetAngle_hip2 * SWING_ANGLE; break;
        default: fTargetLimitAngle2 = m_param.lowerlimit[(NUM_JOINT + 2)*i + 1] + fTargetAngle_hip2 * (m_param.upperlimit[(NUM_JOINT + 2)*i +1] - m_param.lowerlimit[(NUM_JOINT + 2)*i +1]); break;
    }
    btScalar fAngleError2 = fTargetLimitAngle2  - fCurAngle2;
    btScalar fDesiredAngularVel2 = fAngleError2*FPS/10;
    switch (sW) {
        case 2: m_motor2[i]->m_targetVelocity = fDesiredAngularVel2 * f; break;
        default:m_motor2[i]->m_targetVelocity = fDesiredAngularVel2; break;
    }
    
    for (int k = 1; k<=NUM_JOINT; k++){
        btHingeConstraint* hingeC2 = m_joints_ankle[k-1+NUM_JOINT*i];
        btScalar fCurAngle_ankle = hingeC2->getHingeAngle();
        btScalar fTargetPercent_ankle;
        switch (sW) {
            case 1: fTargetPercent_ankle = m_param.targetpercent[(NUM_JOINT + 2)*i + 1] + (int(Master::time_step) % int(m_param.cycle)) / m_param.cycle; break;
            case 2: fTargetPercent_ankle = m_param.targetpercent[(NUM_JOINT + 2)*i + 1] + (int(Master::time_step) % int(m_param.cycle*3)) / (m_param.cycle*3.0); break;
            default: fTargetPercent_ankle = m_param.targetpercent[(NUM_JOINT + 2)*i + 1 + k] + (int(Master::time_step) % int(m_param.cycle)) / m_param.cycle; break;
        }
        btScalar fTargetAngle_ankle = 0.5 * (1 + sin(2 * M_PI * fTargetPercent_ankle));
        btScalar fTargetLimitAngle_ankle;
        switch (sW) {
            case 1: fTargetLimitAngle_ankle = -M_PI_4 + fTargetAngle_ankle * M_PI_4; break;
            case 2: fTargetLimitAngle_ankle = fTargetAngle_ankle * M_PI_4; break;
            default: fTargetLimitAngle_ankle = m_param.lowerlimit[(NUM_JOINT + 2)*i + 1 + k] + fTargetAngle_ankle * (m_param.upperlimit[(NUM_JOINT + 2)*i + 1 + k] - m_param.lowerlimit[(NUM_JOINT + 2)*i + 1 + k]); break;
        }
        btScalar fAngleError_ankle  = fTargetLimitAngle_ankle - fCurAngle_ankle;
        btScalar fDesiredAngularVel_ankle = fAngleError_ankle;
        switch (sW) {
            case 2: hingeC2->enableAngularMotor(true, fDesiredAngularVel_ankle * f, 100000000000); break;
            default: hingeC2->enableAngularMotor(true, fDesiredAngularVel_ankle, 1000000000000); break;
        }
    }
}

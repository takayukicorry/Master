//
//  Ophiuroid.cpp
//  Master
//
//  Created by 増田貴行 on 2017/10/31.
//  Copyright © 2017年 増田貴行. All rights reserved.
//

#include "Ophiuroid.hpp"

Ophiuroid::Ophiuroid(GAparameter p) {
    ///////////////////////////////////
    ///  create() で変数は初期化される  ///
    ///////////////////////////////////
    for (int i = 0; i < NUM_LEGS; i++) {
        leg_state[i] = 0;
    }
    m_param = p;
}

float Ophiuroid::evalue() {
    return 0;
}

float Ophiuroid::evalue2() {
    return 0;
}

void Ophiuroid::idle() {
    /***********    turn_pattern獲得前後でここ違う   ***************/
    setMotorTarget(1);
    glutPostRedisplay();
}

bool Ophiuroid::checkState() {
    return true;
}

void Ophiuroid::create() {
    //
    // Setup geometry
    //
    float fBodySize = 5;
    float fLegLength = 15/NUM_JOINT;
    float fLegWidth = 3;

    float fHeight = 15;
    float alpha = 37*M_PI/36;//thighとshinが何度開いてるか初期値
    float theta = M_PI - alpha;
    GAmanager manager;;
    
    m_shapes[0] = new btCylinderShape(btVector3(fBodySize,fLegWidth,fBodySize));
    
    int i;
    
    for ( i=0; i<NUM_LEGS; i++)
    {
        m_shapes[1 + (NUM_JOINT+1)*i] = new btSphereShape(fLegWidth);
        for (int k = 1; k <= NUM_JOINT; k++)
            
        {
            m_shapes[k + 1 + (NUM_JOINT+1)*i] = new btCapsuleShape(btScalar(fLegWidth), btScalar(fLegLength));
        }
    }

    //
    // Setup rigid bodies
    //
    btTransform offset;
    offset.setIdentity();
    offset.setOrigin(btVector3(0,10,0));
    
    // root
    btVector3 vRoot = btVector3(btScalar(0.), btScalar(fHeight), btScalar(0.));
    btTransform transform;
    btTransform transformS;
    btTransform transformY;
    transform.setIdentity();
    transform.setOrigin(vRoot);
    m_bodies[0] = createRigidBody(btScalar(0.5), offset*transform, m_shapes[0]);
    
    // legs
    for ( i=0; i<NUM_LEGS; i++)
    {
        float fAngle = 2 * M_PI * i / NUM_LEGS;
        float fSin = sin(fAngle);
        float fCos = cos(fAngle);
        
        
        btVector3 vBoneOrigin = btVector3(btScalar(fCos*(fBodySize + fLegWidth + (0.5*fLegLength+2*fLegWidth)*cos(theta))), btScalar(fHeight + (0.5*fLegLength+2*fLegWidth)*sin(theta)), btScalar(fSin*(fBodySize + fLegWidth + (0.5*fLegLength+2*fLegWidth)*cos(theta))));
        btVector3 vToBone = (vBoneOrigin - vRoot).normalize();
        btVector3 vAxis = vToBone.cross(btVector3(0,1,0));
        btVector3 Point = vBoneOrigin;
        btVector3 spherePoint = btVector3(btScalar(fCos*(fBodySize + fLegWidth )), btScalar(fHeight), btScalar(fSin*(fBodySize + fLegWidth)));
        
        
        transformY.setIdentity();
        transformY.setOrigin(btVector3(0,0,0));
        transformY.setRotation(btQuaternion(btVector3(0,1,0), -fAngle));
        
        transformS.setIdentity();
        transformS.setOrigin(spherePoint);
        
        m_bodies[1+(NUM_JOINT+1)*i] = createRigidBody(btScalar(0.5), offset*transformS, m_shapes[1+(NUM_JOINT+1)*i]);
        
        for (int k = 1; k <= NUM_JOINT; k++)
        {            
            transform.setIdentity();
            transform.setOrigin(Point);
            transform.setRotation(btQuaternion(vAxis, M_PI_2 - theta * k));//垂直から下側が何度上がって行くか
            
            m_bodies[k+1+(NUM_JOINT+1)*i] = createRigidBody(btScalar(0.5), offset*transform/*transformY*/, m_shapes[k+1+(NUM_JOINT+1)*i]);//軸だけ回転してて、剛体の初期ポジが横向き疑惑→解決
            Point += btVector3(btScalar(fCos*(0.5*fLegLength+fLegWidth)*cos(k*theta)),btScalar(-(0.5*fLegLength+fLegWidth)*sin(k*theta)),btScalar(fSin*(0.5*fLegLength+fLegWidth)*cos(k*theta))) + btVector3(btScalar(fCos*(0.5*fLegLength+fLegWidth)*cos((k+1)*theta)),btScalar(-(0.5*fLegLength+fLegWidth)*sin((k+1)*theta)),btScalar(fSin*(0.5*fLegLength+fLegWidth)*cos((k+1)*theta)));
        }
        m_bodies[(NUM_JOINT+1)*(i+1)]->setFriction(5.0);//摩擦
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
    /**/
    
    //
    // Setup the constraints
    //
    btHingeConstraint* joint2;//第1リンクと第2リンクの拘束
    btUniversalConstraint* hinge2C;
    
    btTransform localA, localB, localC;
    
    for ( i=0; i<NUM_LEGS; i++)
    {
        float fAngle = 2 * M_PI * i / NUM_LEGS;
        float fSin = sin(fAngle);
        float fCos = cos(fAngle);
        
        /*hinge2で作ってみよう*//*未完成*/
        
        // hip joints
        btVector3 parentAxis( fCos, 0, fSin);//第一ジョイント回転軸１（ワールド座標）-->ローカルz軸
        btVector3 childAxis( fSin, 0, -fCos);//第一ジョイント回転軸２（ワールド座標）-->ローカルy軸
        btVector3 anchor( fCos*fBodySize, 10+fHeight, fSin*fBodySize);//上の２軸の交点（ワールド座標）
        
        hinge2C = new btUniversalConstraint(*m_bodies[0], *m_bodies[1+(NUM_JOINT+1)*i], anchor, parentAxis, childAxis);
        hinge2C->setLinearLowerLimit(btVector3(0,0,0));
        hinge2C->setLinearUpperLimit(btVector3(0,0,0));
        hinge2C->setLowerLimit(-M_PI_2,-M_PI_2);
        hinge2C->setUpperLimit(M_PI_2,M_PI_2);
        m_joints[(NUM_JOINT+1)*i] = hinge2C;
        Master::dynamicsWorld->addConstraint(m_joints[(NUM_JOINT+1)*i]);
        
        
        const int AXIS1_ID = 2;
        const int AXIS2_ID = 1;
        const float MAX_MOTOR_TORQUE = 10000000000;
        m_motor1[i] = hinge2C->getRotationalLimitMotor(AXIS1_ID);
        m_motor2[i] = hinge2C->getRotationalLimitMotor(AXIS2_ID);
        m_motor1[i]->m_maxMotorForce = MAX_MOTOR_TORQUE;
        m_motor2[i]->m_maxMotorForce = MAX_MOTOR_TORQUE;
        
        m_motor1[i]->m_enableMotor = true;
        m_motor2[i]->m_enableMotor = true;
        
        btVector3 JointPoint = btVector3(btScalar(fCos*(fBodySize+2*fLegWidth)), btScalar(0.), btScalar(fSin*(fBodySize+2*fLegWidth)));
        
        
        for (int k = 1; k <= NUM_JOINT; k++)
        {
            localA.setIdentity(); localB.setIdentity(); localC.setIdentity();
            localA.getBasis().setEulerZYX(0,-fAngle,0);
            localA.setOrigin(JointPoint);
            localB = m_bodies[k+(NUM_JOINT+1)*i]->getWorldTransform().inverse() * m_bodies[0]->getWorldTransform() * localA;
            localC = m_bodies[k+1+(NUM_JOINT+1)*i]->getWorldTransform().inverse() * m_bodies[0]->getWorldTransform() * localA;
            joint2 = new btHingeConstraint(*m_bodies[k+(NUM_JOINT+1)*i], *m_bodies[k+1+(NUM_JOINT+1)*i], localB, localC);
            joint2->setLimit(manager.pool[0].lowerlimit[(NUM_JOINT+2)*i + 1 + k], manager.pool[0].upperlimit[(NUM_JOINT+2)*i + 1 + k]);
            m_joints[k+(NUM_JOINT+1)*i] = joint2;
            Master::dynamicsWorld->addConstraint(m_joints[k+(NUM_JOINT+1)*i]);
            JointPoint += btVector3(btScalar(fCos*(fLegLength+2*fLegWidth)*cos(k*theta)),btScalar(-(fLegLength+2*fLegWidth)*sin(k*theta)),btScalar(fSin*(fLegLength+2*fLegWidth)*cos(k*theta)));
            
        }
    }
}

btRigidBody* Ophiuroid::createRigidBody(btScalar mass, const btTransform &startTransform, btCollisionShape *shape) {
    bool isDynamic = (mass != 0.f);
    
    btVector3 localInertia(0,0,0);
    if (isDynamic)
        shape->calculateLocalInertia(mass,localInertia);
    
    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);
    
    Master::dynamicsWorld->addRigidBody(body, RX_COL_BODY, RX_COL_GROUND);
    
    return body;
}

void Ophiuroid::setMotorTarget(double delta) {
    for (int i = 0; i<BODYPART_COUNT; i++){
        btRigidBody* rigid1 = static_cast<btRigidBody*>(m_bodies[i]);
        rigid1->activate(true);
    }
    
    for (int i = 0; i < NUM_LEGS; i++) {
        btRotationalLimitMotor* motor1 = static_cast<btRotationalLimitMotor*>(m_motor1[i]);
        btRotationalLimitMotor* motor2 = static_cast<btRotationalLimitMotor*>(m_motor2[i]);
        //btRotationalLimitMotor* motor3 = static_cast<btRotationalLimitMotor*>(m_motor1[(i+NUM_LEGS-1)%NUM_LEGS]);
        //btRotationalLimitMotor* motor4 = static_cast<btRotationalLimitMotor*>(m_motor1[(i+1)%NUM_LEGS]);
        btUniversalConstraint* hinge2 = static_cast<btUniversalConstraint*>(m_joints[(NUM_JOINT+1)*i]);

        int state = leg_state[(i+NUM_LEGS-1)%NUM_LEGS] + leg_state[(i+1)%NUM_LEGS];
        
        if (m_param.turn < NUM_TURN) {
            if (leg_state[i] == 0) {
                btScalar fCurAngle1  = hinge2->getAngle1();//現在のaxis1（ツイスト）の角度
                btScalar fCurAngle2  = hinge2->getAngle2();//現在のaxis2（車輪）の角度
                
                btScalar fTargetPercent_hip1 = m_param.targetpercent[(NUM_JOINT + 2)*i] + (int(Master::time_step ) % int(m_param.cycle)) / m_param.cycle;//周期に対する現時刻の割合（往復を1とした時の値）
                btScalar fTargetPercent_hip2 = m_param.targetpercent[(NUM_JOINT + 2)*i + 1] + (int(Master::time_step ) % int(m_param.cycle)) / m_param.cycle;//m_time/1000の/1000はなくしちゃダメ
                btScalar fTargetAngle_hip1   = 0.5 * (1 + sin(2* M_PI * fTargetPercent_hip1));//足の動きを波の高さ0.5、中心0.5、周期２PIの波としたときに、現時刻でどこにあるべきか
                btScalar fTargetAngle_hip2   = 0.5 * (1 + sin(2* M_PI * fTargetPercent_hip2));//足の動きを波の高さ0.5、中心0.5、周期２PIの波としたときに、現時刻でどこにあるべきか
                btScalar fTargetLimitAngle1 = m_param.lowerlimit[(NUM_JOINT + 2)*i] + fTargetAngle_hip1 * (m_param.upperlimit[(NUM_JOINT + 2)*i] - m_param.lowerlimit[(NUM_JOINT + 2)*i]);//目標角度１
                btScalar fTargetLimitAngle2 = m_param.lowerlimit[(NUM_JOINT + 2)*i + 1] + fTargetAngle_hip2 * (m_param.upperlimit[(NUM_JOINT + 2)*i +1] - m_param.lowerlimit[(NUM_JOINT + 2)*i +1]);//目標角度２
                
                btScalar fAngleError1   = fTargetLimitAngle1  - fCurAngle1;//動くべき角度１
                btScalar fAngleError2   = fTargetLimitAngle2  - fCurAngle2;//動くべき角度２
                btScalar fDesiredAngularVel1 = fAngleError1/10;//角速度１
                btScalar fDesiredAngularVel2 = fAngleError2/10;//角速度２
                
                motor1->m_targetVelocity = fDesiredAngularVel1;
                motor2->m_targetVelocity = fDesiredAngularVel2;
                
                for (int k = 1; k<=NUM_JOINT; k++){
                    
                    btHingeConstraint* hingeC2 = static_cast<btHingeConstraint*>(m_joints[k+(NUM_JOINT+1)*i]);//ここの[]内を変えた
                    btScalar fCurAngle_ankle   = hingeC2->getHingeAngle();//現在のankleの角度
                    
                    btScalar fTargetPercent_ankle = m_param.targetpercent[(NUM_JOINT + 2)*i + 1 + k] + (int(Master::time_step) % int(m_param.cycle)) / m_param.cycle;//周期に対する現時刻の割合（往復を1とした時の値）
                    btScalar fTargetAngle_ankle   = 0.5 * (1 + sin(2 * M_PI * fTargetPercent_ankle));//足の動きを波の高さ0.5、中心0.5、周期２PIの波としたときに、現時刻でどこにあるべきか
                    btScalar fTargetLimitAngle_ankle = m_param.lowerlimit[(NUM_JOINT + 2)*i + 1 + k] + fTargetAngle_ankle * (m_param.upperlimit[(NUM_JOINT + 2)*i + 1 + k] - m_param.lowerlimit[(NUM_JOINT + 2)*i + 1 + k]);//目標角度
                    btScalar fAngleError_ankle  = fTargetLimitAngle_ankle - fCurAngle_ankle;//動くべき角度
                    btScalar fDesiredAngularVel_ankle = fAngleError_ankle/10;//角速度
                    
                    hingeC2->enableAngularMotor(true, fDesiredAngularVel_ankle, 0.5f);
                }
                
                //btCollisionObject* colObj = Master::dynamicsWorld->getCollisionObjectArray()[1 + (NUM_JOINT+1)*(i+1)];
                btRigidBody* rigid1 = static_cast<btRigidBody*>(m_bodies[(NUM_JOINT+1)*(i+1)]);//ここがそもそもあってるのか問題
                btTransform tran;
                tran.setIdentity();
                rigid1->getMotionState()->getWorldTransform(tran);
                btScalar y = tran.getOrigin().getY();//脚先端の剛体のｙ座標
                
                btVector3 pos  = rigid1->getCenterOfMassPosition();
                btScalar rot   = btScalar(rigid1->getOrientation().getAngle()*RADIAN);
                //btVector3 axis = rigid1->getOrientation().getAxis();
                
                if(rot>120){
                    if (y < 0.6){
                        m_param.turn += 1;
                        leg_state[i] = 1;
                        turn_direction[i] = 1;//右ねじの正
                        m_param.turn_pattern[i] = 1;//i本目がturn判定出したことを保存
                    }
                }
                if(rot<-120){
                    if (y < 0.6){
                        m_param.turn += 1;
                        leg_state[i] = 1;
                        turn_direction[i] = -1;//右ねじの負
                        m_param.turn_pattern[i] = 1;//i本目がturn判定出したことを保存
                    }
                }
            }
        }
        //turn済//
    
        if (m_param.turn >= NUM_TURN){
            //支脚以外の動き//
            if (leg_state[i] == 0){
                //not支脚隣//
                if (state == 0){
                    btScalar fCurAngle1  = hinge2->getAngle1();//現在のaxis1（ツイスト）の角度
                    btScalar fCurAngle2  = hinge2->getAngle2();//現在のaxis2（車輪）の角度
                    
                    btScalar fTargetPercent_hip1 = m_param.targetpercent[(NUM_JOINT + 2)*i] + (int(Master::time_step ) % int(m_param.cycle)) / m_param.cycle;//周期に対する現時刻の割合（往復を1とした時の値）
                    btScalar fTargetPercent_hip2 = m_param.targetpercent[(NUM_JOINT + 2)*i + 1] + (int(Master::time_step ) % int(m_param.cycle)) / m_param.cycle;//m_time/1000の/1000はなくしちゃダメ
                    btScalar fTargetAngle_hip1   = 0.5 * (1 + sin(2* M_PI * fTargetPercent_hip1));//足の動きを波の高さ0.5、中心0.5、周期２PIの波としたときに、現時刻でどこにあるべきか
                    btScalar fTargetAngle_hip2   = 0.5 * (1 + sin(2* M_PI * fTargetPercent_hip2));//足の動きを波の高さ0.5、中心0.5、周期２PIの波としたときに、現時刻でどこにあるべきか
                    btScalar fTargetLimitAngle1 = m_param.lowerlimit[(NUM_JOINT + 2)*i] + fTargetAngle_hip1 * (m_param.upperlimit[(NUM_JOINT + 2)*i] - m_param.lowerlimit[(NUM_JOINT + 2)*i]);//目標角度１
                    btScalar fTargetLimitAngle2 = m_param.lowerlimit[(NUM_JOINT + 2)*i + 1] + fTargetAngle_hip2 * (m_param.upperlimit[(NUM_JOINT + 2)*i +1] - m_param.lowerlimit[(NUM_JOINT + 2)*i +1]);//目標角度２
                    
                    btScalar fAngleError1   = fTargetLimitAngle1  - fCurAngle1;//動くべき角度１
                    btScalar fAngleError2   = fTargetLimitAngle2  - fCurAngle2;//動くべき角度２
                    btScalar fDesiredAngularVel1 = fAngleError1/10;//角速度１
                    btScalar fDesiredAngularVel2 = fAngleError2/10;//角速度２
                    
                    motor1->m_targetVelocity = fDesiredAngularVel1;
                    motor2->m_targetVelocity = fDesiredAngularVel2;
                    
                    for (int k = 1; k<=NUM_JOINT; k++){
                        
                        btHingeConstraint* hingeC2 = static_cast<btHingeConstraint*>(m_joints[k+(NUM_JOINT+1)*i]);//ここの[]内を変えた
                        btScalar fCurAngle_ankle   = hingeC2->getHingeAngle();//現在のankleの角度
                        
                        btScalar fTargetPercent_ankle = m_param.targetpercent[(NUM_JOINT + 2)*i + 1 + k] + (int(Master::time_step) % int(m_param.cycle)) / m_param.cycle;//周期に対する現時刻の割合（往復を1とした時の値）
                        btScalar fTargetAngle_ankle   = 0.5 * (1 + sin(2 * M_PI * fTargetPercent_ankle));//足の動きを波の高さ0.5、中心0.5、周期２PIの波としたときに、現時刻でどこにあるべきか
                        btScalar fTargetLimitAngle_ankle = m_param.lowerlimit[(NUM_JOINT + 2)*i + 1 + k] + fTargetAngle_ankle * (m_param.upperlimit[(NUM_JOINT + 2)*i + 1 + k] - m_param.lowerlimit[(NUM_JOINT + 2)*i + 1 + k]);//目標角度
                        btScalar fAngleError_ankle  = fTargetLimitAngle_ankle - fCurAngle_ankle;//動くべき角度
                        btScalar fDesiredAngularVel_ankle = fAngleError_ankle/10;//角速度
                        
                        hingeC2->enableAngularMotor(true, fDesiredAngularVel_ankle, 0.5);
                    }
                }
            
                //片方or両方支脚隣
                //蹴りだすor持ち上げる
                if (state == 1 || state == 2){
                    
                    btScalar fCurAngle1  = hinge2->getAngle1();//現在のaxis1（ツイスト）の角度
                    btScalar fCurAngle2  = hinge2->getAngle2();//現在のaxis2（車輪）の角度
                    
                    
                    btScalar fTargetPercent_hip2 = m_param.targetpercent[(NUM_JOINT + 2)*i + 1] + (int(Master::time_step ) % int(m_param.cycle)) / m_param.cycle;//m_time/1000の/1000はなくしちゃダメ
                    btScalar fTargetAngle_hip2   = 0.5 * (1 + sin(2* M_PI * fTargetPercent_hip2));//足の動きを波の高さ0.5、中心0.5、周期２PIの波としたときに、現時刻でどこにあるべきか
                    btScalar fTargetLimitAngle2 = -M_PI_2 + fTargetAngle_hip2 * M_PI;//目標角度２
                    
                    btScalar fAngleError1   =  - fCurAngle1;//動くべき角度１
                    btScalar fAngleError2   = fTargetLimitAngle2  - fCurAngle2;//動くべき角度２
                    btScalar fDesiredAngularVel1 = fAngleError1/10;//角速度１
                    btScalar fDesiredAngularVel2 = fAngleError2/10;//角速度２
                    
                    motor1->m_targetVelocity = fDesiredAngularVel1;
                    motor2->m_targetVelocity = fDesiredAngularVel2;
                    
                    
                    
                    for (int k = 1; k<=NUM_JOINT; k++){
                    
                        btHingeConstraint* hingeC2 = static_cast<btHingeConstraint*>(m_joints[k+(NUM_JOINT+1)*i]);//ここの[]内を変えた
                        btScalar fCurAngle_ankle   = hingeC2->getHingeAngle();//現在のankleの角度
                        
                        btScalar fTargetPercent_ankle = m_param.targetpercent[(NUM_JOINT + 2)*i + 1] + (int(Master::time_step) % int(m_param.cycle)) / m_param.cycle;//周期に対する現時刻の割合（往復を1とした時の値）
                        btScalar fTargetAngle_ankle   = 0.5 * (1 + sin(2 * M_PI * fTargetPercent_ankle));//足の動きを波の高さ0.5、中心0.5、周期２PIの波としたときに、現時刻でどこにあるべきか
                        btScalar fTargetLimitAngle_ankle = -M_PI_4 + fTargetAngle_ankle * M_PI_4;//目標角度
                        btScalar fAngleError_ankle  = fTargetLimitAngle_ankle - fCurAngle_ankle;//動くべき角度
                        btScalar fDesiredAngularVel_ankle = fAngleError_ankle/10;//角速度
                        
                        hingeC2->enableAngularMotor(true, fDesiredAngularVel_ankle, 0.5);
                        
                    }
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
        btRigidBody* rigid1 = static_cast<btRigidBody*>(m_bodies[i]);//ここがそもそもあってるのか問題
        rigid1->activate(true);
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
        btRotationalLimitMotor* motors1 = static_cast<btRotationalLimitMotor*>(m_motor1[i]);
        btRotationalLimitMotor* motors2 = static_cast<btRotationalLimitMotor*>(m_motor2[i]);
        
        
        btUniversalConstraint* hinge2 = static_cast<btUniversalConstraint*>(m_joints[(NUM_JOINT+1)*i]);
        
        
        //第一段階（turn本数待ち）//
        if(m_param.turn < NUM_TURN){
            
            //    printf("%d¥n",turn);
            
            //支脚以外の動き//
            if (leg_state[i] == 0){
                
                btScalar fCurAngle1  = hinge2->getAngle1();//現在のaxis1（ツイスト）の角度
                btScalar fCurAngle2  = hinge2->getAngle2();//現在のaxis2（車輪）の角度
                
                btScalar fTargetPercent_hip1 = m_param.targetpercent[(NUM_JOINT + 2)*i] + (int(Master::time_step ) % int(m_param.cycle)) / m_param.cycle;//周期に対する現時刻の割合（往復を1とした時の値）
                btScalar fTargetPercent_hip2 = m_param.targetpercent[(NUM_JOINT + 2)*i + 1] + (int(Master::time_step ) % int(m_param.cycle)) / m_param.cycle;//m_time/1000の/1000はなくしちゃダメ
                btScalar fTargetAngle_hip1   = 0.5 * (1 + sin(2* M_PI * fTargetPercent_hip1));//足の動きを波の高さ0.5、中心0.5、周期２PIの波としたときに、現時刻でどこにあるべきか
                btScalar fTargetAngle_hip2   = 0.5 * (1 + sin(2* M_PI * fTargetPercent_hip2));//足の動きを波の高さ0.5、中心0.5、周期２PIの波としたときに、現時刻でどこにあるべきか
                btScalar fTargetLimitAngle1 = m_param.lowerlimit[(NUM_JOINT + 2)*i] + fTargetAngle_hip1 * (m_param.upperlimit[(NUM_JOINT + 2)*i] - m_param.lowerlimit[(NUM_JOINT + 2)*i]);//目標角度１
                btScalar fTargetLimitAngle2 = m_param.lowerlimit[(NUM_JOINT + 2)*i + 1] + fTargetAngle_hip2 * (m_param.upperlimit[(NUM_JOINT + 2)*i +1] - m_param.lowerlimit[(NUM_JOINT + 2)*i +1]);//目標角度２
                
                btScalar fAngleError1   = fTargetLimitAngle1  - fCurAngle1;//動くべき角度１
                btScalar fAngleError2   = fTargetLimitAngle2  - fCurAngle2;//動くべき角度２
                btScalar fDesiredAngularVel1 = fAngleError1/10;//角速度１
                btScalar fDesiredAngularVel2 = fAngleError2/10;//角速度２
                
                motors1->m_targetVelocity = fDesiredAngularVel1;
                motors2->m_targetVelocity = fDesiredAngularVel2;
                
                for (int k = 1; k<=NUM_JOINT; k++){
                    
                    btHingeConstraint* hingeC2 = static_cast<btHingeConstraint*>(m_joints[k+(NUM_JOINT+1)*i]);//ここの[]内を変えた
                    btScalar fCurAngle_ankle   = hingeC2->getHingeAngle();//現在のankleの角度
                    
                    btScalar fTargetPercent_ankle = m_param.targetpercent[(NUM_JOINT + 2)*i + 1 + k] + (int(Master::time_step) % int(m_param.cycle)) / m_param.cycle;//周期に対する現時刻の割合（往復を1とした時の値）
                    btScalar fTargetAngle_ankle   = 0.5 * (1 + sin(2 * M_PI * fTargetPercent_ankle));//足の動きを波の高さ0.5、中心0.5、周期２PIの波としたときに、現時刻でどこにあるべきか
                    btScalar fTargetLimitAngle_ankle = m_param.lowerlimit[(NUM_JOINT + 2)*i + 1 + k] + fTargetAngle_ankle * (m_param.upperlimit[(NUM_JOINT + 2)*i + 1 + k] - m_param.lowerlimit[(NUM_JOINT + 2)*i + 1 + k]);//目標角度
                    btScalar fAngleError_ankle  = fTargetLimitAngle_ankle - fCurAngle_ankle;//動くべき角度
                    btScalar fDesiredAngularVel_ankle = fAngleError_ankle/10;//角速度
                    
                    hingeC2->enableAngularMotor(true, fDesiredAngularVel_ankle, 0.5);
                }
                
                //turn判定//
                btRigidBody* rigid1 = static_cast<btRigidBody*>(m_bodies[(NUM_JOINT+1)*(i+1)]);
                
                btTransform tran;
                tran.setIdentity();
                rigid1->getMotionState()->getWorldTransform(tran);
                btScalar y = tran.getOrigin().getY();//脚先端の剛体のｙ座標
                
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
                /*        //トルクで振るver.
                 btRigidBody* rigid1 = static_cast<btRigidBody*>(m_rigs[0]->GetBodies()[(NUM_JOINT+1)*(i+1)]);//各腕端
                 
                 btScalar fCurAngle1  = hinge2->getAngle1();//現在のaxis1（ツイスト）の角度
                 btScalar fAngleError1   =  - fCurAngle1;//動くべき角度１
                 btScalar fDesiredAngularVel1 = 100000.f * fAngleError1/ms;//角速度１
                 motors1->m_targetVelocity = fDesiredAngularVel1 ;
                 
                 rigid1->applyTorqueImpulse(btVector3(0,f*0.01,0));
                 
                 //トルクで振る終
                 */
                
                //目標閣で振るver.
                
                btScalar fCurAngle1  = hinge2->getAngle1();//現在のaxis1（ツイスト）の角度
                btScalar fCurAngle2  = hinge2->getAngle2();//現在のaxis2（車輪）の角度
                
                btScalar fTargetPercent_hip2 = m_param.targetpercent[(NUM_JOINT + 2)*i + 1] + (int(Master::time_step ) % int(m_param.cycle*3)) / (m_param.cycle*3.0);//m_time/1000の/1000はなくしちゃダメ
                btScalar fTargetAngle_hip2   = 0.5 * (1 + sin(2* M_PI * fTargetPercent_hip2));//足の動きを波の高さ0.5、中心0.5、周期２PIの波としたときに、現時刻でどこにあるべきか
                btScalar fTargetLimitAngle1 = (m_param.swing[i] - 4) * M_PI_4; //目標角度１
                btScalar fTargetLimitAngle2 = fTargetAngle_hip2 * SWING_ANGLE;//目標角度２
                
                
                btScalar fAngleError1   = fTargetLimitAngle1 - fCurAngle1;//動くべき角度１
                btScalar fAngleError2   = fTargetLimitAngle2 - fCurAngle2;//動くべき角度２
                btScalar fDesiredAngularVel1 = fAngleError1/10;//角速度１
                btScalar fDesiredAngularVel2 = fAngleError2/10;//角速度２
                
                motors1->m_targetVelocity = fDesiredAngularVel1 ;
                motors2->m_targetVelocity = fDesiredAngularVel2 * f;
                
                for (int k = 1; k<=NUM_JOINT; k++){
                    
                    btHingeConstraint* hingeC2 = static_cast<btHingeConstraint*>(m_joints[k+(NUM_JOINT+1)*i]);//ここの[]内を変えた
                    btScalar fCurAngle_ankle   = hingeC2->getHingeAngle();//現在のankleの角度
                    
                    btScalar fTargetPercent_ankle = m_param.targetpercent[(NUM_JOINT + 2)*i + 1] + (int(Master::time_step) % int(m_param.cycle*3)) / (m_param.cycle*3.0);//周期に対する現時刻の割合（往復を1とした時の値）
                    btScalar fTargetAngle_ankle   = 0.5 * (1 + sin(2 * M_PI * fTargetPercent_ankle));//足の動きを波の高さ0.5、中心0.5、周期２PIの波としたときに、現時刻でどこにあるべきか
                    btScalar fTargetLimitAngle_ankle = fTargetAngle_ankle * M_PI_4;//目標角度
                    btScalar fAngleError_ankle  = fTargetLimitAngle_ankle - fCurAngle_ankle;//動くべき角度
                    btScalar fDesiredAngularVel_ankle = fAngleError_ankle/10;//角速度
                    
                    hingeC2->enableAngularMotor(true, fDesiredAngularVel_ankle * f, 0.5);
                }
                //目標閣で振る終
            }
            
            //支脚
            if (leg_state[i] == 1){
                
            }
        }
    }
    
}


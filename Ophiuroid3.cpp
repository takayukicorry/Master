//
//  Ophiuroid3.cpp
//  Master
//
//  Created by 増田貴行 on 2018/01/03.
//  Copyright © 2018年 増田貴行. All rights reserved.
//

#include "Ophiuroid3.hpp"
#include "Utils.hpp"

Ophiuroid3::Ophiuroid3(GAparameter p) {
    m_param = p;
    m_time_step = 0;
    className = 3;
    m_value = 0;
    hasNet = false;
}

Ophiuroid3::Ophiuroid3(Starfish* sf) {
    m_param = sf->m_param;
    m_time_step = 0;
    className = 3;
    m_bodies = sf->m_bodies;
    m_shapes = sf->m_shapes;
    m_joints_hip = sf->m_joints_hip;
    m_joints_ankle = sf->m_joints_ankle;
    m_motor1 = sf->m_motor1;
    m_motor2 = sf->m_motor2;
    
    motor_tX = sf->motor_tX;
    motor_tY = sf->motor_tY;
    motor_tZ = sf->motor_tZ;
    motor_to_groundY = sf->motor_to_groundY;
    motor_to_groundZ = sf->motor_to_groundZ;
    motor_state = sf->motor_state;
    TF_Contact = sf->TF_Contact;
    TF_foward = sf->TF_foward;
    TF_object = sf->TF_object;
    dl_time = sf->dl_time;
    InitTime_tf = sf->InitTime_tf;
    ResumeTime_ground = sf->ResumeTime_ground;
    Init_tf = sf->Init_tf;
    TF_constraint = sf->TF_constraint;
    TF_constraint_ground = sf->TF_constraint_ground;
    TF_axis_direction = sf->TF_axis_direction;
    TF_pos = sf->TF_pos;
    bodies_tf = sf->bodies_tf;
    constraints = sf->constraints;

    hasNet = false;
}

void motorPreTickCallback3(btDynamicsWorld *world, btScalar timeStep) {
    Ophiuroid3* demo = (Ophiuroid3*)world->getWorldUserInfo();
    demo->idleDemo();
}

void motorPreTickCallback3_NEAT(btDynamicsWorld *world, btScalar timeStep) {
    Ophiuroid3* demo = (Ophiuroid3*)world->getWorldUserInfo();
    demo->idleNEAT();
}

void Ophiuroid3::idle() {
    glutPostRedisplay();
}

void Ophiuroid3::idleDemo() {
    m_time_step++;
    
    checkLightDistance();
    setDirection();
    motor();
    contact();
}

void Ophiuroid3::idleNEAT() {
    m_time_step++;
    
    checkLightDistance();
    setDirection_NEAT();
    motor();
    contact();
}

bool Ophiuroid3::checkState() {
    btTransform tr = m_bodies[0]->getWorldTransform();
    btVector3 vY = tr*btVector3(0, 1, 0);
    btVector3 vOrigin = tr.getOrigin();
    
    btVector3 vY_O = vY - vOrigin;
    
    return vY_O[1] > -THRESH_TURN || vOrigin[1] > FLEG_WIDTH;
}

float Ophiuroid3::evalue() {
    btDiscreteDynamicsWorld* dynamicsWorld = GAMaster::createWorld();
    dynamicsWorld->setGravity(btVector3(0, -10, 0));
    //dynamicsWorld->setInternalTickCallback(motorPreTickCallback3, this, true);
    setWorld(dynamicsWorld);
    
    GAMaster::createGround(m_ownerWorld);
    initSF();
    create();
    for (int i = 0; i < SIMULATION_TIME_STEP; i++) {
        dynamicsWorld->stepSimulation(1.f / FPS);
    }
    
    GAMaster::cleanupWorld(m_ownerWorld);
    return (m_value == -1) ? 0 : m_value;
}

float Ophiuroid3::evalue_NEAT(NEAT::Network* net) {
    btDiscreteDynamicsWorld* dynamicsWorld = GAMaster::createWorld();
    dynamicsWorld->setGravity(btVector3(0, -10, 0));
    //dynamicsWorld->setInternalTickCallback(motorPreTickCallback3_NEAT, this, true);
    setWorld(dynamicsWorld);
    setNet(net);
    
    GAMaster::createGround(m_ownerWorld);
    initSF();
    create();
    for (int i = 0; i < SIMULATION_TIME_STEP; i++) {
        dynamicsWorld->stepSimulation(1.f / FPS);
    }
    
    GAMaster::cleanupWorld(m_ownerWorld);
    return (m_value == -1) ? 0 : m_value;
}

btVector3 lightSource(-70, 0, 70);
int lightThresh(170);

void Ophiuroid3::checkLightDistance() {
    btVector3 now;
    double dis;
    //light_patternの数値 = 各腕の光からの距離に応じた受光量の強さを示す
    for (int i = 1; i <= NUM_LEGS; i++){
        now = m_bodies[(NUM_JOINT+1)*i]->getCenterOfMassPosition();
        m_param.light_pattern[i-1] = lightThresh - (int)sqrt((now[0]-lightSource[0])*(now[0]-lightSource[0]) + (now[1]-lightSource[1])*(now[1]-lightSource[1]) + (now[2]-lightSource[2])*(now[2]-lightSource[2]));
        m_param.light_pattern[i-1] = (m_param.light_pattern[i-1] >= 0) ? m_param.light_pattern[i-1] : 0 ;
    }
    now = m_bodies[0]->getCenterOfMassPosition();
    dis = lightThresh - sqrt((now[0]-lightSource[0])*(now[0]-lightSource[0]) + (now[1]-lightSource[1])*(now[1]-lightSource[1]) + (now[2]-lightSource[2])*(now[2]-lightSource[2]));
    m_value += (dis > 0) ? dis : 0;
    //if (m_time_step < SIMULATION_TIME_STEP) std::cout << m_value << std::endl;
}

void Ophiuroid3::setDirection() {
    int mid[NUM_LEGS];
    int out[NUM_LEGS];
    std::map<int, double> f;
    //中間層
    for (int i = 0; i<NUM_LEGS; i++){
        mid[i] = 0;
        for (int k = 0; k<NUM_LEGS; k++){
            mid[i] += m_param.light_pattern[k]*m_param.conect[NUM_LEGS*i + k];
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
    //出力値
    for (int i = 0; i<NUM_LEGS; i++){
        float a = m_param.a[i];//シグモイド関数のパラメータ
        f[i] = 1/(1+exp(-a*out[i]));//出力層からの出力値（シグモイド関数[0,1]）
        
    }
    
    for (auto itr = TF_axis_direction.begin(); itr != TF_axis_direction.end(); ++itr) {
        int index = itr->first;
        int i = (index-101)%NUM_LEGS;
        btScalar angle_target = 2*M_PI*f[i];
        TF_axis_direction[index] = btVector3(cos(angle_target),0,sin(angle_target));
    }
}

void Ophiuroid3::setDirection_NEAT() {
    std::map<int, double> f;
    int i = 0;
    
    m_net->load_sensors(m_param.light_pattern);
    if (!(m_net->activate())) { m_value = -1; return; }
    for (auto itr = m_net->outputs.begin(); itr != m_net->outputs.end(); ++itr) {
        f[i] = (*itr)->activation;
        i++;
    }
    
    for (auto itr = TF_axis_direction.begin(); itr != TF_axis_direction.end(); ++itr) {
        int index = itr->first;
        i = (index-101)%NUM_LEGS;
        btScalar angle_target = 2*M_PI*f[i];
        TF_axis_direction[index] = btVector3(cos(angle_target),0,sin(angle_target));
    }
}

void Ophiuroid3::motor() {
    
    for (auto itr = motor_tZ.begin(); itr != motor_tZ.end(); ++itr) {
        int index = itr->first;
        if (motor_state[index] && m_time_step > InitTime_tf[index]) {
            btRotationalLimitMotor* motor = itr->second;
            //
            //handle
            //
            btScalar angleX = motor_tY[index]->m_currentPosition;
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
            if (TF_axis_direction[index][0] < 0) {
                TF_foward[index] = false;
            }
            motor_tY[index]->m_targetVelocity = (angle_target - angleX)*FPS/10;
            
            //
            //wheel
            //
            btScalar angleZ = motor->m_currentPosition;
            if (!Init_tf[index]) {
                Init_tf[index] = true;
                motor->m_targetVelocity = calcMotorVel(index);
            }
            
            if (angleZ > M_PI_2+m_param.upperlimit2[index-101]-0.1) {
                motor->m_targetVelocity = -calcMotorVel(index);
            } else if (angleZ < M_PI_2+m_param.lowerlimit2[index-101]+0.1) {
                motor->m_targetVelocity = calcMotorVel(index);
                motor->m_maxMotorForce = 100000000;
            }
        }
    }
    
    for (auto itr = motor_to_groundZ.begin(); itr != motor_to_groundZ.end(); ++itr) {
        btRotationalLimitMotor* motor = itr->second;
        motor->m_targetVelocity = -calcMotorVel(itr->first);
    }
}

btScalar Ophiuroid3::calcMotorVel(int index) {
    int i = index - 101;
    
    btScalar fvel = (m_param.upperlimit2[i] - m_param.lowerlimit2[i])*2*FPS / m_param.cycle2;
    
    return fvel;
}

void Ophiuroid3::contact() {
    std::vector<int > contacts;
    
    int numManifolds = m_ownerWorld->getDispatcher()->getNumManifolds();
    for (int i = 0; i < numManifolds; i++)
    {
        btPersistentManifold* contactManifold =  m_ownerWorld->getDispatcher()->getManifoldByIndexInternal(i);
        const btCollisionObject* obA = contactManifold->getBody0();
        const btCollisionObject* obB = contactManifold->getBody1();
        btRigidBody* bodyA = btRigidBody::upcast(const_cast<btCollisionObject *>(obA));
        btRigidBody* bodyB = btRigidBody::upcast(const_cast<btCollisionObject *>(obB));
        btRigidBody* bodyC;
        btRigidBody* bodyG;

        int obIDA = obA->getUserIndex();
        int obIDB = obB->getUserIndex();

        //when a tubefeet attach ground
        if ((10 <= obIDA && obIDA < 100) || (10 <= obIDB && obIDB < 100)) {
            break;
        }
        
        if (0 < obIDA && obIDA <= NUM_GROUND*NUM_GROUND) {
            bodyC = bodyB;
            bodyG = bodyA;
        } else if (0 < obIDB && obIDB <= NUM_GROUND*NUM_GROUND) {
            bodyC = bodyA;
            bodyG = bodyB;
        } else {
            break;
        }
        int numContacts = contactManifold->getNumContacts();
        if (numContacts >= 1) {
            int k = -1;
            for (int j = 0; j < numContacts; j++) {
                btManifoldPoint& pt = contactManifold->getContactPoint(j);
                if (pt.getDistance() < 0.5f){k = j;break;}
            }
            if (k == -1) {break;}
            
            btManifoldPoint& pt = contactManifold->getContactPoint(k);
            const btVector3& ptB = pt.getPositionWorldOnB();
            int obIDC = bodyC->getUserIndex();
        
            if (!TF_Contact[obIDC]) {
                btScalar angle = motor_tZ[obIDC]->m_currentPosition;
                int percent = rand()%100;
                if (percent < TF_PERCENT) {
                    if (((angle > ANGLE_ATTACH_Z_foward && TF_foward[obIDC]) || (angle < ANGLE_ATTACH_Z_back && !TF_foward[obIDC])) && ResumeTime_ground[obIDC] < m_time_step) {
                        TF_Contact[obIDC] = true;
                        dl_time[obIDC] = m_time_step + DL_TIME;
                        //change motor state
                        motor_state[obIDC] = false;
                        motor_tX[obIDC]->m_enableMotor = false;
                        motor_tY[obIDC]->m_enableMotor = false;
                        motor_tZ[obIDC]->m_enableMotor = false;
                        //create tf - ground constraint
                        btTransform btUp, btDown ,btC;
                        btC = bodyC->getWorldTransform();
                        btUp.setIdentity(); btDown.setIdentity();
                        btUp.setOrigin(btVector3(0,5,0));
                        btDown.setOrigin(btVector3(0,-5,0));
                        btUp = btC*btUp; btDown = btC*btDown;
                        btVector3 axC, ax;
                        axC = btUp.getOrigin() - btDown.getOrigin();
                        ax[0] = -axC[2]; ax[1] = 0; ax[2] = axC[0];
                        
                        btUniversalConstraint* univ = new btUniversalConstraint(*bodyG, *bodyC, ptB+btVector3(0,RADIUS_TF,0), btVector3(0,1,0), ax);
                        univ->setLowerLimit(-M_PI, 0);
                        univ->setUpperLimit(M_PI, 0);
                        TF_constraint_ground[obIDC] = univ;
                        m_ownerWorld->addConstraint(univ);
                        //create tf - ground motor
                        btRotationalLimitMotor* motor1 = univ->getRotationalLimitMotor(1);//wheel
                        btRotationalLimitMotor* motor2 = univ->getRotationalLimitMotor(2);//handle
                        motor1->m_enableMotor = true;
                        motor1->m_targetVelocity = 0;
                        motor1->m_maxMotorForce = 1000000;
                        motor2->m_enableMotor = true;
                        motor2->m_targetVelocity = 0;
                        motor2->m_maxMotorForce = 1000000;
                        motor_to_groundZ[obIDC] = motor1;
                        motor_to_groundY[obIDC] = motor2;
                    }
                } else {
                    ResumeTime_ground[obIDC] = m_time_step + RE_TIME;;
                }
            } else {
                btScalar angle = motor_to_groundZ[obIDC]->m_currentPosition;
                if(angle < ANGLE_DETACH - ANGLE_ATTACH || dl_time[obIDC] < m_time_step) {
                    ResumeTime_ground[obIDC] = m_time_step + RE_TIME;
                    //remove tubefeet - ground (constraint & motor)
                    m_ownerWorld->removeConstraint(TF_constraint_ground[obIDC]);
                    motor_to_groundY.erase(obIDC);
                    motor_to_groundZ.erase(obIDC);
                    TF_Contact[obIDC] = false;
                    motor_state[obIDC] = true;
                    //activate tf motor
                    motor_tX[obIDC]->m_enableMotor = true;
                    motor_tY[obIDC]->m_enableMotor = true;
                    motor_tZ[obIDC]->m_enableMotor = true;
                    if (TF_foward[obIDC]) {
                        motor_tZ[obIDC]->m_targetVelocity = calcMotorVel(obIDC);
                    } else {
                        motor_tZ[obIDC]->m_targetVelocity = -calcMotorVel(obIDC);
                    }
                }
            }
        }
    }
}

btScalar F_SIZE = FBODY_SIZE;
btScalar FLEG_SIZE = F_SIZE/2.0;
btScalar FLEG_SIZE_WIDTH = FLEG_WIDTH;

void Ophiuroid3::setSpring(btGeneric6DofSpringConstraint* spring, int index) {
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

btRigidBody* Ophiuroid3::initTubefeet(btScalar* scale, const btTransform &startTransform)
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

btRigidBody* Ophiuroid3::createRigidBody(btScalar mass, const btTransform &startTransform, btCollisionShape *shape, int index) {
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

void Ophiuroid3::initSF() {
    
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
    float alpha = 37*M_PI/36;
    float theta = M_PI - alpha;
    
    btVector3 vRoot = btVector3(btScalar(0.), btScalar(FHEIGHT), btScalar(0.));
    btTransform transform, transformY, transformS, transformSY;
    transform.setIdentity();
    transform.setOrigin(vRoot);
    m_bodies[0] = createRigidBody(btScalar(M_OBJ0), transform, m_shapes[0], 10);
    
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
    }
    //
    // Setup some damping on the m_bodies
    //
    for (i = 0; i < m_bodies.size(); ++i) {
        m_bodies[i]->setDamping(0.05, 0.85);
        m_bodies[i]->setDeactivationTime(1000000000000.f);
        m_bodies[i]->setSleepingThresholds(0.f, 0.f);
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
        m_motor1[i] = hinge2C->getRotationalLimitMotor(AXIS1_ID);
        m_motor2[i] = hinge2C->getRotationalLimitMotor(AXIS2_ID);
        m_motor1[i]->m_maxMotorForce = MAX_MOTOR_TORQUE;
        m_motor2[i]->m_maxMotorForce = MAX_MOTOR_TORQUE;
        
        m_motor1[i]->m_enableMotor = true;
        m_motor2[i]->m_enableMotor = true;
        
        for (int k = 1; k <= NUM_JOINT; k++)
        {
            btVector3 axisA(0, 0, 1);
            btVector3 axisB(0, 0, 1);
            btVector3 pivotA(0, -(FLEG_WIDTH + FLEG_LENGTH/2), 0);
            btVector3 pivotB(0, FLEG_WIDTH + FLEG_LENGTH/2, 0);
            if(k==1){pivotA = btVector3(0, 0, 0); pivotB = btVector3(0, FLEG_WIDTH*2 + FLEG_LENGTH/2, 0);}
            btHingeConstraint* joint2 = new btHingeConstraint(*m_bodies[k+(NUM_JOINT+1)*i], *m_bodies[k+1+(NUM_JOINT+1)*i], pivotA, pivotB, axisA, axisB);
            
            //*****************************************joint2->setLimit(m_param.lowerlimit[(NUM_JOINT+2)*i + 1 + k], m_param.upperlimit[(NUM_JOINT+2)*i + 1 + k]);
            joint2->setLimit(-M_PI, M_PI);
            joint2->setUserConstraintId(10);
            m_joints_ankle[k-1+NUM_JOINT*i] = joint2;
            m_ownerWorld->addConstraint(m_joints_ankle[k-1+NUM_JOINT*i], true);
        }
    }
}

void Ophiuroid3::create() {
    if (!hasNet) {
        m_ownerWorld->setInternalTickCallback(motorPreTickCallback3, this, true);
    } else {
        m_ownerWorld->setInternalTickCallback(motorPreTickCallback3_NEAT, this, true);
    }
    zeroFriction(true);
    
    //if (!kCheck_first) return;
    
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

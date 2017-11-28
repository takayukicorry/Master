//
//  Ophiuroid2.cpp
//  Master
//
//  Created by 増田貴行 on 2017/10/31.
//  Copyright © 2017年 増田貴行. All rights reserved.
//

#include "Ophiuroid2.hpp"
#include "Utils.hpp"

Ophiuroid2::Ophiuroid2(GAparameter p) {
    ///////////////////////////////////
    ///  create() で変数は初期化される  ///
    ///////////////////////////////////
    stay = true;
    m_param = p;
}

void Ophiuroid2::idle() {
    ContactAction();
    glutPostRedisplay();
    setDirection();
    setDirection2();
    ControllTubeFeet();
}

bool Ophiuroid2::checkState() {
    btTransform tr = m_bodies[0]->getWorldTransform();
    btVector3 vY = tr*btVector3(0, 1, 0);
    btVector3 vOrigin = tr.getOrigin();
    
    btVector3 vY_O = vY - vOrigin;
    
    return vY_O[1] > -sin(2*M_PI/5);
}

void Ophiuroid2::initSF() {
    
    float alpha = 37*M_PI/36;
    float theta = M_PI - alpha;
    GAmanager manager;
    
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
    m_bodies[0] = createRigidBody(btScalar(M_OBJ), transform, m_shapes[0]);
    
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
        
        m_bodies[1+(NUM_JOINT+1)*i] = createRigidBody(btScalar(M_OBJ), transformS*transformSY, m_shapes[1+(NUM_JOINT+1)*i]);
        
        for (int k = 1; k <= NUM_JOINT; k++)
        {
            transform.setIdentity();
            transform.setOrigin(Point);
            transform.setRotation(btQuaternion(btVector3(0, 1, 0), -fAngle));
            transformY.setIdentity();
            transformY.setRotation(btQuaternion(btVector3(0, 0, 1), M_PI_2 - theta * k));
            
            m_bodies[k+1+(NUM_JOINT+1)*i] = createRigidBody(btScalar(M_OBJ), transform*transformY, m_shapes[k+1+(NUM_JOINT+1)*i]);
            Point += btVector3(btScalar(fCos*(0.5*FLEG_LENGTH+FLEG_WIDTH)*cos(k*theta)),btScalar(-(0.5*FLEG_LENGTH+FLEG_WIDTH)*sin(k*theta)),btScalar(fSin*(0.5*FLEG_LENGTH+FLEG_WIDTH)*cos(k*theta))) + btVector3(btScalar(fCos*(0.5*FLEG_LENGTH+FLEG_WIDTH)*cos((k+1)*theta)),btScalar(-(0.5*FLEG_LENGTH+FLEG_WIDTH)*sin((k+1)*theta)),btScalar(fSin*(0.5*FLEG_LENGTH+FLEG_WIDTH)*cos((k+1)*theta)));
        }
        m_bodies[(NUM_JOINT+1)*(i+1)]->setFriction(5.0);//摩擦
    }
    
    //停止が続いてもsleeping状態にならないようにする//
    // Setup some damping on the m_bodies
    /*
    for (i = 0; i < m_bodies.size(); ++i)
    {
        m_bodies[i]->setDamping(0.05, 0.85);
        m_bodies[i]->setDeactivationTime(1000000000000.f);
        m_bodies[i]->setSleepingThresholds(1.6, 2.5);
        m_bodies[i]->setSleepingThresholds(0.5f, 0.5f);
        m_bodies[i]->forceActivationState(DISABLE_DEACTIVATION);
        
    }
    */
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
        Master::dynamicsWorld->addConstraint(m_joints_hip[i], true);
        
        const int AXIS1_ID = 2;
        const int AXIS2_ID = 1;
        const float MAX_MOTOR_TORQUE = 10000000000;//出力[W] ＝ ( 2 * M_PI / 60 ) × T[N・m] × θ[rad/min]
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
            
            joint2->setLimit(manager.pool[0].lowerlimit[(NUM_JOINT+2)*i + 1 + k], manager.pool[0].upperlimit[(NUM_JOINT+2)*i + 1 + k]);
            m_joints_ankle[k-1+NUM_JOINT*i] = joint2;
            Master::dynamicsWorld->addConstraint(m_joints_ankle[k-1+NUM_JOINT*i], true);
        }
    }
}

void Ophiuroid2::create() {
    activateMotor(false);
    
    std::vector<btRigidBody* > bodies_tf;
    std::vector<btRigidBody* > bodies_amp;
    std::vector<btTypedConstraint* > constraints;
    
    /*** create tubefeet & amp ***/
    std::random_device rnd;
    std::mt19937 mt(rnd());
    std::uniform_int_distribution<> rand100(0, 99);
    
    btScalar scale[] = {btScalar(RADIUS_TF), btScalar(LENGTH)};
    btVector3 pos_tf, pos_amp;
    int col, row;
    for (int i = 0; i < NUM_TF; i++) {
        col = i % 2;
        row = i / 2;
        pos_tf = btVector3(RADIUS_TF*2 + row * RADIUS_TF * 4, INIT_POS_Y-(RADIUS_TF*2+LENGTH/2), pow(-1, col) * RADIUS_TF * 2);
        pos_amp = btVector3(RADIUS_TF*2 + row * RADIUS_TF * 4, INIT_POS_Y, pow(-1, col) * RADIUS_TF * 2);
        for (int j = 1; j <= NUM_LEGS; j++) {
            //tf - amp (object)
            btRigidBody* body_amp = initAmp(btScalar(RADIUS_TF), pos_amp);
            btRigidBody* body_tf = initTubefeet(scale, pos_tf);
            int index = 100 + NUM_TF * i + j;
            body_tf->setUserIndex(index);
            body_amp->setUserIndex(index);
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
            InitTime_tf[index] = ResumeTime_tf[index];
            //for next
            pos_tf = RotateY(pos_tf, M_PI*2/5);
            pos_amp = RotateY(pos_amp, M_PI*2/5);
        }
    }
    
    for (int i = 0; i < bodies_tf.size(); i++) {
        Master::dynamicsWorld->addRigidBody(bodies_tf[i], RX_COL_TF, RX_COL_GROUND);
    }
    
    for (int i = 0; i < bodies_amp.size(); i++) {
        Master::dynamicsWorld->addRigidBody(bodies_amp[i], RX_COL_AMP, RX_COL_GROUND);
    }
    
    for (int i = 0; i < constraints.size(); i++) {
        Master::dynamicsWorld->addConstraint(constraints[i]);
    }
}

btRigidBody* Ophiuroid2::createRigidBody(btScalar mass, const btTransform &startTransform, btCollisionShape *shape) {
    bool isDynamic = (mass != 0.f);
    
    btVector3 localInertia(0,0,0);
    if (isDynamic)
        shape->calculateLocalInertia(mass,localInertia);
    
    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);
    body->setUserIndex(10);
    
    Master::dynamicsWorld->addRigidBody(body, RX_COL_BODY, RX_COL_GROUND | RX_COL_BODY);
    
    return body;
}

btRigidBody* Ophiuroid2::initAmp(btScalar scale, const btVector3 position)
{
    btCollisionShape* colShape = new btSphereShape(scale);
    
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

btRigidBody* Ophiuroid2::initTubefeet(btScalar* scale, const btVector3 position)
{
    btCollisionShape* sBodyShape = new btCapsuleShape(scale[0], scale[1]);
    
    btScalar mass1(M_TF);
    bool isDynamic = (mass1 != 0.f);
    
    btVector3 localInertia1(0, 0, 0);
    if (isDynamic)
        Master::groundShape->calculateLocalInertia(mass1, localInertia1);
    
    
    btTransform sBodyTransform;
    sBodyTransform.setIdentity();
    sBodyTransform.setOrigin(position);
    btDefaultMotionState* myMotionState1 = new btDefaultMotionState(sBodyTransform);
    
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass1, myMotionState1, sBodyShape, localInertia1);
    btRigidBody* body = new btRigidBody(rbInfo);
    
    body->setActivationState(DISABLE_DEACTIVATION);
    
    return body;
}

void Ophiuroid2::ControllTubeFeet()
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
            btScalar velocity_z = body->getLinearVelocity()[2];
            
            if (velocity_x < velocity_all_x) {
                velocity_all_x = velocity_x;
            }
            if (velocity_z > velocity_all_z) {
                velocity_all_z = velocity_z;
            }
        }
    }
    
    //interacting of tf with body (X, Z direction)
    stay = m_bodies[0]->getCenterOfMassPosition()[1] <= FLEG_WIDTH + 1;
    for (auto itr = m_bodies.begin(); itr != m_bodies.end(); ++itr) {
        btRigidBody* body = itr->second;
        
        if (body && body->getMotionState())
        {
            btVector3 pos = body->getCenterOfMassPosition();
            btTransform tran = body->getWorldTransform();
            tran.setOrigin(btVector3(pos[0]+velocity_all_x*1.5/FPS, pos[1], pos[2]+velocity_all_z*1.5/FPS));
            body->setCenterOfMassTransform(tran);
            
            if (stay) {
                /*btVector3 vel = body->getLinearVelocity();
                if (vel[1] <= 0) vel[1] = 0;*/
                body->setLinearVelocity(btVector3(0, 0, 0));
            }
        }
    }
    
    //interacting of tf with amp (X, Z direction)
    for (auto itr = TF_object_amp.begin(); itr != TF_object_amp.end(); ++itr) {
        
        int index = itr->first;
        btRigidBody* body = itr->second;
        
        if (body && body->getMotionState() && !TF_contact[index])
        {
            btVector3 pos = body->getCenterOfMassPosition();
            btTransform tran = body->getWorldTransform();

            if (Master::time_step > InitTime_tf[index]) {
            tran.setOrigin(btVector3(pos[0]+velocity_all_x/FPS, INIT_POS_Y - (LENGTH/2 + RADIUS_TF*3)*(1 - sin(2*M_PI*((Master::time_step-InitTime_tf[index])%(SECOND*2))/(SECOND*2) + M_PI_2)), pos[2]+velocity_all_z/FPS));
            } else {
            tran.setOrigin(btVector3(pos[0]+velocity_all_x/FPS, pos[1], pos[2]+velocity_all_z/FPS));
            }
            
            body->setCenterOfMassTransform(tran);
        }
    }
    
    //motion of tubefeet - amp (motor)
    for (auto itr = motor_tZ.begin(); itr != motor_tZ.end(); ++itr) {
        
        int index = itr->first;
        btRotationalLimitMotor* motor = itr->second;
        
        motor->m_maxMotorForce = 1000000000000000;
        motor_tY[index]->m_maxMotorForce = 1000000000000000;
        
        //handle motor
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
        if (!TF_contact[index] && ResumeTime_tf[index] < Master::time_step)
        {
            double target_velocity = motor->m_targetVelocity;
            if (target_velocity == 0) {
                motor->m_targetVelocity = -ANGLE_VELOCITY_TF;
            }
            
            btScalar angle = motor->m_currentPosition;
            if (TF_contact_times[index]==0 && -(ANGLE - 0.1) >= angle && target_velocity <= 0) {
                motor->m_targetVelocity *= -1.0;
            } else if (-(0.5*ANGLE - 0.1) >= angle && target_velocity <= 0) {
                motor->m_targetVelocity *= -1.0;
            }
            
            if (TF_contact_times[index]==0 && angle >= ANGLE-0.1  && target_velocity >= 0) {
                motor->m_targetVelocity *= -1.0;
            } else if (1.5*ANGLE - 0.1 <= angle && target_velocity >= 0) {
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
void Ophiuroid2::ContactAction()
{
    std::vector<int > contacts;
    
    int numManifolds = Master::dynamicsWorld->getDispatcher()->getNumManifolds();
    for (int i = 0; i < numManifolds; i++)
    {
        btPersistentManifold* contactManifold =  Master::dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
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
                if (index >= 1000 || index < 100) continue;
                
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
                    Master::dynamicsWorld->removeRigidBody(TF_object_amp[index]);
                    Master::dynamicsWorld->removeConstraint(TF_constraint_amp[index]);
                    motor_tY.erase(index);
                    motor_tZ.erase(index);
                    TF_contact[index] = true;
                    TF_contact_times[index]++;
                    
                    //create tubefeet - ground (constraint)
                    btUniversalConstraint* univ = new btUniversalConstraint(*bodyA, *bodyB, btVector3(ptB[0],ptB[1]+RADIUS_TF ,ptB[2] ), btVector3(0, 1, 0),btVector3(cos(TF_axis_angle[index]), 0, sin(TF_axis_angle[index])));//global
                    univ->setLowerLimit(-ANGLE, -M_PI);
                    univ->setUpperLimit(ANGLE, M_PI);
                    TF_constraint_ground[index] = univ;
                    Master::dynamicsWorld->addConstraint(univ);
                    
                    //create tubefeet - ground (motor)
                    btRotationalLimitMotor* motor1 = univ->getRotationalLimitMotor(1);//wheel
                    btRotationalLimitMotor* motor2 = univ->getRotationalLimitMotor(2);//handle
                    motor1->m_enableMotor = true;
                    motor2->m_enableMotor = true;
                    motor_to_groundZ[index] = motor1;
                    motor_to_groundY[index] = motor2;
                    DeleteTime_tf[index] = Master::time_step + int(double(ANGLE_ATTACH - ANGLE_DETACH)/double(ANGLE_VELOCITY_GROUND) * double(FPS) * 2.0);
                    
                }
                else if (TF_contact[index])
                {
                    //delete the tubefeet attaching too long time & after that, create new one
                    if (Master::time_step > DeleteTime_tf[index]) {
                        //remove tubefeet - ground (constraint & motor)
                        Master::dynamicsWorld->removeConstraint(TF_constraint_ground[index]);
                        TF_constraint_ground.erase(index);
                        motor_to_groundY.erase(index);
                        motor_to_groundZ.erase(index);
                        TF_contact[index] = false;
                        TF_contact_times[index] = 0;
                        
                        //remove tubefeet
                        Master::dynamicsWorld->removeRigidBody(TF_object[index]);
                        TF_object.erase(index);
                        
                        //create new one
                        btRigidBody* body_centor = m_bodies[0];
                        if (body_centor && body_centor->getMotionState())
                        {
                            btVector3 pos = body_centor->getCenterOfMassPosition();
                            std::random_device rnd;
                            std::mt19937 mt(rnd());
                            std::uniform_int_distribution<> rand100(0, 99);
                            btScalar scale[] = {btScalar(RADIUS_TF), btScalar(LENGTH)};
                            int n = index % 10;//j
                            int m = (index-100)/10;//i
                            int col = m % 2;
                            int row = m / 2;
                            int h = INIT_POS_Y-RADIUS_TF*2-LENGTH/2;
                            int from_x = RADIUS_TF*2;
                            btVector3 pos_tf = btVector3(from_x + row * RADIUS_TF * 4, h, pow(-1, col) * RADIUS_TF * 2);
                            btVector3 pos_amp = btVector3(from_x + row * RADIUS_TF * 4 , INIT_POS_Y, pow(-1, col) * RADIUS_TF * 2);
                            pos_tf = RotateY(pos_tf, M_PI*(n-1)*2/5);
                            pos_amp = RotateY(pos_amp, M_PI*(n-1)*2/5);
                            pos_tf[0] += pos[0];
                            pos_tf[2] += pos[2];
                            pos_amp[0] += pos[0];
                            pos_amp[2] += pos[2];
                            //tf - amp (object)
                            btRigidBody* body_amp = initAmp(btScalar(RADIUS_TF), pos_amp);
                            btRigidBody* body_tf = initTubefeet(scale, pos_tf);
                            body_tf->setUserIndex(index);
                            body_amp->setUserIndex(index);
                            TF_object[index] = body_tf;
                            TF_object_amp[index] = body_amp;
                            TF_contact[index] = false;
                            Master::dynamicsWorld->addRigidBody(body_tf, RX_COL_TF, RX_COL_GROUND);
                            Master::dynamicsWorld->addRigidBody(body_amp, RX_COL_AMP, RX_COL_GROUND);
                            //tf - amp (constraint)
                            btUniversalConstraint* univ = new btUniversalConstraint(*body_amp, *body_tf, pos_amp, btVector3(0, 1, 0), btVector3(0, 0, 1));//global
                            univ->setLowerLimit(-ANGLE, -M_PI);
                            univ->setUpperLimit(ANGLE, M_PI);
                            TF_constraint_amp[index] = univ;
                            Master::dynamicsWorld->addConstraint(univ);
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
                        Master::dynamicsWorld->removeConstraint(TF_constraint_ground[index]);
                        motor_to_groundY.erase(index);
                        motor_to_groundZ.erase(index);
                        TF_contact[index] = false;
                        
                        //create amp (object)
                        btTransform tr_tf = bodyB->getWorldTransform();
                        btTransform tr_amp, tr_amp_after;
                        tr_amp.setIdentity();
                        tr_amp.setOrigin(btVector3(0, LENGTH/2 + RADIUS_TF*2, 0));
                        tr_amp_after = tr_tf * tr_amp;
                        
                        btVector3 pos_tf = bodyB->getCenterOfMassPosition();
                        btVector3 pos_amp = tr_amp_after.getOrigin();
                        btRigidBody* body_amp = initAmp(RADIUS_TF, pos_amp);
                        TF_object_amp[index] = body_amp;
                        Master::dynamicsWorld->addRigidBody(body_amp);
                        
                        //create tubefeet - amp (constraint)
                        btUniversalConstraint* univ = new btUniversalConstraint(*body_amp, *TF_object[index], pos_amp, pos_amp-pos_tf, btVector3(cos(TF_axis_angle[index]), 0, sin(TF_axis_angle[index])));//global
                        univ->setLowerLimit(-0.5*ANGLE, 0);
                        univ->setUpperLimit(1.5*ANGLE, 0);
                        TF_constraint_amp[index] = univ;
                        Master::dynamicsWorld->addConstraint(univ);
                        
                        //create tubefeet - amp (motor)
                        btRotationalLimitMotor* motor1 = univ->getRotationalLimitMotor(1);//wheel
                        btRotationalLimitMotor* motor2 = univ->getRotationalLimitMotor(2);//handle
                        motor1->m_enableMotor = true;
                        motor1->m_targetVelocity = ANGLE_VELOCITY_TF;
                        motor2->m_enableMotor = true;
                        motor_tZ[index] = motor1;
                        motor_tY[index] = motor2;
                        ResumeTime_tf[index] = Master::time_step;
                        
                    }
                }
            }
        }
    }
}

void Ophiuroid2::setDirection() {
    //***************light_patternの更新
    
}

void Ophiuroid2::setDirection2() {
    int mid[NUM_LEGS];
    int out[NUM_LEGS];
    
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
        float f = 1/(1+exp(-a*out[i]));//出力層からの出力値（シグモイド関数[0,1]）
        
        //***************このfを何かに使う

    }
    
}

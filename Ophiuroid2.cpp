//
//  Ophiuroid2.cpp
//  Master
//
//  Created by 増田貴行 on 2017/10/31.
//  Copyright © 2017年 増田貴行. All rights reserved.
//

#include "Ophiuroid2.hpp"
#include "Utils.hpp"

Ophiuroid2::Ophiuroid2() {
    
}

void Ophiuroid2::idle() {
    ContactAction();
    glutPostRedisplay();
    ControllTubeFeet();
}

bool Ophiuroid2::checkState() {
    return false;
}

void Ophiuroid2::create() {
    std::vector<btRigidBody* > bodies_body;
    std::vector<btRigidBody* > bodies_tf;
    std::vector<btRigidBody* > bodies_amp;
    std::vector<btTypedConstraint* > constraints;
    
    /*** create body ***/
    btRigidBody* body_body = initBody(btVector3(RADIUS*2, LENGTH, RADIUS*2), btVector3(0, INIT_POS_Y-RADIUS*2, 0));
    BODY_object[0] = body_body;
    body_body->setUserIndex(1000);
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
            //for next
            pos_tf = RotateY(pos_tf, M_PI*2/5);
            pos_amp = RotateY(pos_amp, M_PI*2/5);
        }
    }
    
    for (int i = 0; i < bodies_body.size(); i++) {
        Master::dynamicsWorld->addRigidBody(bodies_body[i], RX_COL_BODY, RX_COL_GROUND);
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

//create body
btRigidBody* Ophiuroid2::initBody(const btVector3 scale, const btVector3 position)
{
    btCollisionShape* sBodyShape = new btBoxShape(scale);
    
    btTransform sBodyTransform;
    sBodyTransform.setIdentity();
    sBodyTransform.setOrigin(position);
    
    btScalar mass(M_BODY);
    
    bool isDynamic = (mass != 0.f);
    
    btVector3 localInertia(0, 0, 0);
    if (isDynamic)
        sBodyShape->calculateLocalInertia(mass, localInertia);
    
    btDefaultMotionState* myMotionState = new btDefaultMotionState(sBodyTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, sBodyShape, localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);
    
    body->setActivationState(DISABLE_DEACTIVATION);
    
    return body;
}

//create an arm
btRigidBody* Ophiuroid2::initArm(const btVector3 scale, const btVector3 position, const btQuaternion rot)
{
    btCollisionShape* sBodyShape = new btBoxShape(scale);
    
    btScalar mass1(M_ARM);
    bool isDynamic = (mass1 != 0.f);
    
    btVector3 localInertia1(0, 0, 0);
    if (isDynamic)
        sBodyShape->calculateLocalInertia(mass1, localInertia1);
    
    
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
            tran.setOrigin(btVector3(pos[0]+velocity_all_x/FPS, INIT_POS_Y - (LENGTH/2 + 4) + (LENGTH/2 + 4)*sin(2*M_PI*(Master::time_step%(SECOND*2))/(SECOND*2) + M_PI_2), pos[2]+velocity_all_z/FPS));
            
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
        if (!TF_contact[index] && ResumeTime_tf[index] < Master::time_step)
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
                    btUniversalConstraint* univ = new btUniversalConstraint(*bodyA, *bodyB, btVector3(ptB[0],ptB[1]+RADIUS ,ptB[2] ), btVector3(0, 1, 0),btVector3(cos(TF_axis_angle[index]), 0, sin(TF_axis_angle[index])));//global
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
                        btVector3 pos_tf = bodyB->getCenterOfMassPosition();
                        btVector3 pos_amp = btVector3(pos_tf[0]+(LENGTH/2+RADIUS*2)*sin(angle+ANGLE_ATTACH)*sin(TF_axis_angle[index]), pos_tf[1]+(LENGTH/2+RADIUS*2)*cos(angle+ANGLE_ATTACH), pos_tf[2]-(LENGTH/2+RADIUS*2)*sin(angle+ANGLE_ATTACH)*cos(TF_axis_angle[index]));
                        btRigidBody* body_amp = initAmp(RADIUS, pos_amp);
                        TF_object_amp[index] = body_amp;
                        Master::dynamicsWorld->addRigidBody(body_amp);
                        
                        //create tubefeet - amp (constraint)
                        btUniversalConstraint* univ = new btUniversalConstraint(*body_amp, *TF_object[index], pos_amp, btVector3(pos_amp[0]-pos_tf[0], pos_amp[1]-pos_tf[1], pos_amp[2]-pos_tf[2]), btVector3(cos(TF_axis_angle[index]), 0, sin(TF_axis_angle[index])));//global
                        univ->setLowerLimit(0, -M_PI);
                        univ->setUpperLimit(2*ANGLE, M_PI);
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




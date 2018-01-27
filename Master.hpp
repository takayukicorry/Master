//
//  Master.hpp
//  Master
//
//  Created by 増田貴行 on 2017/10/31.
//  Copyright © 2017年 増田貴行. All rights reserved.
//

#ifndef Master_hpp
#define Master_hpp

#include "Ophiuroid.hpp"
#include "Ophiuroid2.hpp"
#include "Ophiuroid3.hpp"
//#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class Master {
    
private:
    static btDefaultCollisionConfiguration* collisionConfiguration;
    static btCollisionDispatcher* dispatcher;
    static btBroadphaseInterface* overlappingPairCache;
    static btSequentialImpulseConstraintSolver* solver;
    
    Starfish* starfish;
    GAparameter m_param;
    
    IplImage* video_buf;
    IplImage* video;
    CvVideoWriter* videoWriter;
    
    bool save;
    
public:
    static btDiscreteDynamicsWorld* dynamicsWorld;
    static btCollisionShape* groundShape;
    static btCollisionShape* groundShape_wall;

    Master(const char*);
    void Render();
    void idle();
    void idleDemo();
    void InitBullet();
    void CleanupBullet();
    void init();
    void setStarfish(Starfish* s);
    Starfish* getStarfishes() { return starfish; }
    void createGround();
    void createStarfish();
    void CleanupStarfish();
    void checkStarfish();
    void saveVideo();
    void releaseVideo();
    void keyboard(unsigned char, int, int);
};

#endif /* Master_hpp */

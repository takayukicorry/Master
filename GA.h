//
//  GA.h
//  Master
//
//  Created by 増田貴行 on 2017/10/31.
//  Copyright © 2017年 増田貴行. All rights reserved.
//

#ifndef GA_h
#define GA_h

#include "Variables.h"

class GAparameter {

public:
    float cycle;
    float upperlimit[ARRAY_LENGTH];
    float lowerlimit[ARRAY_LENGTH];
    float targetpercent[ARRAY_LENGTH];//shokiisou
    int turn_pattern[NUM_LEGS];
    double light_pattern[NUM_LEGS];
    
    int conect[CONECT_LENGTH];//weight of the conection
    double a[NUM_LEGS];//parameter of sigumoido function
    int swing[NUM_LEGS];//way of swing in second round
    
    int turn;
    int light;
    int ee;
    
    float cycle2;
    float upperlimit2[ARRAY_LENGTH_2];
    float lowerlimit2[ARRAY_LENGTH_2];
    float upperlimit2_2[ARRAY_LENGTH_2];
    float lowerlimit2_2[ARRAY_LENGTH_2];
    float targetpercent2[ARRAY_LENGTH_2];//shokiisou
    
};

#endif /* GA_h */

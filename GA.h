//
//  GA.h
//  Master
//
//  Created by 増田貴行 on 2017/10/31.
//  Copyright © 2017年 増田貴行. All rights reserved.
//

#include "Variables.h"

#ifndef GA_h
#define GA_h

class GAparameter {

public:
    float cycle;
    float upperlimit[ARRAY_LENGTH];
    float lowerlimit[ARRAY_LENGTH];
    float targetpercent[ARRAY_LENGTH];//shokiisou
    int turn_pattern[NUM_LEGS];
    
    int conect[CONECT_LENGTH];//weight of the conection
    double a[NUM_LEGS];//parameter of sigumoido function
    int swing[NUM_LEGS];//way of swing in second round
    
    int turn;
    int ee;
};

#endif /* GA_h */

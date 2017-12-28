//
//  GA2.h
//  Master
//
//  Created by 増田貴行 on 2017/10/31.
//  Copyright © 2017年 増田貴行. All rights reserved.
//

#ifndef GA2_h
#define GA2_h

#include "GA.h"

class GAmanager {

public:
    GAparameter f;
    GAparameter pool[POOL_SIZE];
    int spiecies_of_starfish;
    
    GAmanager(int);
    GAparameter Adam();
    float evalue(GAparameter p);
    float evalue2(GAparameter p);
    GAparameter first();
    GAparameter CrossOver(GAparameter, GAparameter);
    GAparameter Mutate(GAparameter);
    void CreateNext();
};

#endif /* GA2_h */

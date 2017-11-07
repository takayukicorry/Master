//
//  Ophiuroid.hpp
//  Master
//
//  Created by 増田貴行 on 2017/10/31.
//  Copyright © 2017年 増田貴行. All rights reserved.
//

#ifndef Ophiuroid_hpp
#define Ophiuroid_hpp

#include "Starfish.h"
#include "GA.h"

class Ophiuroid : public Starfish {

private:
    
    
public:
    Ophiuroid();
    
    void idle();
    bool checkState();
    void create();
    float evalue(GAparameter);
    float evalue2(GAparameter);
};

#endif /* Ophiuroid_hpp */

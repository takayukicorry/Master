//
//  GAMain.hpp
//  Master
//
//  Created by 増田貴行 on 2018/01/05.
//  Copyright © 2018年 増田貴行. All rights reserved.
//

#ifndef GAMain_hpp
#define GAMain_hpp

#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <list>
#include <vector>
#include <algorithm>
#include <cmath>
#include <string>
#include "neat.h"
#include "network.h"
#include "population.h"
#include "organism.h"
#include "genome.h"
#include "species.h"
#include "Ophiuroid.hpp"
#include "Ophiuroid2.hpp"
#include "Ophiuroid3.hpp"
#include "GA2.h"

using namespace NEAT;

Population *oph_test_realtime(GAmanager*);
bool oph_evaluate(Organism*, Starfish*);
void oph_realtime_loop(Population*, Starfish*, GAmanager*);

#endif /* GAMain_hpp */

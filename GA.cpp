//
//  GA.cpp
//  Master
//
//  Created by 増田貴行 on 2017/10/31.
//  Copyright © 2017年 増田貴行. All rights reserved.
//

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "GA2.h"
#include "Variables.h"
#include "Ophiroid.hpp"

GAparameter GAmanager::Adam() {
    
    GAparameter param;
    param.cycle = rand()%(MAX_CYCLE-MIN_CYCLE) + MIN_CYCLE;
    for (int i = 0; i < NUM_LEGS; i++)
    {
        param.turn_pattern[i] = 0;
    }
    int a_mas=0;int b_mas=0;
    //    int c_mas=0;
    while (a_mas==b_mas)// || b_mas==c_mas || a_mas==c_mas)
    {
        a_mas = rand()%5;
        b_mas = rand()%5;
        //    c_mas = rand()%5;
        
    }
    param.turn_pattern[a_mas] = 1;
    param.turn_pattern[b_mas] = 1;
    //    param.turn_pattern[c_mas] = 1;
    
    
    /*
     int t = 0;
     int h = 0;
     while (h==0){
     for (int i = 0; i<NUM_LEGS; i++){
     if (rand()%2 == 0){
     param.turn_pattern[i] = 0;
     }else{
     param.turn_pattern[i] = 1;
     t++;
     }
     }
     if(t==NUM_TURN){h=1;}
     }
     */
    for (int i = 0; i<ARRAY_LENGTH; i++)
    {
        //        param.upperlimit[i] = ((rand()%1000)/1000.0)* MAX_ANGLE;
        //        param.lowerlimit[i] = ((rand()%1000)/1000.0)* MIN_ANGLE;
        param.upperlimit[i] = ((rand()%1000)/1000.0)*(MAX_ANGLE - MIN_ANGLE) + MIN_ANGLE;
        param.lowerlimit[i] = ((rand()%1000)/1000.0)*(MAX_ANGLE - MIN_ANGLE) + MIN_ANGLE;
        param.targetpercent[i] = (rand()%1000)/1000.0;
        
        while(param.upperlimit[i] < param.lowerlimit[i])
        {
            param.lowerlimit[i] = ((rand()%1000)/1000.0)*(MAX_ANGLE - MIN_ANGLE) + MIN_ANGLE;
            
        }
    }
    
    //結合の重み（−３〜３）
    for (int i = 0; i<CONECT_LENGTH; i++){
        param.conect[i] = rand()%7 - 3;
    }
    
    //シグモイド関数パラメータ（０．５〜４．０）
    for (int i = 0; i<NUM_LEGS; i++){
        param.a[i] = (1 + rand()%8)* 0.5;
    }
    
    param.turn = 0;
    param.ee = 0;
    
    for (int i = 0; i < NUM_LEGS; i++)
    {
        param.swing[i] = rand()%8;
    }

    
    return param;
}

float GAmanager::evalue(GAparameter p)
{
    Ophiuroid ophiuroid;
    float value = ophiuroid.evalue(p);
    return value;
    
    
    //return p.cycle;
}

float GAmanager::evalue2(GAparameter p)
{
    Ophiuroid ophiuroid;
    float value = ophiuroid.evalue2(p);
    return value;
    
    
    //return p.cycle;
}

GAparameter GAmanager::CrossOver(GAparameter p1,GAparameter p2)
{
    int k = rand()%ARRAY_LENGTH;
    int k2 = rand()%CONECT_LENGTH;
    int k3 = rand()%NUM_LEGS;
    
    GAparameter c;
    c.cycle = ((rand()%1000)/1000.0)*(p1.cycle - p2.cycle) + p2.cycle ;
    for (int i = 0; i<k; i++)
    {
        c.upperlimit[i] = p1.upperlimit[i];
        c.lowerlimit[i] = p1.lowerlimit[i];
        c.targetpercent[i] = p1.targetpercent[i];
    }
    
    for (int i = k; i<ARRAY_LENGTH; i++)
    {
        c.upperlimit[i] = p2.upperlimit[i];
        c.lowerlimit[i] = p2.lowerlimit[i];
        c.targetpercent[i] = p2.targetpercent[i];
    }
    
    int l = rand()%2;
    
    if (l==0){
        for (int i = 0; i<NUM_LEGS; i++){
            c.turn_pattern[i] = p1.turn_pattern[i];
        }
    }else{
        for (int i = 0; i<NUM_LEGS; i++){
            c.turn_pattern[i] = p2.turn_pattern[i];
        }
    }
    
    for (int i = 0; i<k2; i++){
        c.conect[i] = p1.conect[i];
    }
    
    for (int i = k2; i<CONECT_LENGTH; i++){
        c.conect[i] = p2.conect[i];
    }
    
    for (int i = 0; i<k3; i++){
        c.a[i] = p1.a[i];
        c.swing[i] = p1.swing[i];
    }
    
    for (int i = k3; i<NUM_LEGS; i++){
        c.a[i] = p2.a[i];
        c.swing[i] = p2.swing[i];
    }
    
    c.turn = 0;
    c.ee = 0;
    
    
    return c;
}

GAparameter GAmanager::Mutate(GAparameter p)
{
    int pacent = 2;
    
    GAparameter c = p;
    
    if (rand()%100 < pacent)
    {
        c.cycle += rand()%1000-500;
        if(c.cycle<MIN_CYCLE)
        {c.cycle = MIN_CYCLE;
        }
        if(c.cycle>MAX_CYCLE)
        {c.cycle = MAX_CYCLE;
        }
    }
    
    for(int i = 0; i<ARRAY_LENGTH; i++)
    {
        if (rand()%100 < pacent)
        {
            c.upperlimit[i] += ((rand()%1000)/1000.0)*MAX_ANGLE/2.0 - MAX_ANGLE/4.0;
        }
        
        
        if (rand()%100 < pacent)
        {
            c.lowerlimit[i] += 0.5 * ((rand()%1000)/1000.0)*MAX_ANGLE/2.0 - MAX_ANGLE/4.0;
        }
        
        if(c.upperlimit[i] < MIN_ANGLE)
        {
            c.upperlimit[i] = MIN_ANGLE;
        }
        if(c.upperlimit[i] > MAX_ANGLE)
        {
            c.upperlimit[i] = MAX_ANGLE;
        }
        
        if(c.lowerlimit[i] < MIN_ANGLE)
        {
            c.lowerlimit[i] = MIN_ANGLE;
        }
        if(c.lowerlimit[i] > MAX_ANGLE)
        {
            c.lowerlimit[i] = MAX_ANGLE;
        }
        
        /*        while(c.upperlimit[i] < c.lowerlimit[i])
         {
         c.lowerlimit[i] = ((rand()%1000)/1000.0)*(MAX_ANGLE - MIN_ANGLE) + MIN_ANGLE;
         
         }
         */    }
    
    for(int i = 0; i<ARRAY_LENGTH; i++)
    {
        if (rand()%100 < pacent)
        {
            c.targetpercent[i] += ((rand()%1000)/1000.0)*0.2 - 0.1;
        }
        
        
        
        if(c.targetpercent[i] < 0.0)
        {
            c.targetpercent[i] = 0.0;
        }
        if(c.targetpercent[i] > 1.0)
        {
            c.targetpercent[i] = 1.0;
        }
        
    }
    
    
    
    for (int i = 0; i<CONECT_LENGTH; i++){
        if (rand()%100 < pacent){
            c.conect[i] = -c.conect[i];
        }
    }
    
    for (int i = 0; i<NUM_LEGS; i++){
        if (rand()%100 < pacent){
            c.a[i] = (1 + rand()%8) * 0.5;
        }
    }
    for (int i = 0; i<NUM_LEGS; i++){
        if (rand()%100 < pacent){
            c.swing[i] = rand()%8;
        }
    }
    
    return c;
}

GAmanager::GAmanager()
{
    for (int i = 0; i<POOL_SIZE; i++)
    {
        pool[i] = Adam();
    }
    
}

void GAmanager::CreateNext()
{
    GAparameter stock[POOL_SIZE];//pool[]を移す
    float value[POOL_SIZE];//評価値
    float select[POOL_SIZE];//親として選ばれる確率
    
    for (int i = 0; i<POOL_SIZE; i++)
    {
        stock[i] = pool[i];
        value[i] = evalue(stock[i]);
        
    }
    
    
    for (int i = 0; i<POOL_SIZE; i++)
    {
        int num = 1;
        for (int k = 0; k<POOL_SIZE; k++)//順位
        {
            if (value[i] < value[k])
            {
                num++;
            }
        }
        
        if (num == 1)//一位を入れる
        {
            pool[0] = stock[i];
            pool[0].turn = 0;
            pool[0].ee = 0;
        }
        
        select[i] = 1;
        for (int j = 0; j<num; j++)//選択確率
        {
            select[i] = 0.9*select[i];
        }
        
    }
    
    
    for (int i = 1; i<POOL_SIZE; i++)
    {
        int num = 0;//すでに選んだ親の数
        GAparameter p1,p2;
        int k = 0;
        
        while (num == 0)
        {
            if (k >=POOL_SIZE)
            {
                k = 0;
            }
            
            if ((rand()%1000)/1000.0 <= select[k])
            {
                p1 = stock[k];
                num = 1;
            }
            k++;
        }
        
        while (num == 1)
        {
            if (k >=POOL_SIZE)
            {
                k = 0;
            }
            
            if ((rand()%1000)/1000.0 <= select[k])
            {
                p2 = stock[k];
                num = 2;
            }
            k++;
        }
        
        
        pool[i] = CrossOver(p1,p2);
        pool[i] = Mutate(pool[i]);
        
    }
    
    
    
}

GAparameter GAmanager::first()
{
    int i = 0;
    float val = 0;
    for (int k = 0; k<POOL_SIZE; k++)
    {
        if (evalue(pool[k]) > val)
        {
            i = k;
            val = evalue(pool[k]);
        }
        
    }
    
    f = pool[i];
    return pool[i];
}



// -*-c++-*-

/*
 *Copyright:

 Copyright (C) Hidehisa AKIYAMA

 This code is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 3, or (at your option)
 any later version.

 This code is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this code; see the file COPYING.  If not, write to
 the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.

 *EndCopyright:
 */

/////////////////////////////////////////////////////////////////////

#ifndef BHV_BASIC_MOVE_H
#define BHV_BASIC_MOVE_H

#include <rcsc/geom/vector_2d.h>
#include <rcsc/player/soccer_action.h>

class Bhv_BasicMove
    : public rcsc::SoccerBehavior {
public:
    Bhv_BasicMove()
      { }

    bool execute( rcsc::PlayerAgent * agent );
    bool block( rcsc::PlayerAgent * agent );

private:
    double getDashPower( const rcsc::PlayerAgent * agent );
};

class Features{
public:
    static Features * ins;
    static Features * i(const std::string formation_dir = ""){
        if (ins == nullptr){
            ins = new Features(formation_dir);
        }
        return ins;
    }
    int block_number = 1;
    double block_distance = 20;
    double pass_acc = 50;
    double dribble_acc = 50;
    double shoot_acc = 50;
    double pass_eval = 1;
    double dribble_eval = 1;
    double hold_eval = 1;

    Features(const std::string formation_dir);
};

#endif

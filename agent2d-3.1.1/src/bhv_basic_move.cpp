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

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "bhv_basic_move.h"

#include "strategy.h"

#include "bhv_basic_tackle.h"

#include <rcsc/action/basic_actions.h>
#include <rcsc/action/body_go_to_point.h>
#include <rcsc/action/body_intercept.h>
#include <rcsc/action/neck_turn_to_ball_or_scan.h>
#include <rcsc/action/neck_turn_to_low_conf_teammate.h>

#include <rcsc/player/player_agent.h>
#include <rcsc/player/debug_client.h>
#include <rcsc/player/intercept_table.h>

#include <rcsc/common/logger.h>
#include <rcsc/common/server_param.h>
#include <vector>

#include "neck_offensive_intercept_neck.h"

using namespace rcsc;

/*-------------------------------------------------------------------*/
/*!

 */
bool
Bhv_BasicMove::execute( PlayerAgent * agent )
{
    dlog.addText( Logger::TEAM,
                  __FILE__": Bhv_BasicMove" );

    //-----------------------------------------------
    // tackle
    if ( Bhv_BasicTackle( 0.8, 80.0 ).execute( agent ) )
    {
        return true;
    }

    const WorldModel & wm = agent->world();
    /*--------------------------------------------------------*/
    // chase ball
    const int self_min = wm.interceptTable()->selfReachCycle();
    const int mate_min = wm.interceptTable()->teammateReachCycle();
    const int opp_min = wm.interceptTable()->opponentReachCycle();

    if ( ! wm.existKickableTeammate()
         && ( self_min <= 3
              || ( self_min <= mate_min
                   && self_min < opp_min + 3 )
              )
         )
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": intercept" );
        Body_Intercept().execute( agent );
        agent->setNeckAction( new Neck_OffensiveInterceptNeck() );

        return true;
    }

    const Vector2D target_point = Strategy::i().getPosition( wm.self().unum() );
    const double dash_power = Strategy::get_normal_dash_power( wm );

    double dist_thr = wm.ball().distFromSelf() * 0.1;
    if ( dist_thr < 1.0 ) dist_thr = 1.0;

    dlog.addText( Logger::TEAM,
                  __FILE__": Bhv_BasicMove target=(%.1f %.1f) dist_thr=%.2f",
                  target_point.x, target_point.y,
                  dist_thr );

    agent->debugClient().addMessage( "BasicMove%.0f", dash_power );
    agent->debugClient().setTarget( target_point );
    agent->debugClient().addCircle( target_point, dist_thr );

    if ( ! Body_GoToPoint( target_point, dist_thr, dash_power
                           ).execute( agent ) )
    {
        Body_TurnToBall().execute( agent );
    }

    if ( wm.existKickableOpponent()
         && wm.ball().distFromSelf() < 18.0 )
    {
        agent->setNeckAction( new Neck_TurnToBall() );
    }
    else
    {
        agent->setNeckAction( new Neck_TurnToBallOrScan() );
    }

    return true;
}

bool Bhv_BasicMove::block(PlayerAgent *agent)
{
    const WorldModel & wm = agent->world();
    const int self_min = wm.interceptTable()->selfReachCycle();
    const int mate_min = wm.interceptTable()->teammateReachCycle();
    const int opp_min = wm.interceptTable()->opponentReachCycle();
    if (self_min <= opp_min)
        return false;
    if (mate_min <= opp_min)
        return false;
    Vector2D ball_pos = wm.ball().inertiaPoint(opp_min);
    Vector2D dribble_vel = Vector2D::polar2vector(0.8, (Vector2D(-52.5, 0.0) - ball_pos).th());

    double max_dist_to_block = 20;
    int number_of_blocker = 2;
    std::vector<int> blocker_unum;
    std::vector<Vector2D> blocker_pos;

    for(int c = 1; c <= 40; c++){
        ball_pos += dribble_vel;
        int dribble_cycle = c + opp_min;
        for (int t = 2; t <= 11; t++){
            const AbstractPlayerObject * tm = wm.ourPlayer(t);
            if ( tm != nullptr && tm->unum() > 0){
                Vector2D tm_pos = tm->inertiaPoint(dribble_cycle);
                if (tm_pos.dist(ball_pos) > max_dist_to_block)
                    continue;
                int tm_cycle = tm->playerTypePtr()->cyclesToReachDistance(tm_pos.dist(ball_pos) - tm->playerTypePtr()->kickableArea());
                if (tm_cycle <= dribble_cycle){
                    if(std::find(blocker_unum.begin(), blocker_unum.end(), t) != blocker_unum.end()){
                        blocker_unum.push_back(t);
                        blocker_pos.push_back(ball_pos);
                    }
                }
            }
        }
    }
    if(blocker_unum.empty())
        return false;
    for(int i = 0; i < number_of_blocker; i++){
        if (blocker_unum.at(i) == wm.self().unum()){
            Body_GoToPoint(blocker_pos.at(i), 0.1, 100).execute(agent);
            return true;
        }
    }
    return false;
}

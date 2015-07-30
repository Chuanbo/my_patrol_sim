#ifndef MESSAGE_LISTENER_H_
#define MESSAGE_LISTENER_H_

#include <ros/ros.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <string>

#include <set>

#include <stdio.h>

#include <lcm/lcm-cpp.hpp>
#include "exlcm/command_t.hpp"
#include "exlcm/idleness_t.hpp"
#include "exlcm/intention_t.hpp"
#include "exlcm/goal_t.hpp"
#include "exlcm/plan_t.hpp"
#include "exlcm/status_t.hpp"


// 消息处理线程函数  函数声明 
void message_thread();


class Handler 
{
public:
    Handler() 
    {
        robot_id_ = g_robot_id;
        robot_frame_ = tf::resolve(g_tf_prefix, "base_footprint");
        
        srand ( time(NULL) );
        // 通讯可靠性的百分比，完全可靠时 com_rate_ = 100 
        com_rate_ = 100;
        
        // 机器人间通讯范围的限制 
        com_dist_ = 12;
        // com_dist_=12  FOR  2 hops in map grid 
    }
    
    ~Handler() {}
    
    int robot_id_;
    
    std::string robot_frame_;
    
    int com_rate_;  // 通讯可靠性的百分比，完全可靠时 com_rate_ = 100 
    
    int com_dist_;  // 机器人间通讯范围的限制 
    std::set<int> robot_set_;  // 存储在此机器人周围的其它机器人的编号 


    void handleMessageCommand(const lcm::ReceiveBuffer* rbuf,
            const std::string& chan, 
            const exlcm::command_t* command_msg);

    void handleMessageIdleness(const lcm::ReceiveBuffer* rbuf,
            const std::string& chan, 
            const exlcm::idleness_t* idleness_msg);

    void handleMessageIntention(const lcm::ReceiveBuffer* rbuf,
            const std::string& chan, 
            const exlcm::intention_t* intention_msg);
    
    void handleMessageGoal(const lcm::ReceiveBuffer* rbuf,
            const std::string& chan, 
            const exlcm::goal_t* goal_msg);
    
    void handleMessagePlan(const lcm::ReceiveBuffer* rbuf,
            const std::string& chan, 
            const exlcm::plan_t* plan_msg);
    
    void handleMessageState(const lcm::ReceiveBuffer* rbuf,
            const std::string& chan, 
            const exlcm::status_t* status_msg);

};


#endif  // MESSAGE_LISTENER_H_

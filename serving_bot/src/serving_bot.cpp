/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Michael Lautman*/

#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <tf2/LinearMath/Quaternion.h>

#include <sensor_msgs/JointState.h>

#include <tf/transform_listener.h>
#include <find_object_2d/ObjectsStamped.h>
#include <std_msgs/Int32.h>

// Order Listen
#include <std_msgs/String.h>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <math.h>

// tts
#include <stdlib.h>
#include <signal.h>
#include <stdio.h>
#include <simple_speech/script.h>

#define TABLE 0
#define AMERICANO 1
#define LATTE 2
#define MACCHIATO 3

using namespace std;
string line;

class TfExample
{
    public:

    struct objectData{
        std::string name = "";
        double posx = 0;
        double posy = 0;
        double posz = 0;
        int obj_pose = 0;
        bool isSet = false;
    };

    objectData Obj[4];

    int table[10][4];
    int basket[5];
    int orderTable = 0;
    int nowPosition = 2;
    int obj_fnd = 0;

    bool reached = true;
    bool isOrder = false;
    bool arrivedPoint = false;

    TfExample() :
            mapFrameId_("/base_mani"),
            objFramePrefix_("object")
    {
        ros::NodeHandle pnh("~");
        pnh.param("map_frame_id", mapFrameId_, mapFrameId_);
        pnh.param("object_prefix", objFramePrefix_, objFramePrefix_);

        ros::NodeHandle nh;
        subs_obj = nh.subscribe("objectsStamped", 1, &TfExample::objectsDetectedCallback, this);
        subs_order = nh.subscribe("custom_order", 1000, &TfExample::chatterCallback_order,this);
        subs_reached = nh.subscribe("reached", 10, &TfExample::jointReachedCallback,this);
        subs_arrived = nh.subscribe("point_arrival", 10, &TfExample::arrivedCallback,this);

        joint_control_pub = nh.advertise<sensor_msgs::JointState>("move_group/fake_controller_joint_states",50);
        gripper_on_pub = nh.advertise<std_msgs::Int32>("gripper_on",50);
        gripper_off_pub = nh.advertise<std_msgs::Int32>("gripper_off",50);
        table_pub = nh.advertise<std_msgs::Int32>("table_num",50);
        posture_pub = nh.advertise<std_msgs::Int32>("motor1_first_posture",50);

        joint_states.header.frame_id="base_mani";

        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

        mymodel = robot_model_loader.getModel();
        ROS_INFO("Model frame: %s", mymodel->getModelFrame().c_str());


        for(int t = 0 ; t<10; t++)
            for(int d = 0; d<3 ; d++)
                table[t][d] = 0;
        for(int t = 0 ; t<5; t++)
            basket[t]=0;
        grip_speed.data = 220;
        base_x = 0;
        base_y = 0;
    }

    void chatterCallback_order(const std_msgs::String::ConstPtr& msg){      //Get order data
        ROS_INFO("order : [%s]", msg->data.c_str());
        line = msg->data.c_str();
        vector<string> line_vector = split(line, ',');
        int table_num = atoi(line_vector[TABLE].c_str());

        table[table_num][AMERICANO] = atoi(line_vector[AMERICANO].c_str());
        table[table_num][LATTE] = atoi(line_vector[LATTE].c_str());
        table[table_num][MACCHIATO] = atoi(line_vector[MACCHIATO].c_str());

        orderTable = table_num;
        table_order.data = orderTable;
        isOrder = true;
    }

    void arrivedCallback(const std_msgs::Int32::ConstPtr& msg){         //Get arrival state data
        arrivedPoint = true;
        int tmp = msg->data;
        nowPosition = tmp;

        if(tmp == 1){
            speech("Arrived Table");
        }
        if(tmp == 2){
            speech("Arrived Home");
        }
    }


    void jointReachedCallback(const std_msgs::Int32::ConstPtr& msg){        //Get joint reach state data
       reached = true;
    }

    void objectsDetectedCallback(const find_object_2d::ObjectsStampedConstPtr & msg)        //Get object data when reached all of joints
    {
        if(!reached)
            return;

        if(msg->objects.data.size())
        {
            char multiSubId = 'b';
            int previousId = -1;
            for(unsigned int i=0; i<msg->objects.data.size(); i+=12)
            {
                // get data
                int id = (int)msg->objects.data[i];

                std::string whatis;

                if(id<=5) whatis = "Americano";
                else if(id<=10) whatis = "Latte";
                else if(id<=15) whatis = "Macchiato";
                else whatis = "?";

                std::string objectFrameId = whatis.c_str();

                tf::StampedTransform pose;
                tf::StampedTransform poseCam;
                try
                {
                        tfListener_.lookupTransform(mapFrameId_, objectFrameId, msg->header.stamp, pose);
                        tfListener_.lookupTransform(msg->header.frame_id, objectFrameId, msg->header.stamp, poseCam);
                }
                catch(tf::TransformException & ex)
                {
                        continue;
                }

                double d = this->distance(poseCam.getOrigin().x(),poseCam.getOrigin().y(),poseCam.getOrigin().z());

                ROS_INFO("Get Drink id is %d, distance is %lf",id,d);
                if (d>0.65 || pose.getOrigin().z()<0.30) continue;

                if(strcmp(whatis.c_str(),"Americano")==0){
                    this->Obj[AMERICANO].name = whatis;
                    this->Obj[AMERICANO].posx = pose.getOrigin().x();
                    this->Obj[AMERICANO].posy = pose.getOrigin().y();
                    this->Obj[AMERICANO].posz = pose.getOrigin().z();
                    this->Obj[AMERICANO].obj_pose = obj_fnd;
                    this->Obj[AMERICANO].isSet = true;
                }
                if(strcmp(whatis.c_str(),"Latte")==0){
                    this->Obj[LATTE].name = whatis;
                    this->Obj[LATTE].posx = pose.getOrigin().x();
                    this->Obj[LATTE].posy = pose.getOrigin().y();
                    this->Obj[LATTE].posz = pose.getOrigin().z();
                    this->Obj[LATTE].obj_pose = obj_fnd;
                    this->Obj[LATTE].isSet = true;
                }
                if(strcmp(whatis.c_str(),"Macchiato")==0){
                    this->Obj[MACCHIATO].name = whatis;
                    this->Obj[MACCHIATO].posx = pose.getOrigin().x();
                    this->Obj[MACCHIATO].posy = pose.getOrigin().y();
                    this->Obj[MACCHIATO].posz = pose.getOrigin().z();
                    this->Obj[MACCHIATO].obj_pose = obj_fnd;
                    this->Obj[MACCHIATO].isSet = true;
                }
            }
        }
    }

    void grip(int speed){           //Control gripper
        if(speed>0){
            grip_speed.data = speed;
            gripper_on_pub.publish(grip_speed);
        }
        else{
            grip_speed.data = -1*speed;
            gripper_off_pub.publish(grip_speed);
        }
    }

    void posture(void){             //Set the motor1 to first posture
        posture_pub.publish(grip_speed);
        ros::WallDuration(2.0).sleep();
    }

    bool Get_Drink(int drink){      //Bring drink when correct with order
        int error_count = 1;

        while(true){
            if(Obj[drink].isSet){
                pose = Make_Position(Obj[drink].posx,Obj[drink].posy,Obj[drink].posz+0.025);
                break;
            }
            else if(error_count==75){
                obj_fnd = 0;
                speech("I can't find object.");
                reached = false;
                joint_states.position = {-2.348,1.629,0.813,0.0};
                joint_control_pub.publish(joint_states);
                return false;
            }
            else if(error_count==45){
                obj_fnd = 1;
                reached = false;
                joint_states.position = {-2.348,1.629,0.813,0.5};
                joint_control_pub.publish(joint_states);
            }
            else if(error_count==15){
                obj_fnd = -1;
                reached = false;
                joint_states.position = {-2.348,1.629,0.813,-0.5};
                joint_control_pub.publish(joint_states);
            }
            ros::WallDuration(0.20).sleep();
            error_count++;
        }

        const robot_state::JointModelGroup* joint_model_group = mymodel->getJointModelGroup("manipulator");
        robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(mymodel));
        const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

        bool found_ik = kinematic_state->setFromIK(joint_model_group, pose, 10, 1);

        const static double joint_basket_1[5][4] = {{-2.602,-1.807,1.486,0.323},
                                                    {-2.865,-1.781,1.402,0.377},
                                                    {-3.130,-1.779,1.403,0.373},
                                                    {-3.390,-1.795,1.471,0.322},
                                                    {-3.660,-1.807,1.486,0.323}};     //first basket position

        const static double joint_basket_2[5][4] = {{-2.602,-1.838,1.731,0.109},
                                                    {-2.865,-1.826,1.719,0.104},
                                                    {-3.130,-1.823,1.721,0.100},
                                                    {-3.390,-1.826,1.715,0.109},
                                                    {-3.660,-1.837,1.726,0.112}};

        int bask_num = 0;
        int reachedCount = 0;

        for(int i = 0;i<5;i++){
            if(i%2==0 && basket[i] == 0){
                basket[i] = orderTable;
                bask_num = i;
                break;
            }
            else if(basket[4] != 0 && i%2 == 1 && basket[i] == 0){
                basket[i] = orderTable;
                bask_num = i;
                break;
            }
            else if(basket[3] != 0){
                speech("Basket is Full!");
                return false;
            }
        }

        ROS_INFO("Find Inverse Kinematics to %s",Obj[drink].name.c_str());

        if (found_ik)
        {
            kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

            pose = Make_Position(Obj[drink].posx*3/4,Obj[drink].posy*3/4,Obj[drink].posz);
            bool found_ik2 = kinematic_state->setFromIK(joint_model_group, pose, 10, 1);
            kinematic_state->copyJointGroupPositions(joint_model_group, pre_joint_values);

            if(!found_ik2)
                return false;

            for (std::size_t i = 0; i < joint_names.size(); ++i)
            {
              ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
            }
            while(true){
                if(!reached){
                    ros::WallDuration(0.000001).sleep();        //make delay to change flag
                    continue;
                }

                ros::WallDuration(0.1).sleep();
                reachedCount++;

                if(reachedCount == 1){        //ready position to catch
                    speech("Get drink.");         //make sound
                    joint_states.position = {pre_joint_values[1],pre_joint_values[2],pre_joint_values[3],pre_joint_values[0]+0.5*Obj[drink].obj_pose};
                    joint_control_pub.publish(joint_states);
                }
                else if(reachedCount == 2){   //get position
                    joint_states.position = {joint_values[1],joint_values[2],joint_values[3],joint_values[0]+0.5*Obj[drink].obj_pose};
                    joint_control_pub.publish(joint_states);
                }
                else if(reachedCount == 3){   //get and return
                    grip(100);
                    ros::WallDuration(0.5).sleep();
                    grip(220);
                    ros::WallDuration(1.5).sleep();
                    joint_states.position = {-2.155,1.250,0.914,joint_values[0]+0.5*Obj[drink].obj_pose};
                    joint_control_pub.publish(joint_states);
                }
                else if(reachedCount == 4){   //get position
                        joint_states.position = {-2.028,1.074,0.972,joint_basket_1[bask_num][0]};
                        joint_control_pub.publish(joint_states);
                    }
                else if(reachedCount == 5){   //get position
                    joint_states.position = {joint_basket_1[bask_num][1],joint_basket_1[bask_num][2],
                                             joint_basket_1[bask_num][3],joint_basket_1[bask_num][0]};
                    joint_control_pub.publish(joint_states);
                }
                else if(reachedCount == 6){
                    joint_states.position = {joint_basket_2[bask_num][1],joint_basket_2[bask_num][2],
                                             joint_basket_2[bask_num][3],joint_basket_2[bask_num][0]};//joint_basket[0]+basket_offset
                    joint_control_pub.publish(joint_states);
                }
                else if(reachedCount == 7){
                    grip(-200);
                    ros::WallDuration(2).sleep();
                    joint_states.position = {-2.099,1.814,0.302,joint_basket_1[bask_num][0]};
                    joint_control_pub.publish(joint_states);
                }
                else if(reachedCount == 8){
                    joint_states.position = {-2.348,1.629,0.813,0.0};
                    joint_control_pub.publish(joint_states);
                }
                else if(reachedCount == 9){
                    Obj[drink].isSet = false;
                    posture();
                    return true;
                }
                reached = false;
            }
        }
        else
        {
          ROS_INFO("Did not find IK solution");
        }
        return false;
    }

    void Give_Drink(int bask_num){              //Give drink on the ordered table
        if(basket[bask_num] != orderTable)
            return;

        basket[bask_num] = 0;

        int reachedCount = 0;

        const static double joint_basket[5] = {-2.603, -2.870,-3.136,-3.397,-3.660};        // rotate joint
        const static double joint_basket_get[4] = {-2.603,-1.830,1.727,0.136};
        const static double joint_basket_up[4] = {-2.603,-1.700,0.710,0.910};


        const static double joint_give_above[4] =  {0.400,-1.450,0.970,0.454};
        const static double joint_give_table[5][4] = {{0.400,-1.310,0.971,0.315},
                                                      {0.200,-1.420,1.084,0.321},
                                                      {0.000,-1.410,1.083,0.301},
                                                      {-0.200,-1.420,1.084,0.321},
                                                      {-0.400,-1.310,0.971,0.315}};
        const static double joint_give_after[4] = {0.400,-1.855,1.554,0.298};

        while(true){
            if(!reached){
                ros::WallDuration(0.000001).sleep();        //make delay to change flag
                continue;
            }

            ros::WallDuration(0.1).sleep();
            reachedCount++;

            if(reachedCount == 1){        //ready position to catch
                joint_states.position = {-2.028,1.074,0.972,joint_basket[bask_num]};
                joint_control_pub.publish(joint_states);
            }
            else if(reachedCount == 2){   //get position
                joint_states.position = {-2.189,1.882,0.239,joint_basket[bask_num]};
                joint_control_pub.publish(joint_states);
            }
            else if(reachedCount == 3){   //get and return
                joint_states.position = {joint_basket_get[1],joint_basket_get[2],
                                         joint_basket_get[3],joint_basket[bask_num]};
                joint_control_pub.publish(joint_states);
            }
            else if(reachedCount == 4){   //get position
                grip(100);
                ros::WallDuration(0.5).sleep();
                grip(220);
                ros::WallDuration(1).sleep();
                joint_states.position = {joint_basket_up[1],joint_basket_up[2],
                                         joint_basket_up[3],joint_basket[bask_num]};
                joint_control_pub.publish(joint_states);

                if(basket[1] == 0)
                    bask_num = (bask_num+2)/2;
            }
            else if(reachedCount == 5){
                joint_states.position = {-1.988,1.033,1.005,joint_give_above[0]-0.2*(double)bask_num};
                joint_control_pub.publish(joint_states);
            }
            else if(reachedCount == 6){
                joint_states.position = {joint_give_above[1],joint_give_above[2],
                                         joint_give_above[3],joint_give_above[0]-0.2*(double)bask_num};
                joint_control_pub.publish(joint_states);
            }
            else if(reachedCount == 7){
                joint_states.position = {joint_give_table[bask_num][1],joint_give_table[bask_num][2],
                                         joint_give_table[bask_num][3],joint_give_table[bask_num][0]};
                joint_control_pub.publish(joint_states);
            }
            else if(reachedCount == 8){
                grip(-200);
                ros::WallDuration(2).sleep();
                joint_states.position = {joint_give_after[1],joint_give_after[2],
                                         joint_give_after[3],joint_give_after[0]-0.2*(double)bask_num};
                joint_control_pub.publish(joint_states);
            }
            else if(reachedCount == 9){
                joint_states.position = {-2.348,1.629,0.813,-0.05};
                joint_control_pub.publish(joint_states);
            }
            else if(reachedCount == 10){
                posture();
                return;
            }
            reached = false;
        }
    }

    void Go_Table(void){            //Navigate to order table
        speech("Go to table");
        nowPosition = 0;
        table_order.data = 1;
        table_pub.publish(table_order);
    }

    void Go_Home(void){             //Navigate to home
        speech("Go to home");
        nowPosition = 0;
        table_order.data = 0;
        table_pub.publish(table_order);
    }

    void speech(string script)     //Make sound in script
    {
        char cmd[30];
        char say[30];
        char tts[200];
        int r;

        strcpy(cmd,"pico2wave --wave ~/say.wav ");
        strcpy(say,"aplay ~/say.wav");

        snprintf(tts, 200, "%s\"%s\"", cmd, script.c_str());

        r = system(tts);  // make the wav file
        r = system(say);  // play the wav file
    }

    vector<string> split(string str, char delimiter) {      //Split the order data
        vector<string> internal;
        stringstream ss(str);
        string temp;

        while (getline(ss, temp, delimiter)) {
            internal.push_back(temp);
        }
        return internal;
    }

    geometry_msgs::Pose Make_Position(double tx,double ty,double tz){       //Set object position for matching with gripper TF
        geometry_msgs::Pose tpose;
        tpose.position.x = tx; //0.26
        tpose.position.y = ty;
        tpose.position.z = tz;

        double yaw = atan2(ty - base_y, tx - base_x);

        tf2::Quaternion q;
        q.setRPY(0,0,yaw);
        tpose.orientation.x = q[0];
        tpose.orientation.y = q[1];
        tpose.orientation.z = q[2];
        tpose.orientation.w = q[3];

        return tpose;
    }

    double distance(double x, double y, double z){
        return sqrt(x*x + y*y + z*z);
    }

private:
        std::string mapFrameId_;
        std::string objFramePrefix_;
    ros::Subscriber subs_obj,subs_order,subs_reached,subs_arrived;
    ros::Publisher joint_control_pub,gripper_on_pub,gripper_off_pub,table_pub,posture_pub;
    tf::TransformListener tfListener_;
    robot_model::RobotModelPtr mymodel;
    std_msgs::Int32 grip_speed,table_order;
    std::vector<double> joint_values,pre_joint_values;
    sensor_msgs::JointState joint_states;
    geometry_msgs::Pose pose;

    bool moving;
    double base_x,base_y;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "manipulator Inverse Kinematics");
    ros::AsyncSpinner spinner(1);
    TfExample sync;
    ros::NodeHandle nh;

    spinner.start();

    sync.speech("Ready to order");         //make sound

    while(true){
        if(sync.nowPosition == 1){       //Robot in table
            sync.speech("I give drinks that you orderd.");         //make sound
            for(int i=0;i<5;i++){
                if(sync.basket[i] != 0){
                    sync.Give_Drink(i);
                }
            }
            sync.Give_Drink(0);
            sync.Go_Home();
        }
        else if(sync.nowPosition == 2){       // Robot in home
            if(!sync.isOrder){
                ros::WallDuration(0.000001).sleep();        //make delay to change flag
                continue;
            }
            ROS_INFO("Table%d : %d %d %d",sync.orderTable,sync.table[sync.orderTable][AMERICANO],
                         sync.table[sync.orderTable][LATTE],sync.table[sync.orderTable][MACCHIATO]);

            if(sync.table[sync.orderTable][AMERICANO]>0){
                sync.Get_Drink(AMERICANO);
                sync.table[sync.orderTable][AMERICANO]--;
            }
            else if(sync.table[sync.orderTable][LATTE]>0){
                sync.Get_Drink(LATTE);
                sync.table[sync.orderTable][LATTE]--;
            }
            else if(sync.table[sync.orderTable][MACCHIATO]>0){
                sync.Get_Drink(MACCHIATO);
                sync.table[sync.orderTable][MACCHIATO]--;
            }
            else{
                sync.Go_Table();
                sync.isOrder= false;
            }
        }

        else
            ros::WallDuration(0.000001).sleep();        //make delay to change flag

    }

    return 0;
}

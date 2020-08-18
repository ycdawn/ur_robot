#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>


bool setArmJointState(std::string preset_state, moveit::planning_interface::MoveGroupInterface& move_group)
{
    
    std::vector<double> joint_group_positions = move_group.getCurrentJointValues();
    
    if(preset_state=="home")
    {
        joint_group_positions[0]= 0;
        joint_group_positions[1]= -M_PI_2;
        joint_group_positions[2]= 0;
        joint_group_positions[3]= 0;
        joint_group_positions[4]= 0;
        joint_group_positions[5]= -M_PI_2;
    }
    else if(preset_state=="prepick")
    {
        joint_group_positions[0]= 0;
        joint_group_positions[1]= -1;
        joint_group_positions[2]= 1;
        joint_group_positions[3]= -M_PI_2;
        joint_group_positions[4]= -M_PI_2;
        joint_group_positions[5]= 0;
    }
    else if(preset_state=="pickup")
    {
        joint_group_positions[0]= 1;
        joint_group_positions[1]= -M_PI_2;
        joint_group_positions[2]= 1;
        joint_group_positions[3]= -M_PI_2;
        joint_group_positions[4]= -M_PI_2;
        joint_group_positions[5]= 0;
    }
    else ROS_ERROR("NO preset_state find");

    //应用设置的关节状态值
    move_group.setJointValueTarget(joint_group_positions);

    //关节空间路径规划
    moveit::planning_interface::MoveGroupInterface::Plan joint_plan;

    //显示规划是否成功
    bool success = (move_group.plan(joint_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    char* setted_state = (char*)preset_state.c_str();
    success = (move_group.plan(joint_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("arm state plan to '%s' %s", setted_state, success ? "success" : "FAILED");
    
    //执行运动
    move_group.move();

    return success;

}

void showArmState(moveit::planning_interface::MoveGroupInterface& move_group)
{
    
    
    std::vector<double> joint_group_positions = move_group.getCurrentJointValues();

    std::cout<<"\033[32mcurrent robot joint statet：\033[0m"<<std::endl
             <<"shoulder_pan_joint: " <<joint_group_positions[0]<<"\t"
             <<"shoulder_lift_joint: " <<joint_group_positions[1]<<"\t"
             <<"elbow_joint: "<<joint_group_positions[2]<<"\t"
             <<"wrist_1_joint: " <<joint_group_positions[3]<<"\t"
             <<"wrist_2_joint: " <<joint_group_positions[4]<<"\t"
             <<"wrist_3_joint: " <<joint_group_positions[5]<<std::endl
             <<"finger_joint: " <<joint_group_positions[6]<<std::endl;
}


bool setEndPose(moveit::planning_interface::MoveGroupInterface& move_group,
                double px=0.08,double py=0.22,double pz=0.60,
                double ox=-0.5,double oy=0.5,double oz=0.5,double ow=0.5
                )
{
    geometry_msgs::Pose target_pose;

    //默认为home位置的终端位姿

    target_pose.position.x = px;
    target_pose.position.y = py;
    target_pose.position.z = pz;

    target_pose.orientation.x = ox;
    target_pose.orientation.y = oy;
    target_pose.orientation.z = oz;
    target_pose.orientation.w = ow;

    /*   **pickup pose** 
    target_pose1.position.x = 0.0743588;
    target_pose1.position.y = 0.32371;
    target_pose1.position.z = 0.461444;

    target_pose1.orientation.x = -0.459974;
    target_pose1.orientation.y = 0.842118;
    target_pose1.orientation.z = 0.135004;
    target_pose1.orientation.w = 0.247051;
    */

    move_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan pose_plan;
    bool success = (move_group.plan(pose_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("end pose plan %s",  success ? "success" : "FAILED");

    move_group.move();
    
    return success;
}

void showEndPose(moveit::planning_interface::MoveGroupInterface& move_group)
{
    geometry_msgs::PoseStamped endlink_pose =move_group.getCurrentPose();
    std::cout   <<"\033[32mcurrent end link pose：\033[0m"<<std::endl
                <<endlink_pose<<std::endl;
}

void setGripper(double gripper_joint_value)
{
    moveit::planning_interface::MoveGroupInterface gripper("gripper");

    std::vector<double> gripper_joint = gripper.getCurrentJointValues();
    std::cout<<"\033[32mcurrent gripper joint statet：\033[0m"<<std::endl
             <<gripper_joint[0]<<std::endl;
            //  <<gripper_joint[1]<<std::endl
            //  <<gripper_joint[2]<<std::endl
            //  <<gripper_joint[3]<<std::endl
            //  <<gripper_joint[4]<<std::endl
            //  <<gripper_joint[5]<<std::endl;

    for(int i=0;i<6;i++)gripper_joint[i]=gripper_joint_value;
    for(int i=2;i<5;i++)gripper_joint[i]=-gripper_joint_value;//gripper总共6个joint，第3、4、5异号；
    
    gripper.setJointValueTarget(gripper_joint);
    gripper.move();
    // std::cout<<"\033[32mcurrent gripper joint statet：\033[0m"<<std::endl
    //          <<gripper_joint[0]<<std::endl
            //  <<gripper_joint[1]<<std::endl
            //  <<gripper_joint[2]<<std::endl
            //  <<gripper_joint[3]<<std::endl
            //  <<gripper_joint[4]<<std::endl
            //  <<gripper_joint[5]<<std::endl;
    ROS_INFO("set gripper value to %f",gripper_joint_value);
    

}





void setPathConstraints(moveit::planning_interface::MoveGroupInterface& move_group)
{
    //限定终端的方向,要求精确合适的方向数值
    //实测没啥卵用... 数值不对各种鬼畜
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = move_group.getEndEffectorLink().c_str();
    ocm.header.frame_id = move_group.getPlanningFrame().c_str();
    ocm.orientation.x = 0;
    ocm.orientation.y = 0;
    ocm.orientation.z = 0;
    ocm.orientation.w = 1;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;

    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    move_group.setPathConstraints(test_constraints);

}

void CartesianPath(moveit::planning_interface::MoveGroupInterface& move_group)
{
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose start_pose =move_group.getCurrentPose().pose;

    //设置当前终端位置为路点起点
    waypoints.push_back(start_pose);

    geometry_msgs::Pose target_pose1 = start_pose;
    target_pose1.position.z -= 0.2;
    waypoints.push_back(target_pose1);  // down

    target_pose1.position.y -= 0.2;
    waypoints.push_back(target_pose1);  // right

    target_pose1.position.z += 0.2;
    target_pose1.position.y += 0.2;
    target_pose1.position.x -= 0.2;
    waypoints.push_back(target_pose1);  // up and left

    //move_group.setMaxVelocityScalingFactor(0.1);

    moveit_msgs::RobotTrajectory trajectory;

    const double jump_threshold = 0.0;
    //Warning！ - disabling the jump threshold while operating real hardware can cause large unpredictable motions of redundant joints and could be a safety issue

    const double eef_step = 0.01;//终端步进值（分辨率）
    double fraction =0; 
    
    //笛卡尔规划直至规划到全部路点或超过最大尝试次数
    //replan没啥卵用  如果不行重新规划后还是不行
    // int attempts =0;
    // while (fraction<0.99 && attempts < 5)
    // {
    //     fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    //     attempts +=1;
    //     ROS_INFO("cartesian plan trying after %d attempts...",attempts);
    // }
    fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    move_group.execute(trajectory);
    ROS_INFO("plan Cartesian path done (%.2f%% acheived)", fraction * 100.0);

}

moveit_msgs::CollisionObject addObject(
                    moveit::planning_interface::MoveGroupInterface& move_group,
                    const std::string object_id, double dx =1,double dy =1,double dz=1,
                    double px=1,double py=1,double pz =1,double ow =1)
{
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();

    // The id of the object is used to identify it.
    collision_object.id = object_id;

    // 定义物体的类型、size
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = dx;
    primitive.dimensions[1] = dy;
    primitive.dimensions[2] = dz;

    // 定义物体的位置(specified relative to frame_id)
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = ow;
    box_pose.position.x = px;
    box_pose.position.y = py;
    box_pose.position.z = pz;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    ROS_INFO("Add object '%s' ",collision_object.id.c_str());

    return collision_object;
    
}



int main(int argc ,char** argv)
{
    ros::init(argc, argv, "cpp_interface");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //##setup##
    static const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface arm(PLANNING_GROUP);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup* joint_model_group =
        arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    arm.allowReplanning(true);

    ROS_INFO("planning frame:%s",arm.getPlanningFrame().c_str());
    ROS_INFO("initialization completed");
    //##setup end##



    //##pick and place demo##

    
    //1.加入物体到scene
    moveit_msgs::CollisionObject plane = 
                addObject(arm,"workstation",2,2,0.001,0,0,-0.015);
    moveit_msgs::CollisionObject box = 
                addObject(arm,"box1",0.05,0.05,0.1,0.42,0.11,0.05);

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(plane);
    collision_objects.push_back(box);
    
    planning_scene_interface.addCollisionObjects(collision_objects);
    ROS_INFO("add object to world");
    arm.setSupportSurfaceName("workstation");//设置支撑面以放置box

    //2.将夹爪张开
    setGripper(0.00);

    //3.机械臂运动到预抓取位置
    setArmJointState("prepick",arm);
    
    //4.将夹爪收紧，显示当前终端姿态
    setGripper(0.38);
    showEndPose(arm);

    //5.抓取（将box和夹爪连接，arm运动即可带动box）
    arm.attachObject(box.id,"left_inner_finger_pad");
    
    //6.机械臂运动到空中预放置位置
    setArmJointState("pickup",arm);
    ROS_INFO("finnish pick");
    ros::WallDuration(1.0).sleep();

    //7.机械臂运动到放置位置
    setEndPose(arm,0.12,0.43,0.24,0,-1,0,0);//place pose

    //8.松开物体，完成放置
    arm.detachObject(box.id);
    setGripper(0.00);
    ROS_INFO("finnish pick");
    ros::WallDuration(1.0).sleep();

    //9.回到初始位置
    setArmJointState("home",arm);
    ROS_INFO("back home");    
    ros::WallDuration(2.0).sleep();

    //10.清除scene中的物体
    std::vector<std::string> remove_object_ids;
    remove_object_ids.push_back(box.id);
    remove_object_ids.push_back(plane.id);
    planning_scene_interface.removeCollisionObjects(remove_object_ids);
    ROS_INFO("clear scene");

    
    
}
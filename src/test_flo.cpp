#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/JointState.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/RobotState.h>

#include <tf/transform_broadcaster.h>

#include <vector>

#include <msp/Params.h>
#include <msp/State.h>
#include <msp/Node.h>
#include <msp/Tree.h>
#include <msp/MSP.h>
#include <iostream>     // std::cout
#include <iomanip>
#include <fstream>
#include <ctime>

ros::Publisher planning_scene_diff_publisher;
ros::Publisher joint_states;
planning_scene::PlanningScenePtr scene;
collision_detection::CollisionRequest collision_request;
collision_detection::CollisionResult collision_result;
const moveit::core::JointModelGroup* group;
robot_state::RobotState* current_state;
std::vector<double> group_variable_values;
robot_model::RobotModelPtr kinematic_model;

bool checkCollision(robot_state::RobotState& st){
	//collision_result.clear();
	//scene->checkCollision(collision_request, collision_result,st);
	//return collision_result.collision;
	return !st.satisfiesBounds(group) || scene->isStateColliding(st);
}

void setState(robot_state::RobotState& st,std::vector<double> group_variable_values){
	st.setJointGroupPositions(group,group_variable_values);
}

template <unsigned int DIM> bool isObstacle(State<DIM> s){
	for(int i=0;i<DIM;++i){
			group_variable_values[i]=s[i];
	}
	setState(*current_state,group_variable_values);
	return checkCollision(*current_state);
}

int Ocount=0;
time_t timerStart;
/*
template <unsigned int DIM> bool addObstacles(State<DIM> s, int depth, float scale, Tree<DIM>* t){
	if(depth==t->getMaxDepth()){
		Ocount++;
		if(Ocount%((int)pow(2,8))==0){
			std::cout << "\r" << "Obstacle creation " << std::setw(10) << Ocount*100.0/pow(2,DIM*depth) << "\% done, ";
			time_t timerNow=time(NULL);	
			int seconds = (int)difftime(timerNow,timerStart);
			seconds=(int)(seconds*(1-( Ocount/pow(2,DIM*depth)))/( Ocount/pow(2,DIM*depth)));
			int hours = seconds/3600;
			int minutes= seconds/60;
			seconds=seconds%60;
			std::cout << "time left: " 	<< std::setw(4) << hours << ":" 
							<< std::setw(2) << minutes << ":" 
							<< std::setw(2) << seconds;
		}
		//finest resolution: update obstacle presence
		//if obstacles
		if(isObstacle(s)){
			//add obstacle to the tree
			t->addObstacle(s);
			//indicate that the tree was updated
			return true;
		}else{
			//indicate that not changes were performed on the tree
			return false;
		}
	}else{
		bool update=false;
		//update children
		for(const State<DIM>& dir: *(t->getDirections())){
			update=addObstacles(s+dir*(0.5f*scale),depth+1,0.5f*scale,t) || update;
		}
		//if any children created, get node
		if(update){
			Node<DIM>* cur=t->getNode(s,depth);
			//prune and update val (single stage, no recurrence)
			cur->update(false);
		}
		//indicate if updates were performed on the tree
		return update;
	}
}
*/
template <unsigned int DIM> bool addObstacles(Key<DIM> k, int depth, int size, Tree<DIM>* t){
	if(depth==t->getMaxDepth()){
		Ocount++;
		if(Ocount%((int)pow(2,8))==0){
			std::cout << "\r" << "Obstacle creation " << std::setw(10) << Ocount*100.0/pow(2,DIM*depth) << "\% done, ";
			time_t timerNow=time(NULL);	
			int seconds = (int)difftime(timerNow,timerStart);
			seconds=(int)(seconds*(1-( Ocount/pow(2,DIM*depth)))/( Ocount/pow(2,DIM*depth)));
			int hours = seconds/3600;
			int minutes= seconds/60;
			seconds=seconds%60;
			std::cout << "time left: " 	<< std::setw(4) << hours << ":" 
							<< std::setw(2) << minutes << ":" 
							<< std::setw(2) << seconds;
		}
		//finest resolution: update obstacle presence
		//if obstacles
		if(isObstacle(t->getState(k))){
			//add obstacle to the tree
			t->addObstacle(k);
			//indicate that the tree was updated
			return true;
		}else{
			//indicate that not changes were performed on the tree
			return false;
		}
	}else{
		bool update=false;
		//update children
		int size2=size>>1;
		 for(const Key<DIM>& dir: *(t->getDirections())){
			 update=addObstacles(k+dir*size2,depth+1,size2,t) || update;
		 }
		//if any children created, get node
		 if(update){
			 Node<DIM>* cur=t->getNode(k);
			 //prune and update val (single stage, no recurrence (children are up to date))
			 cur->update(false);
		 }
		 //indicate if updates were performed on the tree
		 return update;
	}
}

void addObstaclesToPlanningScene(){
	moveit_msgs::PlanningScene planning_scene_msg;
	planning_scene_msg.is_diff = false;
	std::ifstream file("/home/florian/workspace/catkin_ws/src/plannerarena/problems/pr2_scenes/bookshelves.scene");
	scene->loadGeometryFromStream(file);
	file.close();
	scene->getPlanningSceneMsg(planning_scene_msg);
	planning_scene_diff_publisher.publish(planning_scene_msg);
}

void publishState(robot_state::RobotState& st){
	sensor_msgs::JointState joint_msg;
	moveit::core::robotStateToJointStateMsg(st,joint_msg);
	joint_states.publish(joint_msg);
}

#define AD 4
int main(int argc, char **argv)
{
	//Init all the variables
	ros::init (argc, argv, "right_arm_kinematics");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::Duration sleeper5(5);
	ros::NodeHandle node_handle;
	//publisher for world changes
	planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
	joint_states = node_handle.advertise<sensor_msgs::JointState>("joint_states",1);
	//load robot model
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	kinematic_model = robot_model_loader.getModel();
	scene=planning_scene::PlanningScenePtr(new planning_scene::PlanningScene(kinematic_model));
	while(planning_scene_diff_publisher.getNumSubscribers() < 1 && joint_states.getNumSubscribers() < 1)
	{
		ros::WallDuration sleep_t(0.5);
		sleep_t.sleep();
	}
	//robot state, change robot state
	current_state = &(scene->getCurrentStateNonConst());
	//set base position
	group=kinematic_model->getJointModelGroup("base");
	current_state->copyJointGroupPositions(group,group_variable_values);
	group_variable_values[0] = 0.13; //set x to be close from the bookshelves
	group_variable_values[1] = 0.35; //set y
	setState(*current_state,group_variable_values);
	//publishState(*current_state);
	static tf::TransformBroadcaster br;
	static tf::Transform transform;
	transform.setOrigin( tf::Vector3(group_variable_values[0], group_variable_values[1], 0.0) );
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	transform.setRotation(q);
	ros::Timer timer = node_handle.createTimer(ros::Duration(0.1), [](const ros::TimerEvent&){br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/odom_combined", "base_footprint"));});
	//add Obstacles
	sleeper5.sleep();
	sleeper5.sleep();
	addObstaclesToPlanningScene();
	//set left arm position
	group=kinematic_model->getJointModelGroup("left_arm");
	current_state->copyJointGroupPositions(group,group_variable_values);
	//group_variable_values[0] = 1.0;
	group_variable_values[1] = 1.2;
	group_variable_values[3] = -2.1;
	//group_variable_values[3] = 0.0;
	setState(*current_state,group_variable_values);
	publishState(*current_state);
	//set current group to right arm for planning
	group=kinematic_model->getJointModelGroup("right_arm");
	current_state->copyJointGroupPositions(group,group_variable_values);
	for(int i=0;i<group_variable_values.size();++i){
		group_variable_values[i] = 0.0;
	}
	group_variable_values[1]=-0.35;
	group_variable_values[2]=-3.14;
	group_variable_values[3]=-0.21;
	group_variable_values[4]=-3.1;
	group_variable_values[5]=-0.5;
	setState(*current_state,group_variable_values);
	publishState(*current_state);
	sleeper5.sleep();
	sleeper5.sleep();
	moveit::core::JointBoundsVector bvec=group->getActiveJointModelsBounds();
	for(auto v: bvec){
		for(auto v2: (*v)){
			std::cout << v2.min_position_ << " , " << v2.max_position_ << std::endl;
		}
		std::cout <<std::endl;
	}
	collision_detection::CollisionRobotPtr colRob = scene->getCollisionRobotNonConst ();
	//std::map< std::string, double > pad=colRob->getLinkPadding();
	//for (auto& kv : pad) {
    	//	std::cout << kv.first << " has padding value " << kv.second << std::endl;
	//}
	colRob->setPadding(0.04);
	scene->propogateRobotPadding ();
	//Create Tree
	Tree<AD>* t=new Tree<AD>();
	//Set Search Space Bounds
	State<AD> minState(-1.5f);
	State<AD> maxState(1.5f);
	minState[2]=-3.5;
	minState[3]=-2.25;
	maxState[2]=0.75;
	maxState[3]=0.25;
	t->setStateBounds(minState,maxState);
	//Set Tree Max Depth
	int depth=5;
	t->setMaxDepth(depth);
	//Depth First Obstacle Creation
	std::cout << "Obstacle creation " << std::setw(10) << 0.0 << "\% done.";
	timerStart=time(NULL);
	//addObstacles(t->getRootState(),0,1.0f,t);
	addObstacles(t->getRootKey(),0,t->getRootKey()[0],t);
	std::cout << std::endl;
	time_t timerNow=time(NULL);	
	int seconds = (int)difftime(timerNow,timerStart);
	int hours = seconds/3600;
	int minutes= seconds/60;
	seconds=seconds%60;
	std::cout << "Obstacles created in " 	<< std::setw(4) << hours << ":" 
						<< std::setw(2) << minutes << ":" 
						<< std::setw(2) << seconds 
						<< std::endl;
	//	//print tree to check
//	std::streamsize prev=std::cout.width(0);
//	std::cout.flags(std::ios_base::right);
//	std::cout<<*(t->getRoot())<<std::endl;
//	std::cout.width(prev);

	//Create algo
	std::cout << "create algo" << std::endl;
	MSP<AD> algo(t);
	//Set algo parameters
	State<AD> start(0.001f);
	start[1]=-0.35f;
	start[2]=-3.14f;
	start[3]=-0.21f;
	//start[5]=-0.11f;
	State<AD> goal(0.001f);
	goal[1]=0.31f;
	goal[3]=-0.31f;
	//start[5]=-0.11f;
	bool initAlgo=algo.init(start,goal);
	std::cout << "init algo " <<initAlgo << std::endl;
	//Run algo
	timerStart=time(NULL);
	if(initAlgo && algo.run()){
		timerNow=time(NULL);	
		int seconds = (int)difftime(timerNow,timerStart);
		int hours = seconds/3600;
		int minutes= seconds/60;
		seconds=seconds%60;
		std::cout << "Solution found in " 	<< std::setw(4) << hours << ":" 
						<< std::setw(2) << minutes << ":" 
						<< std::setw(2) << seconds 
						<< std::endl;
		std::cout << "raw solution" <<std::endl;
		std::deque<State<AD>> sol=algo.getPath();
		std::cout << "Path length: " << sol.size() << std::endl;
		std::cout << "Path cost: " << algo.getPathCost() << std::endl;
		//std::cout << "Path :" << std::endl;
		//for(std::deque<State<AD>>::iterator it=sol.begin(),end=sol.end();it!=end;++it){
		//	std::cout << (*it) << " -- ";
		//}
		//std::cout << std::endl;
		std::cout << "smoothed solution" <<std::endl;
		sol=algo.getSmoothedPath();
		std::cout << "Path length: " << sol.size() << std::endl;
		//Visualize results
		ros::Duration sleeper1(0.01);
		State<AD> prev(start);
		double pas=0.005;
		while(ros::ok()){
			for(std::deque<State<AD>>::iterator it=sol.begin(),end=sol.end();it!=end;++it){
				while((prev-*it).norm()>pas){
					prev=prev+(*it-prev)*(pas/(*it-prev).norm());
					for(int i=0;i<AD;++i){
						group_variable_values[i]=prev[i];
					}
					setState(*current_state,group_variable_values);
					publishState(*current_state);
					sleeper1.sleep();
				}
			}
			sleeper5.sleep();
			prev=start;
			for(int i=0;i<AD;++i){
				group_variable_values[i]=prev[i];
			}
			setState(*current_state,group_variable_values);
			publishState(*current_state);
			sleeper5.sleep();
		}
	}else{
		std::cout << "no solution found or init failed" <<std::endl;
	}

	delete t;

	std::cout << "no crash " << std::endl;
	ros::shutdown();
	return 0;
}

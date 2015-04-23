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
#include <ros/package.h>

#include <vector>
#include <sstream>

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
collision_detection::AllowedCollisionMatrix acm;

bool checkCollision(robot_state::RobotState& st){
	collision_result.clear();
	scene->checkCollision(collision_request, collision_result,st,acm);
	//return collision_result.collision;
	//return !st.satisfiesBounds(group) || scene->isStateColliding(st);
	return !st.satisfiesBounds(group) || collision_result.collision;
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

template <unsigned int DIM> bool addObstacles(Key<DIM> k, int depth, int size, Tree<DIM>* t){
	if(depth==t->getMaxDepth()){
		Ocount++;
		if(Ocount%((int)pow(2,8))==0){
			std::cout << "\r" << "Obstacle creation " << std::setw(10) << Ocount*100.0/pow(2,DIM*depth) << "\% done, ";
			time_t timerNow=time(NULL);	
			int seconds = (int)difftime(timerNow,timerStart);
			seconds=(int)(seconds*(1-( Ocount/pow(2,DIM*depth)))/( Ocount/pow(2,DIM*depth)));
			int hours = seconds/3600;
			int minutes= (seconds/60)%60;
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
	//add book in right hand
	moveit_msgs::AttachedCollisionObject attached_object;
	attached_object.link_name = "r_wrist_roll_link";
	attached_object.object.header.frame_id = "r_wrist_roll_link";
	attached_object.object.id = "book_r";
	geometry_msgs::Pose pose;
	pose.position.x=0.2;
	pose.orientation.w = 1.0;
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;  
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.15;
	primitive.dimensions[1] = 0.05;
	primitive.dimensions[2] = 0.25;  
	attached_object.object.primitives.push_back(primitive);
	attached_object.object.primitive_poses.push_back(pose);
	attached_object.object.operation = attached_object.object.ADD;
	//ADD book
	scene->processAttachedCollisionObjectMsg(attached_object);
	// add book in left hand too
	attached_object=moveit_msgs::AttachedCollisionObject();
	attached_object.link_name = "l_wrist_roll_link";
	attached_object.object.header.frame_id = "l_wrist_roll_link";
	attached_object.object.id = "book_l";
	attached_object.object.primitives.push_back(primitive);
	attached_object.object.primitive_poses.push_back(pose);
	attached_object.object.operation = attached_object.object.ADD;
	//ADD book
	scene->processAttachedCollisionObjectMsg(attached_object);
	//remove collision between book and robot hand
	acm=scene->getAllowedCollisionMatrixNonConst();
	collision_detection::CollisionRequest collision_request;
	collision_detection::CollisionResult collision_result;
	collision_request.contacts = true;
	collision_request.max_contacts = 1000;
	collision_result.clear();
	scene->checkSelfCollision(collision_request, collision_result);
	for(collision_detection::CollisionResult::ContactMap::const_iterator it = collision_result.contacts.begin(); 
	    it != collision_result.contacts.end(); 
	    ++it)
	{
	  	acm.setEntry(it->first.first, it->first.second, true);    
		std::cout << "adding entry in acm between " << it->first.first << " and " << it->first.second << std::endl;
	}

	moveit_msgs::PlanningScene planning_scene_msg;
	planning_scene_msg.is_diff = false;
	std::string path = ros::package::getPath("msp_pr2_test");
	std::stringstream ss;
	ss << path << "/scenes/bookshelves_big.scene";
	std::ifstream file(ss.str());
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
	group_variable_values[0] = 0.03; //set x to be close from the bookshelves
	group_variable_values[1] = -0.03; //set y
	setState(*current_state,group_variable_values);
	//publishState(*current_state);
	static tf::TransformBroadcaster br;
	static tf::Transform transform;
	transform.setOrigin( tf::Vector3(group_variable_values[0], group_variable_values[1], 0.0) );
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	transform.setRotation(q);
	ros::Timer timer = node_handle.createTimer(ros::Duration(1.0), [](const ros::TimerEvent&){br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/odom_combined", "base_footprint"));});
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
	std::vector<float> startvecr(5);
	startvecr[0]=0.001f;
	startvecr[1]=1.201f;
	startvecr[2]=-0.01f;
	startvecr[3]=-0.8499f;
	startvecr[4]=-3.1f;
	startvecr[5]=-0.11f;
	for(int i=0;i<group_variable_values.size();++i){
		group_variable_values[i] = startvecr[i];
	}
	/*group_variable_values[0]=0.001f;
	group_variable_values[1]=-0.35f;
	group_variable_values[2]=-3.14f;
	group_variable_values[3]=-0.21f;
	group_variable_values[4]=-3.1f;
	group_variable_values[5]=-0.11f;*/
	setState(*current_state,group_variable_values);
	publishState(*current_state);
	sleeper5.sleep();
	sleeper5.sleep();
	collision_detection::CollisionRobotPtr colRob = scene->getCollisionRobotNonConst ();
	colRob->setPadding(0.05);
	scene->propogateRobotPadding ();
	moveit::core::JointBoundsVector bvec=group->getActiveJointModelsBounds();
	//*
	for(auto v: bvec){
		for(auto v2: (*v)){
			std::cout << v2.min_position_ << " , " << v2.max_position_ << std::endl;
		}
		std::cout <<std::endl;
	}//*/
	//Create Tree
	Tree<AD>* t=new Tree<AD>();
	//Set Search Space Bounds
	State<AD> minStater;
	State<AD> maxStater;
	for(int i=0;i<AD;++i){
		minStater[i]=(*(bvec[i]))[0].min_position_;
		maxStater[i]=(*(bvec[i]))[0].max_position_;
	}
	/*State<AD> minState(-1.5f);
	State<AD> maxState(1.5f);
	minState[2]=-3.5;
	minState[3]=-2.25;
	maxState[2]=0.75;
	maxState[3]=0.25;*/
	t->setStateBounds(minStater,maxStater);
	//Set Tree Max Depth
	int depth=5;
	t->setMaxDepth(depth);
	//Start and goal
	State<AD> startr(0.001f);
	for(int i=0;i<AD;++i){
		startr[i]=startvecr[i];
	}
	State<AD> goalr(0.001f);
	goalr[1]=-0.35f;
	goalr[2]=-3.14f;
	goalr[3]=-0.21f;
	//Test if start and goal would be obstacle free before creating the entire map
	Key<AD> k;
	if(!(t->getKey(startr,k) && !isObstacle(t->getState(k)))){
		std::cout << "start not in free space" << std::endl;
		std::cout << "desired start : " << startr << std::endl;
		std::cout << "actual start : " << t->getState(k) << " with key " << k << std::endl;
		std::cout << "is obstacle : " << isObstacle(t->getState(k)) << std::endl;
		for(int i=0;i<group_variable_values.size();++i){
			group_variable_values[i] = t->getState(k)[i];
		}
		setState(*current_state,group_variable_values);
		publishState(*current_state);
		exit(1);
	}
	if(!(t->getKey(goalr,k) && !isObstacle(t->getState(k)))){
		std::cout << "goal not in free space" << std::endl;
		std::cout << "desired goal : " << goalr << std::endl;
		std::cout << "actual goal : " << t->getState(k) << " with key " << k << std::endl;
		for(int i=0;i<group_variable_values.size();++i){
			group_variable_values[i] = t->getState(k)[i];
		}
		setState(*current_state,group_variable_values);
		publishState(*current_state);
		exit(1);
	}
	//Depth First Obstacle Creation
	std::cout << "Obstacle creation " << std::setw(10) << 0.0 << "\% done.";
	timerStart=time(NULL);
	//addObstacles(t->getRootKey(),0,t->getRootKey()[0],t);
	std::cout << std::endl;
	time_t timerNow=time(NULL);	
	int seconds = (int)difftime(timerNow,timerStart);
	int hours = seconds/3600;
	int minutes= (seconds/60)%60;
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

	//variables to hold results
	std::deque<State<AD>> solr,soll;
	//Create algo
	std::cout << "create algo" << std::endl;
	MSP<AD> algo(t);
	//Set algo parameters
	algo.setNewNeighboorCheck(true);
	algo.setMapLearning(true,10,isObstacle);
	algo.setSpeedUp(true);
	algo.setAlpha(2*sqrt(AD));
	algo.setEpsilon(0.5);
	bool initAlgo=algo.init(startr,goalr);
	std::cout << "init algo " <<initAlgo << std::endl;
	//Run algo
	timerStart=time(NULL);
	if(initAlgo && algo.run()){
		timerNow=time(NULL);	
		int seconds = (int)difftime(timerNow,timerStart);
		int hours = seconds/3600;
		int minutes= (seconds/60)%60;
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
		solr=sol;
	}else{
		std::cout << "no solution found or init failed" <<std::endl;
	}

	delete t;

	//Plan for the second arm
	std::cout << "NOW PLANINNG FOR SECOND ARM" << std::endl;
	for(int i=0;i<group_variable_values.size();++i){
		group_variable_values[i] = startvecr[i];
	}
	setState(*current_state,group_variable_values);
	publishState(*current_state);

	group=kinematic_model->getJointModelGroup("left_arm");
	current_state->copyJointGroupPositions(group,group_variable_values);

	std::vector<float> startvecl(5);
	startvecl[0]=-0.001f;
	startvecl[1]=-0.35f;
	startvecl[2]=3.14f;
	startvecl[3]=-0.21f;
	startvecl[4]=3.1f;
	startvecl[5]=-0.11f;
	for(int i=0;i<group_variable_values.size();++i){
		group_variable_values[i] = startvecl[i];
	}
	setState(*current_state,group_variable_values);
	publishState(*current_state);

	bvec=group->getActiveJointModelsBounds();
	//*
	for(auto v: bvec){
		for(auto v2: (*v)){
			std::cout << v2.min_position_ << " , " << v2.max_position_ << std::endl;
		}
		std::cout <<std::endl;
	}//*/
	//Set Search Space Bounds
	State<AD> minStatel;
	State<AD> maxStatel;
	for(int i=0;i<AD;++i){
		minStatel[i]=(*(bvec[i]))[0].min_position_;
		maxStatel[i]=(*(bvec[i]))[0].max_position_;
	}
	
	//Create Tree
	t=new Tree<AD>();
	//Set Search Space Bounds
	t->setStateBounds(minStatel,maxStatel);
	t->setMaxDepth(depth);
	//Start and goal
	State<AD> startl(0.001f);
	for(int i=0;i<AD;++i){
		startl[i]=startvecl[i];
	}
	State<AD> goall(-0.001f);
	goall[1]=0.31f;
	goall[3]=-0.31f;
	//Test if start and goal would be obstacle free before creating the entire map
	if(!(t->getKey(startl,k) && !isObstacle(t->getState(k)))){
		std::cout << "start not in free space" << std::endl;
		std::cout << "desired start : " << startl << std::endl;
		std::cout << "actual start : " << t->getState(k) << " with key " << k << std::endl;
		std::cout << "is obstacle : " << isObstacle(t->getState(k)) << std::endl;
		for(int i=0;i<group_variable_values.size();++i){
			group_variable_values[i] = t->getState(k)[i];
		}
		setState(*current_state,group_variable_values);
		publishState(*current_state);
		exit(1);
	}
	if(!(t->getKey(goall,k) && !isObstacle(t->getState(k)))){
		std::cout << "goal not in free space" << std::endl;
		std::cout << "desired goal : " << goall << std::endl;
		std::cout << "actual goal : " << t->getState(k) << " with key " << k << std::endl;
		for(int i=0;i<group_variable_values.size();++i){
			group_variable_values[i] = t->getState(k)[i];
		}
		setState(*current_state,group_variable_values);
		publishState(*current_state);
		exit(1);
	}
	//Depth First Obstacle Creation
	std::cout << "Obstacle creation " << std::setw(10) << 0.0 << "\% done.";
	timerStart=time(NULL);
	Ocount=0;
	//addObstacles(t->getRootKey(),0,t->getRootKey()[0],t);
	std::cout << std::endl;
	timerNow=time(NULL);	
	seconds = (int)difftime(timerNow,timerStart);
	hours = seconds/3600;
	minutes= (seconds/60)%60;
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

	//variables to hold results
	//Create algo
	std::cout << "create algo" << std::endl;
	algo.clear();
	MSP<AD> algo2(t);
	//Set algo parameters
	algo2.setNewNeighboorCheck(true);
	algo2.setMapLearning(true,10,isObstacle);
	algo2.setSpeedUp(true);
	algo2.setAlpha(2*sqrt(AD));
	algo2.setEpsilon(0.5);
	initAlgo=algo2.init(startl,goall);
	std::cout << "init algo " <<initAlgo << std::endl;
	//Run algo
	timerStart=time(NULL);
	if(initAlgo && algo2.run()){
		timerNow=time(NULL);	
		int seconds = (int)difftime(timerNow,timerStart);
		int hours = seconds/3600;
		int minutes= (seconds/60)%60;
		seconds=seconds%60;
		std::cout << "Solution found in " 	<< std::setw(4) << hours << ":" 
						<< std::setw(2) << minutes << ":" 
						<< std::setw(2) << seconds 
						<< std::endl;
		std::cout << "raw solution" <<std::endl;
		std::deque<State<AD>> sol=algo2.getPath();
		std::cout << "Path length: " << sol.size() << std::endl;
		std::cout << "Path cost: " << algo2.getPathCost() << std::endl;
		//std::cout << "Path :" << std::endl;
		//for(std::deque<State<AD>>::iterator it=sol.begin(),end=sol.end();it!=end;++it){
		//	std::cout << (*it) << " -- ";
		//}
		//std::cout << std::endl;
		std::cout << "smoothed solution" <<std::endl;
		sol=algo2.getSmoothedPath();
		std::cout << "Path length: " << sol.size() << std::endl;
		soll=sol;
	}else{
		std::cout << "no solution found or init failed" <<std::endl;
	}

	delete t;
	
	//Visualize results
	ros::Duration sleeper1(0.01);
	State<AD> prevl(startl);
	State<AD> prevr(startr);
	int indexl=0;
	int indexr=0;
	double pas=0.005;
	const moveit::core::JointModelGroup* groupl=kinematic_model->getJointModelGroup("left_arm");
	const moveit::core::JointModelGroup* groupr=kinematic_model->getJointModelGroup("right_arm");
	std::vector<double> group_variable_valuesl;
	std::vector<double> group_variable_valuesr;
	current_state->copyJointGroupPositions(groupr,group_variable_valuesr);
	current_state->copyJointGroupPositions(groupl,group_variable_valuesl);
	while(ros::ok()){
		bool donel=false;
		bool doner=false;
		/*for(std::deque<State<AD>>::iterator it=solr.begin(),end=solr.end();it!=end;++it){
			while((prev-*it).norm()>pas){
				prev=prev+(*it-prev)*(pas/(*it-prev).norm());
				for(int i=0;i<AD;++i){
					group_variable_values[i]=prev[i];
				}
				setState(*current_state,group_variable_values);
				publishState(*current_state);
				sleeper1.sleep();
			}
		}*/
		while(!donel || !doner){
			if(!donel){
				if((prevl-soll[indexl]).norm()>pas){
					prevl=prevl+(soll[indexl]-prevl)*(pas/(soll[indexl]-prevl).norm());
					for(int i=0;i<AD;++i){
						group_variable_valuesl[i]=prevl[i];
					}
					current_state->setJointGroupPositions(groupl,group_variable_valuesl);
				}else{
					if(indexl<soll.size()-1){
						indexl++;
					}else{
						donel=true;
					}
				}
			}
			if(!doner){
				if((prevr-solr[indexr]).norm()>pas){
					prevr=prevr+(solr[indexr]-prevr)*(pas/(solr[indexr]-prevr).norm());
					for(int i=0;i<AD;++i){
						group_variable_valuesr[i]=prevr[i];
					}
					current_state->setJointGroupPositions(groupr,group_variable_valuesr);
				}else{
					if(indexr<solr.size()-1){
						indexr++;
					}else{
						doner=true;
					}
				}
			}
			publishState(*current_state);
			sleeper1.sleep();
			
		}
		sleeper5.sleep();
		prevl=startl;
		for(int i=0;i<AD;++i){
			group_variable_valuesl[i]=prevl[i];
		}
		prevr=startr;
		for(int i=0;i<AD;++i){
			group_variable_valuesr[i]=prevr[i];
		}
		current_state->setJointGroupPositions(groupl,group_variable_valuesl);
		current_state->setJointGroupPositions(groupr,group_variable_valuesr);
		indexl=0;
		indexr=0;
		publishState(*current_state);
		sleeper5.sleep();
	}
	std::cout << "no crash " << std::endl;
	ros::shutdown();
	return 0;
}

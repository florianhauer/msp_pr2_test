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

#include <iostream>     // std::cout
#include <iomanip>
#include <fstream>
#include <ctime>

#include <msp/State.h>

#include <ompl/base/samplers/InformedStateSampler.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/DRRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/tools/benchmark/Benchmark.h>

#define AD 4

namespace ob = ompl::base;
namespace og = ompl::geometric;

ob::StateSpacePtr space;

State<2*AD+1> obstateState(const ob::State* s){
	State<2*AD+1> s2;
	for(int i=0;i<2*AD+1;++i){
		s2[i]=s->as<ob::RealVectorStateSpace::StateType>()->values[i];
	}
	return s2;
}

ros::Publisher planning_scene_diff_publisher;
ros::Publisher joint_states;
planning_scene::PlanningScenePtr scene;
collision_detection::CollisionRequest collision_request;
collision_detection::CollisionResult collision_result;
robot_state::RobotState* current_state;
robot_model::RobotModelPtr kinematic_model;
std::vector<double> group_variable_values;
const moveit::core::JointModelGroup* group;
std::vector<double> group_variable_values_l;
const moveit::core::JointModelGroup* group_l;
std::vector<double> group_variable_values_r;
const moveit::core::JointModelGroup* group_r;

bool checkCollision(robot_state::RobotState& st){
	return !st.satisfiesBounds(group) || !st.satisfiesBounds(group_l) || !st.satisfiesBounds(group_r) || scene->isStateColliding(st);
}

void setState(robot_state::RobotState& st, std::vector<double> val_r, std::vector<double> val_l, std::vector<double> val_b){
	st.setJointGroupPositions(group, val_b);
	st.setJointGroupPositions(group_r, val_r);
	st.setJointGroupPositions(group_l, val_l);
}

bool isObstacle(const ob::State* state){
	ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s(space,state);
	for(int i=0;i<AD;++i){
			group_variable_values_r[i]=s[i];
			group_variable_values_l[i]=s[AD+i];
	}
    group_variable_values[0]=s[2*AD];
	setState(*current_state,group_variable_values_r,group_variable_values_l,group_variable_values);
	return checkCollision(*current_state);
}

time_t timerStart,timerNow;

void addObstaclesToPlanningScene(){
	moveit_msgs::PlanningScene planning_scene_msg;
	planning_scene_msg.is_diff = false;
	std::string path = ros::package::getPath("msp_pr2_test");
	std::stringstream ss;
	ss << path << "/scenes/bookshelves.scene";
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

class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr& si) :
        ob::StateValidityChecker(si) {}

    bool isValid(const ob::State* s) const
    {
        return !isObstacle(s);
    }
};


int main(int argc, char **argv)
{
	//Init all the variables
	ros::init (argc, argv, "dual_arm_planning");
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
	
	std::cout << "waiting for subscriber" << std::endl;
	while(planning_scene_diff_publisher.getNumSubscribers() < 1 && joint_states.getNumSubscribers() < 1)
	{
		ros::WallDuration sleep_t(0.5);
		sleep_t.sleep();
	}
	std::cout << "done. modifying scene" << std::endl;
	//robot state, change robot state
	current_state = &(scene->getCurrentStateNonConst());
	//set base position
	group=kinematic_model->getJointModelGroup("base");
	current_state->copyJointGroupPositions(group,group_variable_values);
	group_variable_values[0] = -0.5; //set x to be close from the bookshelves
	group_variable_values[1] = -0.03; //set y
	current_state->setJointGroupPositions(group, group_variable_values);
	static tf::TransformBroadcaster br;
	static tf::Transform transform;
	transform.setOrigin( tf::Vector3(group_variable_values[0], group_variable_values[1], 0.0) );
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	transform.setRotation(q);
	//ros::Timer timer = node_handle.createTimer(ros::Duration(0.1), [](const ros::TimerEvent&){br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/odom_combined", "base_footprint"));});
	//add Obstacles
	sleeper5.sleep();
	sleeper5.sleep();
	std::cout << "adding obstacles" << std::endl;
	addObstaclesToPlanningScene();
	std::cout << "done" << std::endl;


	//set left arm position
	group_l=kinematic_model->getJointModelGroup("left_arm");
	current_state->copyJointGroupPositions(group_l,group_variable_values_l);
	std::vector<float> startvecl(5);
	startvecl[0]=-0.001f;
	startvecl[1]=-0.35f;
	startvecl[2]=3.14f;
	startvecl[3]=-0.21f;
	startvecl[4]=3.1f;
	startvecl[5]=-0.11f;

	//set current group to right arm for planning
	group_r=kinematic_model->getJointModelGroup("right_arm");
	current_state->copyJointGroupPositions(group_r,group_variable_values_r);
	std::vector<float> startvecr(5);
	startvecr[0]=0.001f;
	startvecr[1]=1.201f;
	startvecr[2]=-0.01f;
	startvecr[3]=-0.8499f;
	startvecr[4]=-3.1f;
	startvecr[5]=-0.11f;
	for(int i=0;i<group_variable_values_r.size();++i){
		group_variable_values_r[i] = startvecr[i];
		group_variable_values_l[i] = startvecl[i];
	}
	setState(*current_state,group_variable_values_r,group_variable_values_l,group_variable_values);
	publishState(*current_state);
	sleeper5.sleep();
	sleeper5.sleep();

	//collision_detection::CollisionRobotPtr colRob = scene->getCollisionRobotNonConst ();
	//colRob->setPadding(0.04);
	//scene->propogateRobotPadding();

	std::cout << "before any ompl call" << std::endl;
	//Set Search Space Bounds
	ob::RealVectorBounds bound(2*AD+1);
	moveit::core::JointBoundsVector bvec_r=group_r->getActiveJointModelsBounds();
	moveit::core::JointBoundsVector bvec_l=group_l->getActiveJointModelsBounds();
	for(int i=0;i<AD;++i){
		bound.setLow(i,(*(bvec_r[i]))[0].min_position_);
		bound.setHigh(i,(*(bvec_r[i]))[0].max_position_);
		bound.setLow(AD+i,(*(bvec_l[i]))[0].min_position_);
		bound.setHigh(AD+i,(*(bvec_l[i]))[0].max_position_);
	}
	bound.setLow(2*AD,-0.5);
	bound.setHigh(2*AD,0.0);

	////////////////////////////////////

	space.reset(new ob::RealVectorStateSpace(2*AD+1));
	space->as<ob::RealVectorStateSpace>()->setBounds(bound);
    	ompl::geometric::SimpleSetup ss(space);
	ss.setStateValidityChecker(ompl::base::StateValidityCheckerPtr(new ValidityChecker(ss.getSpaceInformation())));
	ss.getSpaceInformation()->setup();

	ob::ScopedState<> start(space);
	ob::ScopedState<> goal(space);
	for(int i=0;i<AD;++i){
		start[i]=startvecr[i];
		goal[i]=0.001;
		start[AD+i]=startvecl[i];
		goal[AD+i]=0.001;
	}
	goal[1]=-0.35f;
	goal[2]=-3.14f;
	goal[3]=-0.21f;
	goal[AD+1]=0.31f;
	goal[AD+3]=-0.31f;
    start[2*AD]=group_variable_values[0];
    goal[2*AD]=group_variable_values[0];
	if(AD>4){
		goal[4]=startvecr[4];
		goal[AD+4]=startvecl[4];
	}
	if(AD>5){
		goal[5]=startvecr[5];
		goal[AD+5]=startvecl[5];
	}

	std::cout << "valid start goal : " << !isObstacle(start.get()) << " , " << !isObstacle(goal.get()) << std::endl;


	/*
	ROS_INFO("Benchmarking");

	// by default, use the Benchmark class
	double runtime_limit = 200, memory_limit = 2048;
	int run_count = 5;
	ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 0.05,true,true,false,false);

    	ss.setStartAndGoalStates(start, goal);
	ss.setOptimizationObjective(ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(ss.getSpaceInformation())));
	ompl::tools::Benchmark b(ss, "pr2arm");

	double range=0.4;
	// benchmark
	ompl::base::PlannerPtr rrtstar(new ompl::geometric::RRTstar(ss.getSpaceInformation()));
	rrtstar->as<ompl::geometric::RRTstar>()->setName("RRT*");
	rrtstar->as<ompl::geometric::RRTstar>()->setDelayCC(false);
	rrtstar->as<ompl::geometric::RRTstar>()->setFocusSearch(false);
	rrtstar->as<ompl::geometric::RRTstar>()->setRange(range);
	b.addPlanner(rrtstar);
	ompl::base::PlannerPtr rrtsharpepsilon0c(new ompl::geometric::RRTsharp(ss.getSpaceInformation()));
	rrtsharpepsilon0c->as<ompl::geometric::RRTsharp>()->setName("RRTsharp");
	rrtsharpepsilon0c->as<ompl::geometric::RRTsharp>()->setRange(range);
	b.addPlanner(rrtsharpepsilon0c);
	
	ompl::base::PlannerPtr drrttd(new ompl::geometric::DRRT(ss.getSpaceInformation()));
	drrttd->as<ompl::geometric::DRRT>()->setName("DRRTtd");
	drrttd->as<ompl::geometric::DRRT>()->setRange(range);
	drrttd->as<ompl::geometric::DRRT>()->setVariant(ompl::geometric::DRRT::TREE);
	drrttd->as<ompl::geometric::DRRT>()->setDelayOptimizationUntilSolution(true);
	b.addPlanner(drrttd);
	ompl::base::PlannerPtr drrtt(new ompl::geometric::DRRT(ss.getSpaceInformation()));
	drrtt->as<ompl::geometric::DRRT>()->setName("DRRTt");
	drrtt->as<ompl::geometric::DRRT>()->setRange(range);
	drrtt->as<ompl::geometric::DRRT>()->setVariant(ompl::geometric::DRRT::TREE);
	b.addPlanner(drrtt);
	ompl::base::PlannerPtr drrttf(new ompl::geometric::DRRT(ss.getSpaceInformation()));
	drrttf->as<ompl::geometric::DRRT>()->setName("DRRTtf0.3");
	drrttf->as<ompl::geometric::DRRT>()->setRange(range);
	drrttf->as<ompl::geometric::DRRT>()->setVariant(ompl::geometric::DRRT::TREE);
	drrttf->as<ompl::geometric::DRRT>()->setDeformationFrequency(0.3);
	b.addPlanner(drrttf);
	std::cout << "starting benchmark" << std::endl;
	b.benchmark(request);
	std::cout << "benchmark done" << std::endl;
	b.saveResultsToFile(boost::str(boost::format("/home/florian/results/pr2arm2_%i.log") % AD ).c_str());

	*/

	//* to just run the algorithm and visualize results


	ROS_INFO("RRT star planning");
	//space->setLongestValidSegmentFraction(0.01/(maxX(local_map->info)-minX(local_map->info)));
	ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
	si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si)));
	si->setup();
	ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
	pdef->setStartAndGoalStates(start, goal);
	pdef->setOptimizationObjective(ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si)));

	std::cout << "create algo" << std::endl;

	auto plan_pt=new og::RRTstar(si);
	std::cout << "create optimizingplanner" << std::endl;
	ob::PlannerPtr optimizingPlanner(plan_pt);
	std::cout << "set problem defionition" << std::endl;
	optimizingPlanner->setProblemDefinition(pdef);
	std::cout << "setup" << std::endl;
	optimizingPlanner->setup();
	ob::PlannerStatus solved;
	int it=0;
	std::cout << "run algo" << std::endl;
	timerStart=time(NULL);
	while(solved!=ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION && it<5){
		it++;
		solved = optimizingPlanner->solve(10.0);
	}
	if(solved==ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION || solved==ompl::base::PlannerStatus::StatusType::APPROXIMATE_SOLUTION){
		if(solved!=ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION)
			std::cout << "approximate solution" << std::endl;
		timerNow=time(NULL);	
		int seconds = (int)difftime(timerNow,timerStart);
		int hours = seconds/3600;
		int minutes= (seconds/60)%60;
		seconds=seconds%60;
		std::cout << "Solution found in " 	<< std::setw(4) << hours << ":" 
						<< std::setw(2) << minutes << ":" 
						<< std::setw(2) << seconds 
						<< std::endl;
		std::vector< ob::State * > sol1 = std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath())->getStates();
		std::deque<State<2*AD+1>> sol;//current pose not included
		sol.resize(sol1.size());
		std::transform(sol1.begin(), sol1.end(), sol.begin(), &obstateState);
		ROS_INFO("Planning success");

		ros::Duration sleeper1(0.01);
		State<2*AD+1> prev(obstateState(start.get()));
		double pas=0.005;
		while(ros::ok()){
			for(std::deque<State<2*AD+1>>::iterator it=sol.begin(),end=sol.end();it!=end;++it){
				while((prev-*it).norm()>pas){
					prev=prev+(*it-prev)*(pas/(*it-prev).norm());
					for(int i=0;i<AD;++i){
						group_variable_values_r[i]=prev[i];
						group_variable_values_l[i]=prev[AD+i];
					}
					group_variable_values[0]=prev[2*AD];
					setState(*current_state, group_variable_values_r, group_variable_values_l,group_variable_values);
					publishState(*current_state);
					sleeper1.sleep();
				}
			}
			sleeper5.sleep();
			prev=obstateState(start.get());
			for(int i=0;i<AD;++i){
				group_variable_values_r[i]=prev[i];
				group_variable_values_l[i]=prev[AD+i];
			}
			group_variable_values[0]=prev[2*AD];
			setState(*current_state, group_variable_values_r, group_variable_values_l,group_variable_values);
			publishState(*current_state);
			sleeper5.sleep();
		}

	}else{
		ROS_INFO("Planning failed");
	}//*/
	
	std::cout << "no crash " << std::endl;
	ros::shutdown();
	return 0;
}

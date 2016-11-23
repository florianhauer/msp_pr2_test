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
#include <mutex>

#include <msp/State.h>

#include <ompl/base/samplers/InformedStateSampler.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/DRRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/cforest/CForest.h>

#include <ompl/tools/benchmark/Benchmark.h>

#include <ros/console.h>

#define AD 4

namespace ob = ompl::base;
namespace og = ompl::geometric;

ob::StateSpacePtr space;

State<AD> obstateState(const ob::State* s){
	State<AD> s2;
	for(int i=0;i<AD;++i){
		s2[i]=s->as<ob::RealVectorStateSpace::StateType>()->values[i];
	}
	return s2;
}

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

std::mutex mut;

bool isObstacle(const ob::State* state){
    mut.lock();    
	ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s(space,state);
	for(int i=0;i<s.reals().size();++i){
			group_variable_values[i]=s[i];
	}
	setState(*current_state,group_variable_values);
	bool temp = checkCollision(*current_state);
    mut.unlock();
    return temp;
}

bool isObstacle2(const ob::State* state){ 
	ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s(space,state);
    std::vector<double> vals;
	robot_state::RobotState*  cs = &(scene->getCurrentStateNonConst());
	cs->copyJointGroupPositions(group,vals);
	for(int i=0;i<s.reals().size();++i){
			vals[i]=s[i];
	}
	setState(*cs,vals);
	return checkCollision(*cs);
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
	std::cout << "adding obstacles" << std::endl;
	addObstaclesToPlanningScene();
	std::cout << "done" << std::endl;
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
	std::vector<float> startvec(5);
	startvec[0]=0.001f;
	startvec[1]=-0.35f;
	startvec[2]=-3.14f;
	startvec[3]=-0.21f;
	startvec[4]=-3.1f;
	startvec[5]=-0.11f;
	for(int i=0;i<group_variable_values.size();++i){
		group_variable_values[i] = startvec[i];
	}
	/*group_variable_values[0]=0.001f;
	group_variable_values[1]=-0.35f;
	group_variable_values[2]=-3.14f;
	group_variable_values[3]=-0.21f;
	group_variable_values[4]=-3.1f;
	group_variable_values[5]=-0.5f;*/
	setState(*current_state,group_variable_values);
	publishState(*current_state);
	sleeper5.sleep();
	sleeper5.sleep();
	collision_detection::CollisionRobotPtr colRob = scene->getCollisionRobotNonConst ();
	colRob->setPadding(0.04);
	scene->propogateRobotPadding();
	moveit::core::JointBoundsVector bvec=group->getActiveJointModelsBounds();

	std::cout << "before any ompl call" << std::endl;
	//Set Search Space Bounds
	ob::RealVectorBounds bound(AD);
	std::vector<double> maxState;
	for(int i=0;i<AD;++i){
		bound.setLow(i,(*(bvec[i]))[0].min_position_);
		bound.setHigh(i,(*(bvec[i]))[0].max_position_);
	}

	////////////////////////////////////

	ROS_INFO("RRT star planning");
	space.reset(new ob::RealVectorStateSpace(AD));
	space->as<ob::RealVectorStateSpace>()->setBounds(bound);
    	ompl::geometric::SimpleSetup ss(space);
	ss.setStateValidityChecker(ompl::base::StateValidityCheckerPtr(new ValidityChecker(ss.getSpaceInformation())));
	ss.getSpaceInformation()->setup();
	//space->setLongestValidSegmentFraction(0.01/(maxX(local_map->info)-minX(local_map->info)));
	//ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
	//si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si)));
	//si->setup();

	ob::ScopedState<> start(space);
	ob::ScopedState<> goal(space);
	for(int i=0;i<AD;++i){
		start[i]=startvec[i];
		goal[i]=0.001;
	}
	goal[1]=0.31f;
	goal[3]=-0.31f;
	if(AD>4)
		goal[4]=startvec[4];
	if(AD>5)
		goal[5]=startvec[5];

	std::cout << "valid start goal : " << !isObstacle(start.get()) << " , " << !isObstacle(goal.get()) << std::endl;

	//ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
	//pdef->setStartAndGoalStates(start, goal);
    	ss.setStartAndGoalStates(start, goal);

	//pdef->setOptimizationObjective(ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si)));
	// by default, use the Benchmark class
	double runtime_limit = 300, memory_limit = 2048;
	int run_count = 30;
	ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 10,true,true,false,false);

	ss.setOptimizationObjective(ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(ss.getSpaceInformation())));
    ss.setup();
	ompl::tools::Benchmark b(ss, "pr2arm");

	double range=0.4;
    bool knearest=false;
	//* benchmark
	ompl::base::PlannerPtr rrtstar(new ompl::geometric::RRTstar(ss.getSpaceInformation()));
	rrtstar->as<ompl::geometric::RRTstar>()->setName("RRT*");
	rrtstar->as<ompl::geometric::RRTstar>()->setDelayCC(false);
	rrtstar->as<ompl::geometric::RRTstar>()->setFocusSearch(false);
	rrtstar->as<ompl::geometric::RRTstar>()->setRange(range);
	rrtstar->as<ompl::geometric::RRTstar>()->setKNearest(knearest);
	b.addPlanner(rrtstar);
	ompl::base::PlannerPtr rrtsharpepsilon0c(new ompl::geometric::RRTsharp(ss.getSpaceInformation()));
	rrtsharpepsilon0c->as<ompl::geometric::RRTsharp>()->setName("RRTsharp");
	rrtsharpepsilon0c->as<ompl::geometric::RRTsharp>()->setRange(range);
	rrtsharpepsilon0c->as<ompl::geometric::RRTsharp>()->setKNearest(knearest);
	b.addPlanner(rrtsharpepsilon0c);
	ompl::base::PlannerPtr rrtx(new ompl::geometric::RRTXstatic(ss.getSpaceInformation()));
	rrtx->as<ompl::geometric::RRTXstatic>()->setName("RRTX0.1");
	rrtx->as<ompl::geometric::RRTXstatic>()->setRange(range);
	rrtx->as<ompl::geometric::RRTXstatic>()->setEpsilon(0.1);
	rrtx->as<ompl::geometric::RRTXstatic>()->setKNearest(knearest);
	b.addPlanner(rrtx);
	//*
	ompl::base::PlannerPtr drrttd(new ompl::geometric::DRRT(ss.getSpaceInformation()));
	drrttd->as<ompl::geometric::DRRT>()->setName("DRRTtd");
	drrttd->as<ompl::geometric::DRRT>()->setRange(range);
	drrttd->as<ompl::geometric::DRRT>()->setVariant(ompl::geometric::DRRT::TREE);
	drrttd->as<ompl::geometric::DRRT>()->setDelayOptimizationUntilSolution(true);
	drrttd->as<ompl::geometric::DRRT>()->setKNearest(knearest);
	b.addPlanner(drrttd);
	ompl::base::PlannerPtr drrtt(new ompl::geometric::DRRT(ss.getSpaceInformation()));
	drrtt->as<ompl::geometric::DRRT>()->setName("DRRTt");
	drrtt->as<ompl::geometric::DRRT>()->setRange(range);
	drrtt->as<ompl::geometric::DRRT>()->setVariant(ompl::geometric::DRRT::TREE);
	drrtt->as<ompl::geometric::DRRT>()->setKNearest(knearest);
	b.addPlanner(drrtt);//*
	ompl::base::PlannerPtr drrttf(new ompl::geometric::DRRT(ss.getSpaceInformation()));
	drrttf->as<ompl::geometric::DRRT>()->setName("DRRTtf0.3");
	drrttf->as<ompl::geometric::DRRT>()->setRange(range);
	drrttf->as<ompl::geometric::DRRT>()->setVariant(ompl::geometric::DRRT::TREE);
	drrttf->as<ompl::geometric::DRRT>()->setDeformationFrequency(0.3);
	drrttf->as<ompl::geometric::DRRT>()->setKNearest(knearest);
	b.addPlanner(drrttf);//*/
	// Obstacle check is not thread safe...
    ompl::base::PlannerPtr cf(new ompl::geometric::CForest(ss.getSpaceInformation()));
	cf->as<ompl::geometric::CForest>()->setName("CForest-DRRTtd0.3");
	cf->as<ompl::geometric::CForest>()->setProblemDefinition(ss.getProblemDefinition());
    const int threadsNb=8;
    cf->as<ompl::geometric::CForest>()->setNumThreads(threadsNb);
    cf->as<ompl::geometric::CForest>()->addPlannerInstances<ompl::geometric::DRRT>(threadsNb);
    for(int i=0;i<threadsNb;++i){
        auto plan= cf->as<ompl::geometric::CForest>()->getPlannerInstance(i);
	    plan->as<ompl::geometric::DRRT>()->setRange(range);
	    plan->as<ompl::geometric::DRRT>()->setVariant(ompl::geometric::DRRT::TREE);
	    plan->as<ompl::geometric::DRRT>()->setDelayOptimizationUntilSolution(true);
	    plan->as<ompl::geometric::DRRT>()->setDeformationFrequency(0.3);
	    plan->as<ompl::geometric::DRRT>()->setKNearest(knearest);
    }
	b.addPlanner(cf);//*/
	std::cout << "starting benchmark" << std::endl;
	b.benchmark(request);
	std::cout << "benchmark done" << std::endl;
	b.saveResultsToFile(boost::str(boost::format("/home/flo/results/pr2arm_%i.log") % AD ).c_str());
	//*/

	/* to just run the algorithm and visualize results
	std::cout << "create algo" << std::endl;
	
	//og::DRRT *plan_pt=new og::DRRT(si);
	//plan_pt->setDelayOptimizationUntilSolution(false);
	//plan_pt->setDeformationFrequency(0.1);
	//
	og::RRTstar *plan_pt=new og::RRTstar(si);
	std::cout << "set goal bias" << std::endl;
	plan_pt->setGoalBias(0.05);
	//plan_pt->setRange(1.0);
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
	while(solved!=ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION && it<10){
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
		std::vector< ob::State * > sol1 = boost::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath())->getStates();
		std::deque<State<AD>> sol;//current pose not included
		sol.resize(sol1.size());
		std::transform(sol1.begin(), sol1.end(), sol.begin(), &obstateState);
		ROS_INFO("Planning success");

		ros::Duration sleeper1(0.01);
		State<AD> prev(obstateState(start.get()));
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
			prev=obstateState(start.get());
			for(int i=0;i<AD;++i){
				group_variable_values[i]=prev[i];
			}
			setState(*current_state,group_variable_values);
			publishState(*current_state);
			sleeper5.sleep();
		}

	}else{
		ROS_INFO("Planning failed");
	}*/
	
	std::cout << "no crash " << std::endl;
	ros::shutdown();
	return 0;
}

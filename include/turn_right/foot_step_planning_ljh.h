#include <FootstepPlannerLJH/AStarFootstepPlanner.h>
#include <FootstepPlannerLJH/parameters.h>
#include <FootstepPlannerLJH/StepConstraints/StepConstraintCheck.h>
#include <FootstepPlannerLJH/StepConstraints/StepConstraintCheck.h>
#include <iostream>
#include <assert.h>
#include <Eigen/Core>
// param 参数分别为 goalX， goalY， goalyaw, point1.x ... point4.y
// clockwise_ 1为顺时针，0为逆时针
std::vector<std::pair<Eigen::Vector3d, bool> > foot_step_planning(std::vector<double> param_dis, int clockwise_, double start_x_)
{
    assert(param_dis.size() == 11 && "your entered param is wrong");
    assert(clockwise_ == 0 || clockwise_ == 1 && "points direct is wrong");
    CHECK(param_dis.size() == 11);
    CHECK(clockwise_ == 0 || clockwise_ == 1);
    std::cout<<"in function foot_step_planning:"<<std::endl;
    std::cout<<"goal pose: x "<<param_dis.at(0)<<" y "<<param_dis.at(1)<<" yaw "<<param_dis.at(2)<<std::endl;
    std::cout<<"4 points: "<<endl;
    std::cout<<"point 1: "<<param_dis.at(3)<<" "<<param_dis.at(4)<<std::endl;
    std::cout<<"point 2: "<<param_dis.at(5)<<" "<<param_dis.at(6)<<std::endl;
    std::cout<<"point 3: "<<param_dis.at(7)<<" "<<param_dis.at(8)<<std::endl;
    std::cout<<"point 4: "<<param_dis.at(9)<<" "<<param_dis.at(10)<<std::endl;
    ljh::path::footstep_planner::StepConstraintCheck checker;
    ljh::path::footstep_planner::LatticePoint latticepoint;
    ljh::path::footstep_planner::parameters param;

    latticepoint.setGridSizeXY(latticepoint, 0.01);
    latticepoint.setYawDivision(latticepoint, 72);

    param.SetEdgeCostDistance(param, 4.0);
    param.SetEdgeCostYaw(param, 4.0);
    param.SetEdgeCostStaticPerStep(param, 1.4);
    param.SetDebugFlag(param, false);
    param.SetMaxStepYaw(param, pi/12);//pi/11
    param.SetMinStepYaw(param, -pi/12);//-p1/11        

    param.SetFinalTurnProximity(param, 0.3);
    param.SetGoalDistanceProximity(param, 0.04);
    param.SetGoalYawProximity(param, 4.0/180.0 * pi);
    param.SetFootPolygonExtendedLength(param, 0.025);
    
    param.SetHWPOfWalkDistacne(param,1.3);
    param.SetHWPOfPathDistance(param,2.50);
    
    param.SetHWPOfFinalTurnDistacne(param,1.30);
    param.SetHWPOfFinalWalkDistacne(param,1.30);

    param.SetMaxStepLength(param, 0.08);//0.12
    param.SetMinStepLength(param,-0.08);//-0.12
    param.SetMaxStepWidth(param,0.22);//0.22
    param.SetMinStepWidth(param,0.16);
    param.SetMaxStepReach(param,sqrt((param.MaxStepWidth-param.MinStepWidth) * (param.MaxStepWidth-param.MinStepWidth) + param.MaxStepLength * param.MaxStepLength));

    std:: cout<< "gridSizeXY is "<<latticepoint.getGridSizeXY(latticepoint)<<std::endl;
    std:: cout<< "gridSizeYaw is "<<latticepoint.getGridSizeYaw(latticepoint)<<std::endl;

    std:: cout<< "EdgeCost Weight Distance is "<<param.getEdgeCostDistance(param)<<std::endl;
    std:: cout<< "EdgeCost Weight Yaw is "<<param.getEdgeCostYaw(param)<<std::endl; 
    // define the initial and final pose

    double startX = start_x_;
    double startY = 0.0;
    double startZ = 0.0;
    double startYaw = 0.0/180.0 * pi;

    // double goalX = 0.8;
    // double goalY = 0.0;
    // double goalZ = 0.0;
    // double goalYaw = -60.0/180.0 * pi;

    double goalX = param_dis.at(0);
    double goalY = param_dis.at(1);
    double goalZ = 0.0;
    double goalYaw = param_dis.at(2);

    ljh::mathlib::Pose2D<double> goalPose2D(goalX,goalY,goalYaw);
    ljh::mathlib::Pose3D<double> goalPose(goalX,goalY,goalZ,goalYaw,0.0,0.0);
    ljh::mathlib::Pose3D<double> startPose(startX,startY,startZ,startYaw,0.0,0.0);

    // double xFromGoalToStair = 0.16+0.005;
    // double xLenOfStair = 0.5;
    // double yLenOfStair = 0.5;

    // Point2D<double> p0(xFromGoalToStair,yLenOfStair/2);
    // Point2D<double> p1(xFromGoalToStair+xLenOfStair,yLenOfStair/2);
    // Point2D<double> p2(xFromGoalToStair+xLenOfStair,-yLenOfStair/2);
    // Point2D<double> p3(xFromGoalToStair,-yLenOfStair/2);

    Point2D<double> p0(param_dis.at(3), param_dis.at(4));
    Point2D<double> p1(param_dis.at(5), param_dis.at(6));
    Point2D<double> p2(param_dis.at(7), param_dis.at(8));
    Point2D<double> p3(param_dis.at(9), param_dis.at(10));

    std::vector<Point2D<double> > stairBuffer({p0,p1,p2,p3});
    // No Need of the transformation!!!!!!!!!!!
    // for(int i=0;i<4;i++)
    // {
    //     stairBuffer[i].setPoint2D(goalX + cos(goalYaw)*stairBuffer[i].getX() - sin(goalYaw)*stairBuffer[i].getY(),
    //                               goalY + sin(goalYaw)*stairBuffer[i].getX() + cos(goalYaw)*stairBuffer[i].getY());
    // }
    param.SetStairAlignMode(param,true);
    param.SetStairPolygon(param,stairBuffer,4,clockwise_);// 1 表示点为顺时针
    

    std::cout<<"initilize start! "<<std::endl;
    ljh::path::footstep_planner::AStarFootstepPlanner footstepPlanner;
    std::cout<<"initilize start 2! "<<std::endl;
    footstepPlanner.initialize(goalPose2D,goalPose,startPose);
    std::cout<<"initilize over! "<<std::endl;

    footstepPlanner.doAStarSearch();
    footstepPlanner.calFootstepSeries();
    auto Out = footstepPlanner.getOrCalAccurateFootstepSeries();
    std::cout<<"the size of the footsteps: "<<Out.size()<<std::endl;
    std::cout<< "First x y yaw"<<Out.at(0).getX() <<" "<<Out.at(0).getY() <<" "<<Out.at(0).getYaw()<<std::endl;
    std::vector<std::pair<Eigen::Vector3d, bool>> steps;
    for (auto & iter_footstep : Out)
    {
        Eigen::Vector3d foot_x_y_yaw(iter_footstep.getX(), iter_footstep.getY(), iter_footstep.getYaw());
        bool is_left = iter_footstep.getStepFlag() == 0 ? true : false;
        steps.emplace_back(make_pair(foot_x_y_yaw, is_left));
    }
    
    // ljh::path::footstep_planner::PlotChecker pltChecker;
    // auto outcome = footstepPlanner.getFootstepSeries();

    // footstepPlanner.plotAccurateSearchOutcome();
    // pltChecker.plotSearchOutcome2(outcome,goalPose,startPose);
    // pltChecker.plotAccurateSearchOutcome(Out,goalPose,startPose);
    // std::cout<<"Collide 1:"<<checker.isTwoFootCollidedAndPlot(Out.at(7))<<std::endl;
    // std::cout<<"Distance of last two: "<<
    // sqrt(pow(Out[Out.size()-1].getSecondStep().getX()-Out[Out.size()-2].getSecondStep().getX(),2)+
    // pow(Out[Out.size()-1].getSecondStep().getY()-Out[Out.size()-2].getSecondStep().getY(),2))<<std::endl;
    return steps;

    // a second time search
    //test 5
    // startX = 0.0;
    // startY = 0.0;
    // startZ = 0.0;
    // startYaw = 0.0/180.0 * pi;
    // goalX = 0.8;
    // goalY = -0.8/sqrt(3);
    // goalZ = 0.0;
    // goalYaw = -90.0/180.0 * pi;

    // goalPose2D.setPosition(goalX,goalY);
    // goalPose2D.setOrientation(goalYaw);

    // goalPose.setYawPitchRoll(goalYaw,0.0,0.0);
    // goalPose.setX(goalX);goalPose.setY(goalY);goalPose.setZ(goalZ);

    // startPose.setYawPitchRoll(startYaw,0.0,0.0);
    // startPose.setX(startX);startPose.setY(startY);startPose.setZ(startZ);

    // std::vector<Point2D<double> > stairBuffer2({p0,p1,p2,p3});
    // for(int i=0;i<4;i++)
    // {
    //     stairBuffer2[i].setPoint2D(goalX + cos(goalYaw)*stairBuffer2[i].getX() - sin(goalYaw)*stairBuffer2[i].getY(),
    //                               goalY + sin(goalYaw)*stairBuffer2[i].getX() + cos(goalYaw)*stairBuffer2[i].getY());
    // }
    // param.SetStairPolygon(param,stairBuffer2,4,1);

    // footstepPlanner.initialize(goalPose2D,goalPose,startPose);
    // footstepPlanner.doAStarSearch();
    // footstepPlanner.calFootstepSeries();
    // std::vector<ljh::path::footstep_planner::FootstepGraphNode> Out2 = footstepPlanner.getFootstepSeries();
    // pltChecker.plotSearchOutcome2(Out2,goalPose,startPose);
    // //Location test = Out2.at(7);
    // std::vector<ljh::path::footstep_planner::FootstepGraphNode> Out3;
    // Out3.push_back(Out2.at(7));
    // Out3.push_back(Out2.at(8));
    // pltChecker.plotSearchOutcome(Out3,goalPose,startPose);

    // std::cout<<"Collide 1:"<<checker.isTwoFootCollided(Out2.at(7))<<std::endl;
    // std::cout<<"Collide 2:"<<checker.isTwoFootCollided(Out2.at(8))<<std::endl;
    // std::cout<<"Distance of last two: "<<
    // sqrt(pow(Out2[Out2.size()-1].getSecondStep().getX()-Out2[Out2.size()-2].getSecondStep().getX(),2)+
    // pow(Out2[Out2.size()-1].getSecondStep().getY()-Out2[Out2.size()-2].getSecondStep().getY(),2))<<std::endl;
    // std::cout<<"Quit"<<std::endl;
}

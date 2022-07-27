



#include "DisantanceMapCoverage.h"
#include "GoalCalculator.h"
#include "AstarPathManager.h"


using namespace cv;
using namespace std;


#ifndef Map_Coverage_Manager
#define Map_Coverage_Manager

class MapCoverageManager
{
public:

    MapCoverageManager() {

        algoTimer_ = 
        node_.createTimer(ros::Rate(publishRate_),
        &MapCoverageManager::algoTimerCallback, this);

    }

    ~MapCoverageManager() {}


    void  algoTimerCallback(const ros::TimerEvent &)
    {

        if (  mapClosed_ && coverageDone_)
        {   
            return;
        }

        

    }


   
    bool updateRobotLocation() {

        tf::StampedTransform transform;

        try
        {

            //get current robot pose
            tfListener_.lookupTransform(mapFrame_, baseLinkFrame_,
                                        ros::Time(0), transform);

            robotPose_.position.x = transform.getOrigin().x();
            robotPose_.position.y = transform.getOrigin().y();
            robotPose_.position.z = 0;
            robotPose_.orientation.x = transform.getRotation().x();
            robotPose_.orientation.y = transform.getRotation().y();
            robotPose_.orientation.z = transform.getRotation().z();
            robotPose_.orientation.w = transform.getRotation().w();

            return true;
        }

        catch (...)
        {
            cerr << " error between " << mapFrame_ << " to " << baseLinkFrame_ << endl;
            return false;
        }
    }

    

    
};

#endif
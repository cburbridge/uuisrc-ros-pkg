#ifndef __SCITOSBASE__
#define __SCITOSBASE__

#include <MetraLabsBase.h>
#include <config/MLRobotic_config.h>
#include <base/Application.h>
#include <robot/Robot.h>

using namespace MetraLabs::base;
using namespace MetraLabs::robotic::base;
using namespace MetraLabs::robotic::robot;


class ScitosBase {
    
    public:
	ScitosBase(const char*, int pArgc, char* pArgv[]);
	~ScitosBase();
	
	void publish_odometry(double x, double y, double theta, double v, double w);
	void get_odometry(double& x, double& y, double& theta, double& v, double& w);
	
	void set_velocity(double v, double w);
	void loop();
	
	
    private:
	class OdometryCallbackHandler : public BlackboardDataUpdateCallback
	{
	    public:
	    OdometryCallbackHandler(ScitosBase* base) : BlackboardDataUpdateCallback() {
		m_base = base;
	    }
	    
	    void set_base(ScitosBase* base) {
		m_base = base;
	    }

	    ScitosBase* m_base;
		
	    private:
	    // Implementation of BlackboardDataUpdateCallback
	    void dataChanged(const BlackboardData* pData) {
		const BlackboardDataOdometry* tOdometryData = dynamic_cast<const BlackboardDataOdometry*>(pData);
		if (tOdometryData != NULL) {
		MTime tTime;
		Pose tPose;
		Velocity tVelocity;
		float tMileage;

		tOdometryData->getData(tPose, tVelocity, tMileage);		
		m_base->publish_odometry(tPose.getX(), tPose.getY(), tPose.getPhi(),
				    tVelocity.getVelocityTranslational(),
				    tVelocity.getVelocityRotational());	

		}
	    }
	    

	};

    private:
	Application* tApp;
	ClassFactory* tClassFactory;
	Blackboard* tBlackboard;
	Robot* tRobot;
	BlackboardDataOdometry* tOdometryData;
	BlackboardDataVelocity* tVelocityData;
	OdometryCallbackHandler tOdometryHandler;
	
	double m_x;
	double m_y;
	double m_theta;
	double m_v;
	double m_w;
	
	double m_command_v;
	double m_command_w;



};

#endif

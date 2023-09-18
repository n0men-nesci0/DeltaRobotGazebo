#include <gazebo/gazebo.hh>                
#include <gazebo/common/common.hh>       
#include <gazebo/physics/physics.hh>       
#include <ignition/math/Vector3.hh>  
#include <cmath>      
#include <vector>
#include <string>
#include <unistd.h>


#define DELTA_Z 0.2
#define DELTA_XY 0.5
#define START_TARGET_Z -0.726
#define TIME_INTERVAL 5


const double e = 0.055;     
const double f = 0.15;     
const double re = 0.79;
const double rf = 0.285;
const double sqrt3 = sqrt(3.0);
const double pi = 3.141592653;    // PI
const double sin120 = sqrt3/2.0;
const double cos120 = -0.5;
const double tan60 = sqrt3;
const double sin30 = 0.5;
const double tan30 = 1/sqrt3;

namespace gazebo {
class DeltaCalc : public ModelPlugin {
private:
    physics::ModelPtr model;	
	ignition::math::Pose3d global_pos;
	ignition::math::Pose3d local_pos;
	transport::PublisherPtr pub;
	transport::SubscriberPtr sub;
	event::ConnectionPtr hand_event;
    unsigned long long count_world_iter;
	unsigned count_robot_iter;
	bool is_action;


public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

		this->model = _model;
		this->global_pos = _model->GetLink("spider_link")->WorldPose();
		this->local_pos = ignition::math::Pose3d(0, 0, START_TARGET_Z, 0, 0, 0);

		transport::NodePtr node_pub(new transport::Node()); 
		node_pub->Init();
		this->pub = node_pub->Advertise<ignition::msgs::Vector3d>("/gazebo/delta_request");

		transport::NodePtr node_sub(new transport::Node());
		node_sub->Init();
		while(!(this->sub = node_sub->Subscribe<ignition::msgs::StringMsg, DeltaCalc>("/gazebo/delta_answer", &DeltaCalc::msg_handler, this))){
			
			this->sub = node_sub->Subscribe<ignition::msgs::StringMsg, DeltaCalc>("/gazebo/delta_answer", &DeltaCalc::msg_handler, this);
		}

		this->is_action = true;
		this->hand_event = event::Events::ConnectWorldUpdateBegin(std::bind(&DeltaCalc::work, this));
		return;
	}

	void work() {

		if (!is_action) {
			++count_world_iter;
			return;
		}

		std::cout << "Iteration " << count_robot_iter << std::endl;

		ignition::math::Pose3d target_pos = generate_point();

		std::vector<double> target_angles(3);
		bool status = inverse_kinematics(target_pos, target_angles);

		if (!status) {
			std::cerr << "Impossible to reach this point!\n";
		} 
		else {
			ignition::msgs::Vector3d msg;
			msg.set_x(target_angles[0]);
			msg.set_y(target_angles[1]);
			msg.set_z(target_angles[2]);
			this->pub->Publish(msg);
			is_action = false;
		}

		++count_robot_iter;
		++count_world_iter;
		return;
	}

	void msg_handler(const boost::shared_ptr<const ignition::msgs::StringMsg> &msg) {

		if (msg->data() == "OK") {
			std::cout << "Final position: ";
			print_pose3_vector(this->model->GetLink("spider_link")->WorldPose());
		}
		else {
			std::cerr << "Error while trying to reach this point\n";
		}	
		is_action = true;
		return;
	}	

	ignition::math::Pose3d generate_point() {

		double dx = DELTA_XY * ((random() / (double)RAND_MAX) - 0.5);
		double dy = DELTA_XY * ((random() / (double)RAND_MAX) - 0.5);
		double dz = DELTA_Z * (random() / (double)RAND_MAX);

		ignition::math::Pose3d d_vec = ignition::math::Pose3d(dx, dy, dz, 0, 0, 0);

		std::cout << "Point coordinates generated:\n";

		std::cout << "Global: ";
		print_pose3_vector(d_vec + this->global_pos);

		std::cout << "Local: ";
		print_pose3_vector(d_vec + this->local_pos);

		return this->local_pos + d_vec;
	}

	void print_pose3_vector(ignition::math::Pose3d pos) {

		std::cout << pos.X() << '\t' << pos.Y() << '\t' << pos.Z() << std::endl; 
		return;
	}

	bool calc_one_angle(double x0, double y0, double z0, double &theta) {

		double y1 = -0.5 * 0.57735 * f; 
		y0 -= 0.5 * 0.57735 * e;       
		
		double a = (x0*x0 + y0*y0 + z0*z0 + rf*rf - re*re - y1*y1)/(2*z0);
		double b = (y1 - y0)/z0;
		// Discriminant
		double d = -(a + b*y1)*(a + b*y1) + rf*(b*b*rf + rf);
		if (d < 0) 
			return false; // Impossible to reach
		double yj = (y1 - a*b - sqrt(d))/(b*b + 1); // Choose outside point
		double zj = a + b*yj;
		theta = 180.0*atan(-zj/(y1 - yj))/pi + ((yj > y1) ? 180.0 : 0.0);
		theta = theta/180 * pi; // Degrees to radians
		return true;
	}

	bool inverse_kinematics(ignition::math::Pose3d target, std::vector<double> &angles) {

		angles[0] = angles[1] = angles[2] = 0;
		double x0 = target.X();
		double y0 = target.Y();
		double z0 = target.Z();

		bool status = calc_one_angle(x0, y0, z0, angles[0]);
		if (status) {
			status &= calc_one_angle(x0*cos120 + y0*sin120, y0*cos120 - x0*sin120, z0, angles[2]); 
			if (status) {
				status &= calc_one_angle(x0*cos120 - y0*sin120, y0*cos120 + x0*sin120, z0, angles[1]); 
			}
		}

		angles[1] = -angles[1];
		angles[2] = -angles[2];

		return status && angles[0] > -0.5 && angles[0] < 2.6
					  && angles[1] > -2.6 && angles[1] < 0.5
					  && angles[2] > -2.6 && angles[2] < 0.5;
	}

};

	GZ_REGISTER_MODEL_PLUGIN(DeltaCalc);
} 


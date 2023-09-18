#include <gazebo/gazebo.hh>                
#include <gazebo/common/common.hh>       
#include <gazebo/physics/physics.hh>       
#include <ignition/math/Vector3.hh> 
#include <vector>
#include <string> 
#include <unistd.h>


#define ANGULAR_VELOCITY 0.2
#define EPS 0.001
#define TIME_INTERVAL 3


typedef struct State {
	bool is_action = false;
	int idx = -1;
	bool freezed[3];
	double angles[3];
} state_struct;



namespace gazebo {

class DeltaControl : public ModelPlugin {

private:    
    physics::ModelPtr model;
    std::vector<physics::JointPtr> arms_joints; 
    transport::PublisherPtr pub;
	transport::SubscriberPtr sub;
	event::ConnectionPtr hand_event;
	state_struct state;
	unsigned long long count_world_iter;


public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

		this->model = _model;
		this->arms_joints.push_back(_model->GetJoint("arm1u_joint"));
		this->arms_joints.push_back(_model->GetJoint("arm2u_joint"));
		this->arms_joints.push_back(_model->GetJoint("arm3u_joint"));

		this->freeze_joints();

		state.is_action = false;
		state.freezed[0] = state.freezed[1] = state.freezed[2] = true;

		transport::NodePtr node_sub(new transport::Node()); 
		node_sub->Init();
        while (!(this->sub = node_sub->Subscribe<ignition::msgs::Vector3d, DeltaControl>("/gazebo/delta_request", &DeltaControl::msg_handler, this))) {

			this->sub = node_sub->Subscribe<ignition::msgs::Vector3d, DeltaControl>
			("/gazebo/delta_request", &DeltaControl::msg_handler, this);
		}

		transport::NodePtr node_pub(new transport::Node()); 
		node_pub->Init();
		
		this->pub = node_pub->Advertise<ignition::msgs::StringMsg>("/gazebo/delta_answer");

		this->hand_event = event::Events::ConnectWorldUpdateBegin(std::bind(&DeltaControl::work, this));

		return;
	}

	void work() {

		if (!state.is_action) {
			++count_world_iter;
			return;
		}

		physics::JointPtr joint = this->arms_joints[state.idx];
		if (abs(joint->Position(0) - state.angles[state.idx]) < EPS) {

			std::cout << "Arm " << state.idx << " done!\n";

			joint->SetVelocity(0, 0);

			freeze_joint(state.idx);
			state.freezed[state.idx] = true;

			if (state.idx == 2) {
				state.is_action = false;
				state.idx = -1;
				ignition::msgs::StringMsg msg;
				msg.set_data("OK");
				this->pub->Publish(msg);
				sleep(TIME_INTERVAL);
			}
			else {
				state.idx += 1;
			}
			
			return;
		}

		if (state.freezed[state.idx]) {
			state.freezed[state.idx] = false;
			unfreeze_joint(state.idx);
		}

		if (joint->Position(0) < state.angles[state.idx]) {
			joint->SetVelocity(0, ANGULAR_VELOCITY);
		}
		else {
			joint->SetVelocity(0, -ANGULAR_VELOCITY);
		}
		++count_world_iter;
		return;
	}

    void msg_handler(const boost::shared_ptr<const ignition::msgs::Vector3d> &msg) {

		state.is_action = true;
		state.idx = 0;
		state.angles[0] = msg->x();
		state.angles[1] = msg->y();
		state.angles[2] = msg->z();

		std::cout << "message " << msg->x() << ' ' << msg->y() << ' ' << msg->z() << std::endl;

        return;
    }

    void freeze_joints() {

		for (int i = 0; i < 3; i++) {
			this->freeze_joint(i);
		}
		return;
	}

	void freeze_joint(int idx) {

		physics::JointPtr joint = this->arms_joints[idx];
		double cur_pos = joint->Position(0);
		joint->SetParam("hi_stop", 0, cur_pos);
		joint->SetParam("lo_stop", 0, cur_pos);
		return;
	}

    void unfreeze_joint(int idx) {
		
		double hi_stop, lo_stop;
		if (idx == 0) {
			hi_stop = 2.6;
			lo_stop = -0.5;
		}
		else {
			hi_stop = 0.5;
			lo_stop = -2.6;
		}  

		physics::JointPtr joint = this->arms_joints[idx];
		joint->SetParam("hi_stop", 0, hi_stop);
		joint->SetParam("lo_stop", 0, lo_stop);
		return;
	}

};
    GZ_REGISTER_MODEL_PLUGIN(DeltaControl);
}
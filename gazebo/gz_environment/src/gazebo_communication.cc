#include "gz_environment/gazebo_communication.h"
#include "gz_environment/status_helper.h"

#include <gazebo/math/Quaternion.hh>
#include <gazebo/transport/TransportIface.hh>


const static std::string TOPIC_NAME_WORLD_CONTROL = "/gazebo/default/world_control";
const static std::string TOPIC_NAME_LASERSCAN = "/gazebo/default/turtlebot2/base_footprint/laser/scan";
const static std::string TOPIC_NAME_TURTLEBOT2_VELOCITY_CMD = "/gazebo/default/turtlebot2/vel_cmd";
const static std::string TOPIC_NAME_TURTLEBOT2_MODEL_MODIFY = "/gazebo/default/model/modify";

const static std::string ROBOT_NAME = "turtlebot2";
const static int STD_WORLD_STEPS = 50;
const static int STD_TO_START_POSITION_CYCLES = 3;

GazeboCommunication::GazeboCommunication()
{
    initialised = false;
    cycles_to_start_position = 0;

    this->state_laser = GazeboCommunication::STATE_LASER::UNSYNC;
}

GazeboCommunication::~GazeboCommunication()
{
    gazebo::client::shutdown();
}

void GazeboCommunication::subscriber_callback_laser(ConstLaserScanStampedPtr &laserscan)
{
    // Reset the turtelbot to the startpositio if the cycle counter is not zero
    if (0 < cycles_to_start_position) {
        send_model_modify_turtlebot2_pose(input_processing.get_pose_start());
        send_velocity_cmd(0,0);
        send_world_control_steps(STD_WORLD_STEPS);

        cycles_to_start_position--;
    }
    // Process the laserscan message when the laserscan is snychronized
    else if(this->get_state_laser() == GazeboCommunication::STATE_LASER::SYNCED)
    {
        // Process laserscan message to protobuffer message
        std::shared_ptr<MsgToAgent> msg = input_processing.process(laserscan);

        if (msg->done())
        {
            cycles_to_start_position = STD_TO_START_POSITION_CYCLES;
            send_model_modify_turtlebot2_pose(input_processing.get_pose_start());
            send_velocity_cmd(0,0);
            send_world_control_steps(STD_WORLD_STEPS);
            resets++;
        }

        signal_process_msg_to_agent(msg);

        MESSAGE_INFO("iteration: " << iterations << "  resets: " << resets << "  reward: " << msg->reward());

    }
    // Set the laserscan state to snchronized when the first laserscan message is arrived.
    else if (this->get_state_laser() == GazeboCommunication::STATE_LASER::SYNCING)
    {
        this->state_laser = GazeboCommunication::STATE_LASER::SYNCED;
    }
}

void GazeboCommunication::send_velocity_cmd(const double &linear_velocity, const double &angular_velocity)
{
    gazebo::msgs::Pose msg_pose;
    gazebo::msgs::Vector3d *msg_vector = new gazebo::msgs::Vector3d;
    gazebo::msgs::Quaternion *msg_quaternion = new gazebo::msgs::Quaternion;

    gazebo::math::Quaternion quaternion;

    // calculate angular velocity from Euler to quaternion
    quaternion.SetFromEuler(0.0, 0.0, angular_velocity);

    // set angular velocity
    msg_quaternion->set_w(quaternion.w);
    msg_quaternion->set_x(quaternion.x);
    msg_quaternion->set_y(quaternion.y);
    msg_quaternion->set_z(quaternion.z);

    // set linaer velocity
    msg_vector->set_x(linear_velocity);
    msg_vector->set_y(0.0);
    msg_vector->set_z(0.0);

    msg_pose.set_allocated_orientation(msg_quaternion);
    msg_pose.set_allocated_position(msg_vector);

    publisher_turtlebot2_velocity_cmd->Publish(msg_pose);
}

void GazeboCommunication::send_world_control_pause()
{
    gazebo::msgs::WorldControl msg_world_control;

    msg_world_control.set_pause(true);

    this->publisher_world_control->Publish(msg_world_control);
}

void GazeboCommunication::send_world_control_steps(int steps)
{
    gazebo::msgs::WorldControl msg_world_control;

    msg_world_control.set_pause(true);
    msg_world_control.set_step(true);
    msg_world_control.set_multi_step(steps);

    this->publisher_world_control->Publish(msg_world_control);
}

bool GazeboCommunication::init(int argc, char *argv[])
{
    gazebo::transport::get_master_uri(address, port);

    // init gazebo communication
    if (!gazebo::client::setup(argc, argv))
    {
        MESSAGE_ERROR("Gazebo client setup failed!");
        return false;
    }

    MESSAGE_INFO("Connecting to Gazebo Server.");
    MESSAGE_INFO("address: " << address);
    MESSAGE_INFO("port:    " << port);

    this->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
    this->node->Init();

    this->publisher_world_control = this->node->Advertise<gazebo::msgs::WorldControl>(TOPIC_NAME_WORLD_CONTROL);
    this->publisher_turtlebot2_velocity_cmd = this->node->Advertise<gazebo::msgs::Pose>(TOPIC_NAME_TURTLEBOT2_VELOCITY_CMD);
    this->publisher_turtlebot2_model_modify = this->node->Advertise<gazebo::msgs::Model>(TOPIC_NAME_TURTLEBOT2_MODEL_MODIFY);

    this->subscriber_laserscan = this->node->Subscribe(TOPIC_NAME_LASERSCAN, &GazeboCommunication::subscriber_callback_laser, this);

    // Wait for the publisher to connected to the gazebo sever
    this->publisher_world_control->WaitForConnection();
    this->publisher_turtlebot2_velocity_cmd->WaitForConnection();
    this->publisher_turtlebot2_model_modify->WaitForConnection();

    MESSAGE_INFO("Connecting done.");
}

unsigned short GazeboCommunication::get_port()
{
    return port;
}

std::string GazeboCommunication::get_address()
{
    return address;
}

void GazeboCommunication::process_msg_to_environment(std::shared_ptr<MsgToEnvironment> msg)
{
    this->send_velocity_cmd(msg->linear_velocity(), msg->angular_velocity());

    iterations++;

    this->send_world_control_steps(STD_WORLD_STEPS);
}

boost::signals2::signal<void (std::shared_ptr<MsgToAgent>)> &GazeboCommunication::get_signal_process_msg_to_agent()
{
    return signal_process_msg_to_agent;
}

void GazeboCommunication::send_world_control_resume()
{
    gazebo::msgs::WorldControl msg_world_control;

    msg_world_control.set_pause(false);

    this->publisher_world_control->Publish(msg_world_control);
}

bool GazeboCommunication::is_initialised()
{
    return initialised;
}

GazeboCommunication::STATE_LASER GazeboCommunication::get_state_laser()
{
    return this->state_laser;
}

void GazeboCommunication::set_state_laser(const GazeboCommunication::STATE_LASER state)
{
    this->state_laser = state;
}

void GazeboCommunication::send_model_modify_turtlebot2_pose(const gazebo::math::Pose &pose)
{
    gazebo::msgs::Model msg_model;

    gazebo::msgs::Pose *msg_pose = new gazebo::msgs::Pose();

    gazebo::msgs::Quaternion *msg_quaternion = new gazebo::msgs::Quaternion();
    gazebo::msgs::Vector3d *msg_vector = new gazebo::msgs::Vector3d();

    // set rotation
    msg_quaternion->set_w(pose.rot.w);
    msg_quaternion->set_x(pose.rot.x);
    msg_quaternion->set_y(pose.rot.y);
    msg_quaternion->set_z(pose.rot.z);

    // set position
    msg_vector->set_x(pose.pos.x);
    msg_vector->set_y(pose.pos.y);
    msg_vector->set_z(pose.pos.z);

    msg_pose->set_allocated_orientation(msg_quaternion);
    msg_pose->set_allocated_position(msg_vector);

    msg_model.set_allocated_pose(msg_pose);


    msg_model.set_name("turtlebot2");

    publisher_turtlebot2_model_modify->Publish(msg_model,true);
}

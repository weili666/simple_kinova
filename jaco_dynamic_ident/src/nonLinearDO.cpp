#include <disturbance_observer/nonLinearDO.h>

nonLinearDO::nonLinearDO(ros::NodeHandle nh)
{
    disturbancePublisher = nh.advertise<std_msgs::Float32MultiArray>("/jaco_disturbance_data", 1);
    disturb_vector.setZero();
    //force_vector.clear();
    JacoParamVector g_param;
    g_param << 11.835024,
            1.6362565,
            2.0623081,
            1.8572530,
            1.3655201,
            1.6619842,
            0.38202310,
            1.0875487,
            2.0258353,
            1.0346359,
            1.5005500,
            -1.7717659;
    regressor.setParam(g_param);
    observer.setTsample(0.033);
    JointVector gain;
    gain.setOnes();
    gain *= 5;
    observer.setGain(gain);

    regressor.printParam();
    observer.printParam();
    nonDOSubscriber = nh.subscribe("/jaco_arm/joint_cartesian_states", 1, &nonLinearDO::NdoCallback, this);
}

nonLinearDO::~nonLinearDO()
{
}

void nonLinearDO::NdoCallback(const wpi_jaco_msgs::JointPosiCartForce::ConstPtr& joint_cartesian_states)
{
    //Get realtime execution data(position, velocity, effort)
    std::vector<double> posi_vector, vel_vector, posivel_vector;
    posi_vector.assign(joint_cartesian_states->joint_state.position.begin(), joint_cartesian_states->joint_state.position.end());
    vel_vector.assign(joint_cartesian_states->joint_state.velocity.begin(), joint_cartesian_states->joint_state.velocity.end());
    //posivel_vector.assign(joint_cartesian_states->joint_state.position.begin(), joint_cartesian_states->joint_state.position.end());
    //posivel_vector.insert(posivel_vector.end(), vel_vector.begin(), vel_vector.end());
    for (unsigned int i = 0; i < 6; i++)
    {
        posivel_vector.push_back(posi_vector[i]);
    }
    for (unsigned int i = 0; i < 6; i++)
    {
        posivel_vector.push_back(vel_vector[i]);
    }

    std::vector<double> torque_command_vector;
    torque_command_vector.assign(joint_cartesian_states->joint_state.effort.begin(), joint_cartesian_states->joint_state.effort.end());
    JointVector torque_command;
    for (unsigned int i = 0; i < 6; i++)
    {
        torque_command(i) = torque_command_vector[i];
    }

    //Compute torque generated from nn nonlinear compensator
    JointVector torque_compensator;
    compensator.genJointTorque(posivel_vector,  torque_compensator);
    //Compute torque generated from gravity regressor and linear parameter
    JointVector torque_gravity;
    regressor.genJointTorque(posi_vector, torque_gravity);
    //Compute the totally inverse dynamic torque as the sum of gravity torque and nonlinear torque
    JointVector torque_invdyn = torque_compensator + torque_gravity;

    //Compute the corresponding disturbance observer output
    observer.updateDO(disturb_vector, torque_command, torque_invdyn);

    //Compute the Cartesian force corresponding to external disturbance
    std::vector<double> disturb_vector_form;
    disturb_vector_form.clear();
    for (unsigned int i = 0; i < 6; i++)
    {
        disturb_vector_form.push_back(disturb_vector(i));

    }

    std::vector<double> force_vector;
    converter.genCatersianForce(posi_vector, disturb_vector_form, force_vector);

    std_msgs::Float32MultiArray d_data;
    for (unsigned int i = 0; i < 6; i++)
    {
        d_data.data.push_back(force_vector[i]);
        //d_data.data.push_back(disturb_vector(i));
        //d_data.data.push_back(torque_gravity[i]);
    }
    disturbancePublisher.publish(d_data);
}

#include <iostream>
#include <vector>
#include <algorithm>

#include <visp_naoqi/vpNaoqiGrabber.h>
#include <vpRomeoTkConfig.h>

#include <visp3/gui/vpDisplayX.h>

#include "vs_grasping_pepper.h"
#include "recorded_motion.h"

vpDisplayX d;

vs_grasping_pepper::vs_grasping_pepper(ros::NodeHandle &nh): m_cam(),  m_camInfoIsInitialized(false)
{
  // read in config options
  n = nh;

  m_state = GoToInitialPosition;

  m_cMh_isInitialized = false;
  m_cMdh_isInitialized = false;

  m_statusPoseHand = 0;
  m_statusPoseDesired = 0;
  m_servo_time_init = 0;
  m_offset.setIdentity();

  n.param( "frequency", freq, 50);
  n.param<std::string>("ip", m_ip, "131.254.10.126");
  n.param<std::string>("actualPoseTopicName", actualPoseTopicName, "/pepper_hand_pose");
  n.param<std::string>("desiredPoseTopicName", desiredPoseTopicName, "/desired_pepper_hand_pose");
  n.param<std::string>("cmdVelTopicName", cmdVelTopicName, "/joint_state");
  n.param<std::string>("cameraInfoName", m_cameraInfoName, "/camera/camera_info");
  n.param<std::string>("statusPoseHandTopicName", statusPoseHandTopicName, "/pepper_hand_pose/status");
  n.param<std::string>("opt_arm", m_opt_arm, "right");
  n.param<std::string>("eMh_pepper_path", m_eMh_pepper_path, "");
  n.param<std::string>("offsetFileName", m_offsetFileName, "pose.xml");
  n.param("statusPoseDesired_isEnable", m_statusPoseDesired_isEnable, false );
  n.param("mode", m_mode, 0);

  if (m_mode == 0)
    std::cout << "Mode 0: Visual servoing mode" << std::endl;

  if (m_mode == 1)
  {
    m_state = Learning;
    std::cout << "Mode 1: save pose and servo" << std::endl;
  }
  if (m_mode == 2)
  {
    m_state = Learning;
    std::cout << "Mode 2: save offset from object to hand" << std::endl;
  }

  if (m_mode < 0 || m_mode > 2)
  {
    std::cout <<"ERROR: Mode not recognized. Please choose a mode between 0 and 2. Used default mode 0" << std::endl;
    m_mode = 0;
  }

  // Initialize subscriber and publisher
  if ( m_mode > 0 )
  {
    n.param<std::string>("StatusPoseDesiredTopicName", statusPoseDesiredTopicName, "/tracker_mbt/status");
    statusPoseDesiredSub = n.subscribe ( statusPoseDesiredTopicName, 1, (boost::function < void(const std_msgs::Int8::ConstPtr  &)>) boost::bind( &vs_grasping_pepper::getStatusPoseDesiredCb, this, _1 ));
  }
  else
    m_statusPoseDesired = 1;

  desiredPoseSub = n.subscribe( desiredPoseTopicName, 1, (boost::function < void(const geometry_msgs::TransformStampedConstPtr&)>) boost::bind( &vs_grasping_pepper::getDesiredPoseCb, this, _1 ));
  actualPoseSub = n.subscribe( actualPoseTopicName, 1, (boost::function < void(const geometry_msgs::PoseStampedConstPtr &)>) boost::bind( &vs_grasping_pepper::getActualPoseCb, this, _1 ));
  statusPoseHandSub = n.subscribe ( statusPoseHandTopicName, 1, (boost::function < void(const std_msgs::Int8::ConstPtr  &)>) boost::bind( &vs_grasping_pepper::getStatusPoseHandCb, this, _1 ));
  m_cameraInfoSub = n.subscribe( m_cameraInfoName, 1, (boost::function < void(const sensor_msgs::CameraInfoConstPtr & )>) boost::bind( &vs_grasping_pepper::getCameraInfoCb, this, _1 ));

  //cmdVelPub = n.advertise<sensor_msgs::JointState >(cmdVelTopicName, 10);

  if (m_opt_arm == "right")
    m_chain_name = "RArm";
  else
    m_chain_name = "LArm";

  std::string name_transform = "eMh_" + m_chain_name;
  vpXmlParserHomogeneousMatrix pm; // Create a XML parser

  if (pm.parse(m_eMh, m_eMh_pepper_path, name_transform) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
    std::cout << "Cannot found the homogeneous matrix named " << name_transform << "." << std::endl;
    ros::shutdown();
  }
  else
    std::cout << "Homogeneous matrix " << name_transform <<": " << std::endl << m_eMh << std::endl;

  if (m_mode == 0)
  {
    std::string name_transform_offset = "dhMoffset_"+ m_chain_name;

    if (pm.parse(m_offset, m_offsetFileName, name_transform_offset) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
      std::cout << "Cannot find the homogeneous matrix named " << name_transform_offset << "." << std::endl;
      ros::shutdown();
    }
    else
      std::cout << "Homogeneous matrix " << name_transform_offset <<": " << std::endl << m_eMh << std::endl;
  }

  // Init display
  I.init(480, 640);
  d.init(I);
  vpDisplay::setTitle(I, "ViSP viewer");
  // vpDisplay::display(I);
  //vpDisplay::flush(I);

  ROS_INFO("Launch vs_grasping_pepper_node");
  // Connect to the robot
  if (! m_ip.empty())
    robot.setRobotIp(m_ip);
  robot.open();
  if (robot.getRobotType() != vpNaoqiRobot::Pepper)
  {
    std::cout << "ERROR: You are not connected to Pepper, but to a different Robot. Check the IP. " << std::endl;
    exit (EXIT_FAILURE);
  }
  robot.open();

  m_jointNames_arm =  robot.getBodyNames(m_chain_name);
  m_jointNames_arm.pop_back(); // Delete last joints LHand, that we don't consider in the servo

  std::cout << "Controlling joints: " << m_jointNames_arm <<": " << std::endl << m_eMh << std::endl;

  m_numJoints = m_jointNames_arm.size();
  m_q.resize(m_numJoints);
  m_q_dot.resize(m_numJoints);
  m_q2_dot.resize(m_numJoints);
  m_q_dot_msg.velocity.resize(m_numJoints);
  m_q_dot_msg.name = m_jointNames_arm;
  m_jointMin.resize(m_numJoints);
  m_jointMax.resize(m_numJoints);

  //Get joint limits
  robot.getJointMinAndMax(m_jointNames_arm, m_jointMin, m_jointMax);

  //Set the stiffness
  robot.setStiffness(m_jointNames_arm, 1.f);
  // Disable anti-collision arm
  robot.getProxy()->setExternalCollisionProtectionEnabled(m_chain_name, false);

  m_servo_enabled = false;

}

vs_grasping_pepper::~vs_grasping_pepper(){

}

void vs_grasping_pepper::spin()
{
  ros::Rate loop_rate(freq);
  vpDisplay::display(I);


  while(!m_camInfoIsInitialized)
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  while(ros::ok()){
    vpDisplay::display(I);

    vpMouseButton::vpMouseButtonType button;
    bool ret = vpDisplay::getClick(I, button, false);

    // Display useful informations
    if (m_cMdh_isInitialized)
    {
      vpDisplay::displayFrame(I, m_cMdh, m_cam, 0.06, vpColor::red);
      vpDisplay::displayFrame(I, m_cMdh*m_offset, m_cam, 0.06, vpColor::cyan);
    }
    if (m_statusPoseHand)
      vpDisplay::displayFrame(I, m_cMh, m_cam, 0.06, vpColor::none);

    if (m_state == Learning)
    {
      if (m_mode == 1) // Test mode: save a desired pose
      {
        if (ret && button == vpMouseButton::button2)
        {
          m_cMdh = m_cMh;
          m_cMdh_isInitialized = true;
          std::cout << "Desired Pose saved" << std::endl;
        }
      }

      if (m_mode == 2) // learn desired pose of the hand wrt the object
      {
        vpDisplay::displayText(I, 30, 30, "Press middle button to save the offset", vpColor::green);
        if (ret && button == vpMouseButton::button2)
          this->saveOffset();
      }
    }

    if (m_state == GoToInitialPosition)
    {
      vpDisplay::displayText(I, 30, 30, "Click to move the robot to the initial position", vpColor::green);

      if (ret && button == vpMouseButton::button1)
      {
        if(goTointialPose(robot.getProxy()))
          m_state = Servoing;
        ret = false;
      }
    }

    if (m_state == Servoing)
    {

      if (ret && button == vpMouseButton::button2)
      {
        m_servo_enabled = !m_servo_enabled;
        robot.setStiffness(m_jointNames_arm, 1.f);
        ret = false;
      }

      if (m_mode == 0 || m_mode == 1 )
      {
        if (m_servo_enabled && !this->computeControlLaw())
        {
          vpDisplay::displayText(I, 30, 30, "Servo enabled", vpColor::green);
        }
        else
        {
          vpDisplay::displayText(I, 30, 30, "Servo disable or finished: middle click to start the VS, left click to grasp", vpColor::green);
          robot.stop(m_jointNames_arm);
          //vpColVector q_dot_zero(m_numJoints,0);
          //publishCmdVel(q_dot_zero);
        }
      }

      if (ret && button == vpMouseButton::button1)
      {
        robot.stop(m_jointNames_arm);
        m_state = Grasp;
        ret = false;
      }
    }

    if (m_state == Grasp)
    {
      robot.stopPepperControl();
      vpDisplay::displayText(I, 30, 30, "Closing", vpColor::green);
      std::string  hand = "RHand";
      robot.getProxy()->setStiffnesses(hand, 1.0f);
      AL::ALValue angle = 0.0;
      robot.getProxy()->setAngles(hand, angle, 0.9);

      if (ret && button == vpMouseButton::button1)
      {
        m_state = RaiseArm;
        ret = false;
      }
    }

    if (m_state == RaiseArm)
    {
      bool static first_time = true;
      vpDisplay::displayText(I, 30, 30, "Raise the box", vpColor::green);
      if (first_time)
      {
        std::string  joint_name = "RShoulderPitch";
        robot.getProxy()->setStiffnesses(joint_name, 1.0f);
        vpColVector p = robot.getPosition(joint_name,true);
        std::cout << "Actual angle: " <<  p[0] << std::endl;
        AL::ALValue angle = p[0] - vpMath::rad(10.);
        std::cout << "Actual angle: " <<  angle << std::endl;
        robot.getProxy()->setAngles(joint_name, angle, 0.02);
        first_time = false;
      }
      //robot.getProxy()->closeHand("RHand");

      if (ret && button == vpMouseButton::button1)
      {
        m_state = OpenHand;
        ret = false;
      }
    }

    if (m_state == LowerArm)
    {
      bool static first_time = true;

      vpDisplay::displayText(I, 30, 30, "Raise the box", vpColor::green);
      if (first_time)
      {
        std::string  joint_name = "RShoulderPitch";
        robot.getProxy()->setStiffnesses(joint_name, 1.0f);
        vpColVector p = robot.getPosition(joint_name,true);
        AL::ALValue angle = p[0] + vpMath::rad(3.);
        robot.getProxy()->setAngles(joint_name, angle, 0.02);
        first_time = false;
      }

      if (ret && button == vpMouseButton::button1)
      {
        m_state = OpenHand;
        ret = false;
      }
    }

    if (m_state == OpenHand)
    {
      vpDisplay::displayText(I, 30, 30, "Click to open the arm", vpColor::green);
      if (ret && button == vpMouseButton::button1)
      {
        std::string  hand = "RHand";
        robot.getProxy()->setStiffnesses(hand, 1.0f);
        AL::ALValue angle = 1.00;
        robot.getProxy()->setAngles(hand, angle, 0.5);
        m_state = End;
        ret = false;
      }
    }




    if (m_state == End)
    {
      vpDisplay::displayText(I, 30, 30, "Click to move back the arm", vpColor::green);


      if (ret && button == vpMouseButton::button1)
      {
        std::cout << "+++++++++++++++++ move back the arm" << std::endl;
        backTointialPose(robot.getProxy());
      }

    }

    if (ret && button == vpMouseButton::button3)
      break;

    ret = false;
    ros::spinOnce();
    vpDisplay::flush(I);

    loop_rate.sleep();
  }
}


void vs_grasping_pepper::saveOffset()
{

  vpHomogeneousMatrix dhMoffset = m_cMdh.inverse() * m_cMh;
  vpXmlParserHomogeneousMatrix xml;
  std::string path = "/tmp/dhMoffset_pepper.xml";

  if (m_cMh_isInitialized && m_cMdh_isInitialized && m_statusPoseHand && m_statusPoseDesired )
  {
    ROS_INFO_STREAM("cMdh = " << m_cMdh << "\n cMh = " << m_cMh << "\n dhMh = " << dhMoffset);
    if( xml.save(dhMoffset, path.c_str(),"dhMoffset") == vpXmlParserHomogeneousMatrix::SEQUENCE_OK )
      std::cout << "Pose between the hand and the object successfully saved in \"" << path << "\"" << std::endl;
    else {
      std::cout << "Failed to save the pose in \"" << path << "\"" << std::endl;
      std::cout << "A file with the same name exists. Remove it to be able to save the parameters." << std::endl;
    }
    ros::shutdown();
  }
  else
  {
    std::cout << "m_cMh_isInitialized: " << m_cMh_isInitialized << std::endl
              << "m_cMdh_isInitialized: " << m_cMdh_isInitialized << std::endl
              << "m_statusPoseHand: " << m_statusPoseHand << std::endl
              << "m_statusPoseDesired: " << m_statusPoseDesired << std::endl;
  }


}



bool vs_grasping_pepper::computeControlLaw()
{
  bool vs_finished = false;

  if ( m_cMh_isInitialized && m_cMdh_isInitialized  && m_statusPoseHand && m_statusPoseDesired)
  {
    static bool first_time = true;
    if (first_time) {
      std::cout << "-- Start visual servoing of the arm" << std::endl;
      m_servo_time_init = vpTime::measureTimeSecond();
      first_time = false;
    }

    vpAdaptiveGain lambda(0.5, 0.05, 3);
    m_servo_arm.setLambda(lambda);
    m_servo_arm.set_eJe(robot.get_eJe(m_chain_name));
    vpHomogeneousMatrix cdMc = m_offset.inverse() * m_cMdh.inverse() * m_cMh;
    m_servo_arm.setCurrentFeature(cdMc) ;
    // Create twist matrix from target Frame to Arm end-effector (WristPitch)
    vpVelocityTwistMatrix oVe_LArm(m_eMh);
    m_servo_arm.m_task.set_cVe(oVe_LArm);

    //Compute velocities PBVS task
    m_q_dot = - m_servo_arm.computeControlLaw(vpTime::measureTimeSecond() - m_servo_time_init);

    m_q = robot.getPosition(m_jointNames_arm);
    m_q2_dot  = m_servo_arm.m_task.secondaryTaskJointLimitAvoidance(m_q, m_q_dot, m_jointMin, m_jointMax);

    // publishCmdVel(m_q_dot + m_q2_dot);
    robot.setVelocity(m_jointNames_arm, m_q_dot + m_q2_dot);

    vpTranslationVector t_error_grasp = cdMc.getTranslationVector();
    vpRotationMatrix R_error_grasp;
    cdMc.extract(R_error_grasp);
    vpThetaUVector tu_error_grasp;
    tu_error_grasp.buildFrom(R_error_grasp);
    double theta_error_grasp;
    vpColVector u_error_grasp;
    tu_error_grasp.extract(theta_error_grasp, u_error_grasp);
    std::cout << "Error t: " << sqrt(t_error_grasp.sumSquare()) << "< 0.024 " << std::endl <<
                 "Error r: " << theta_error_grasp << "< " << vpMath::rad(5) << std::endl;

        if ( (sqrt(t_error_grasp.sumSquare()) < 0.014) && (theta_error_grasp < vpMath::rad(10)) )
        {
          vs_finished = true;
          m_state = Grasp;
          robot.stop(m_jointNames_arm);
        }

  }
  else
    robot.stop(m_jointNames_arm);

  return vs_finished;

}

//void vs_grasping_pepper::publishCmdVel(const vpColVector &q)
//{
//  for (int i = 0; i < q.size(); i++)
//  {
//    m_q_dot_msg.velocity[i] = q[i];
//  }

//  cmdVelPub.publish(m_q_dot_msg);

//}


void vs_grasping_pepper::getDesiredPoseCb(const geometry_msgs::TransformStamped::ConstPtr &desiredPose)
{
  m_cMdh = visp_bridge::toVispHomogeneousMatrix(desiredPose->transform);

  if ( !m_cMdh_isInitialized )
  {
    ROS_INFO("DesiredPose received");
    m_cMdh_isInitialized = true;
  }

}


void vs_grasping_pepper::getActualPoseCb(const geometry_msgs::PoseStamped::ConstPtr &actualPose)
{
  m_cMh = visp_bridge::toVispHomogeneousMatrix(actualPose->pose);
  if ( !m_cMh_isInitialized )
  {
    ROS_INFO("ActualPose received");
    m_cMh_isInitialized = true;
  }
}

void vs_grasping_pepper::getStatusPoseHandCb(const std_msgs::Int8::ConstPtr  &status)
{
  m_statusPoseHand = status->data;
  // std::cout << "Status_pose_hand" << m_statusPoseHand << std::endl;
}

void vs_grasping_pepper::getStatusPoseDesiredCb(const std_msgs::Int8::ConstPtr  &status)
{
  m_statusPoseDesired = status->data;
}

void vs_grasping_pepper::getCameraInfoCb(const sensor_msgs::CameraInfoConstPtr &msg)
{
  std::cout << "Received Camera INFO"<<std::endl;
  // Convert the paramenter in the visp format
  m_cam = visp_bridge::toVispCameraParameters(*msg);
  m_cam.printParameters();

  // Stop the subscriber (we don't need it anymore)
  this->m_cameraInfoSub.shutdown();

  m_camInfoIsInitialized = 1;
}




#include <iostream>
#include <vector>
#include <algorithm>

//#include <visp_naoqi/vpNaoqiGrabber.h>
//#include <vpRomeoTkConfig.h>


#include <visp/vpFeatureBuilder.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpMeterPixelConversion.h>

#include "vs_grasping_pepper.h"
#include "recorded_motion.h"


vs_grasping_pepper::vs_grasping_pepper(ros::NodeHandle &nh): m_cam(),  m_camInfoIsInitialized(false), m_camIsInitialized(false), m_it(nh), m_points(), robot()

{
  // read in config options
  n = nh;

  m_state = Init;//Init; //GoToInitialPosition;

  m_cMh_isInitialized = false;
  m_cMdh_isInitialized = false;
  m_statusPoseHand = 0;
  m_statusPoseDesired = 0;
  m_servo_time_init = 0;
  m_command_give_box = false;
  m_offset.eye();

  n.param( "frequency", freq, 20);
  n.param<std::string>("ip", m_ip, "131.254.10.126");
  n.param<std::string>("actualPoseTopicName", actualPoseTopicName, "/pepper_hand_pose");
  n.param<std::string>("desiredPoseTopicName", desiredPoseTopicName, "/desired_pepper_hand_pose");
  n.param<std::string>("cmdVelTopicName", cmdVelTopicName, "/joint_state");
  n.param<std::string>("cameraInfoName", m_cameraInfoName, "/pepper_robot/camera/bottom/camera_info");
  n.param<std::string>("cameraTopicName", m_cameraTopicName, "/pepper_robot/camera/bottom/image_rect_color");
  n.param<std::string>("statusPoseHandTopicName", statusPoseHandTopicName, "/pepper_hand_pose/status");
  n.param<std::string>("statusObjectPolygonTopicName", m_statusObjectPolygonTopicName, "/vision/predator_alert/status");
  n.param<std::string>("ObjectPolygonTopicName", m_objectPolygonTopicName, "/vision/predator_alert");
  n.param<std::string>("pointArrayName", m_pointArrayName, "/whycon/points");
  n.param<std::string>("opt_arm", m_opt_arm, "right");
  n.param<std::string>("eMh_pepper_path", m_eMh_pepper_path, "");
  n.param<std::string>("offsetFileName", m_offsetFileName, "pose.xml");
  n.param("statusPoseDesired_isEnable", m_statusPoseDesired_isEnable, false );
  n.param("mode", m_mode, 0);
  
  
  // Create a session to connect with the Robot
  m_session = qi::makeSession();
  std::string ip_port = "tcp://" + m_ip + ":9559";
  m_session->connect(ip_port);


  robot = new vpNaoqiRobot(m_session);
  robot->open();
  

  if (m_mode == 0)
    std::cout << "Mode 0: Visual servoing mode" << std::endl;

  if (m_mode == 1)
  {
    m_state = Learning;
    std::cout << "Mode 1: save pose and servo" << std::endl;
  }
  if (m_mode == 2 || m_mode == 3)
  {
    m_state = Learning;
    std::cout << "Mode 2: save offset from object to hand" << std::endl;
  }

  if (m_mode < 0 || m_mode > 3)
  {
    std::cout <<"ERROR: Mode not recognized. Please choose a mode between 0 and 3. Used default mode 0" << std::endl;
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
  m_statusObjectPolygonSub = n.subscribe( m_statusObjectPolygonTopicName, 1, (boost::function < void(const std_msgs::Int8::ConstPtr & )>) boost::bind( &vs_grasping_pepper::getStatusObjectPolygonCb, this, _1 ));
  m_ObjectPolygonSub = n.subscribe( m_objectPolygonTopicName, 1, (boost::function < void(const geometry_msgs::Polygon::ConstPtr & )>) boost::bind( &vs_grasping_pepper::getObjectPolygonCb, this, _1 ));
  m_pointArraySub = n.subscribe( m_pointArrayName, 1, (boost::function < void(const whycon::PointArrayConstPtr &)>) boost::bind( &vs_grasping_pepper::getPointArrayCb, this, _1 ));

  m_itSub = m_it.subscribe(m_cameraTopicName, 1, (boost::function < void(const sensor_msgs::ImageConstPtr&)>) boost::bind( &vs_grasping_pepper::imageCb, this, _1 ));

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
  // get trasformation from HeadRool to Bottom CAmera
  m_eMc = vpNaoqiGrabber::getExtrinsicCameraParameters("CameraBottomPepper",vpCameraParameters::perspectiveProjWithDistortion);

  // Init display
  // I.resize(480, 640, 0);
  // d.init(I);
  // vpDisplay::setTitle(I, "ViSP viewer");
  // vpDisplay::display(I);
  //vpDisplay::flush(I);

  ROS_INFO("Launch vs_grasping_pepper_node");

  // Iniziale motion proxy
  m_pMotion = m_session->service("ALMotion");

  m_jointNames_head = robot->getBodyNames("Head");
  m_q_head.resize(m_jointNames_head.size());

  m_jointNames_arm =  robot->getBodyNames(m_chain_name);
  m_jointNames_arm.pop_back(); // Delete last joints LHand, that we don't consider in the servo

  //std::cout << "Controlling joints: " << m_jointNames_arm <<": " << std::endl << m_eMh << std::endl;

  m_numJoints = m_jointNames_arm.size();
  m_q.resize(m_numJoints);
  m_q_dot.resize(m_numJoints);
  m_q2_dot.resize(m_numJoints);
  m_q_dot_msg.velocity.resize(m_numJoints);
  m_q_dot_msg.name = m_jointNames_arm;
  m_jointMin.resize(m_numJoints);
  m_jointMax.resize(m_numJoints);

  //Get joint limits
  robot->getJointMinAndMax(m_jointNames_arm, m_jointMin, m_jointMax);

  // Disable anti external collision arm
  robot->setExternalCollisionProtectionEnabled(m_chain_name, false);
  // Disable anti external collision base
  robot->setExternalCollisionProtectionEnabled("Move", false);

  m_servo_enabled = false;
  m_pbvs_base = true;

  // Jacobian 6x3 (vx,vy,wz)
  m_tJe.resize(6,3);
  m_tJe[0][0]= 1;
  m_tJe[1][1]= 1;
  m_tJe[5][2]= 1;
  //vpMatrix eJe(6,3);
  m_base_vel.resize(3);

  m_bodyJointNames = robot->getBodyNames("Body");
  m_bodyJointValues.resize(m_bodyJointNames.size());

  robot->getMotionProxy()->call<void>("setSmartStiffnessEnabled", false);
  robot->getMotionProxy()->call<void>("setMoveArmsEnabled", false, false);
  robot->setStiffness("Body", 1.0);

  // Initialize memory and speech proxies
  m_pMemory = m_session->service("ALMemory");
  m_pSpeechRecognition = new qi::AnyObject(m_session->service("ALSpeechRecognition"));
  m_pTextToSpeech = m_session->service("ALTextToSpeech");

  //m_mem_proxy = new AL::ALMemoryProxy (m_ip, 9559);
  //m_asr_proxy = new AL::ALSpeechRecognitionProxy (m_ip, 9559);
  m_vocabulary.push_back("Pepper, Can you bring me the box of tabasco?");
  m_vocabulary.push_back("give me the box please");

  m_pSpeechRecognition->call<void>("pause", true);
  m_pSpeechRecognition->call<void>("setVisualExpression", false);
  m_pSpeechRecognition->call<void>("setLanguage","English");

  // Set the vocabulary
  m_pSpeechRecognition->call<void>("setVocabulary", m_vocabulary, false);

  // Start the speech recognition engine with user Test_m_asr_proxy
  m_pSpeechRecognition->call<void>("pause", false);
  m_pSpeechRecognition->call<void>("subscribe", "request_ASR");

  m_pTextToSpeech.call<void>("setLanguage", "English");


  // PBVS Visual Servoing, using information coming from the MBT

  m_t.setFeatureTranslationType(vpFeatureTranslation::cdMc);
  m_tu.setFeatureThetaURotationType(vpFeatureThetaU::cdRc);

  // Build the desired visual feature s* = (0,0)
  m_s_star_t.setFeatureTranslationType(vpFeatureTranslation::cdMc); // Default initialization to zero
  m_s_star_tu.setFeatureThetaURotationType(vpFeatureThetaU::cdRc);// Default initialization to zero

  // We want to see a point on a point
  m_base_task.addFeature(m_t, m_s_star_t) ;   // 3D translation
  m_base_task.addFeature(m_tu,m_s_star_tu) ; // 3D rotation
  m_base_task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
  // Interaction matrix is computed with the desired visual features sd
  m_base_task.setInteractionMatrixType(vpServo::CURRENT);
  vpAdaptiveGain lambda_base_poly(0.6, 0.07, 3);//(1.2, 1.0, 10); // 2.3, 0.7, 15
  m_base_task.setLambda(lambda_base_poly);

  std::string name_transform_desbox = "cMdbox";
  if (m_mode == 0)
  {
    if (pm.parse(m_cMdbox, m_offsetFileName, name_transform_desbox) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
      std::cout << "Cannot find the homogeneous matrix named " << name_transform_desbox << "." << std::endl;
      ros::shutdown();
    }
    else
      std::cout << "Homogeneous matrix " << name_transform_desbox <<": " << std::endl << m_cMdbox << std::endl;
  }

}

vs_grasping_pepper::~vs_grasping_pepper(){

  // Disable anti external collision arm
  robot->setExternalCollisionProtectionEnabled(m_chain_name, true);
  // Disable anti external collision base
  robot->setExternalCollisionProtectionEnabled("Move", true);

  robot->getMotionProxy()->call<void>("setSmartStiffnessEnabled", true);
  robot->getMotionProxy()->call<void>("setMoveArmsEnabled", true, true);


  m_base_task.kill();

}


void  vs_grasping_pepper::initializationVS()
{

  d.init(I);
  vpDisplay::setTitle(I, "ViSP viewer");

  // Visual Servoing using the COGx and the Area of the polygon

  m_base_poly_task.setServo(vpServo::EYEINHAND_L_cVe_eJe) ;
  m_base_poly_task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE);
  //    vpAdaptiveGain lambda_adapt;
  //    lambda_adapt.initStandard(1.6, 1.8, 15);
  vpAdaptiveGain lambda_base(1.2, 1.0, 10);//(1.2, 1.0, 10); // 2.3, 0.7, 15
  m_base_poly_task.setLambda(lambda_base) ;

  //m_head_cog_des.set_uv(I.getWidth()/2, I.getHeight()/2);

  vpImagePoint ip;
  ip.set_uv(291.5,279.5);
  m_head_cog_des.set_uv(291.5,279.5);
  //m_head_cog_cur = m_head_cog_des;
  // Create the current x visual feature
  vpFeatureBuilder::create(m_s, m_cam, ip);
  vpFeatureBuilder::create(m_sd, m_cam, ip);

  // Add the feature
  m_base_poly_task.addFeature(m_s, m_sd, vpFeaturePoint::selectX()) ;

  m_coeff = 0.08652971743;
  m_Z = 0.45;
  m_Zd = m_Z;

  m_s_Z.buildFrom(m_s.get_x(), m_s.get_y(), m_Z , 0); // log(Z/Z*) = 0 that's why the last parameter is 0
  m_s_Zd.buildFrom(m_sd.get_x(), m_sd.get_y(), m_Zd , 0);

  // Add the feature
  m_base_poly_task.addFeature(m_s_Z, m_s_Zd);


}

void vs_grasping_pepper::spin()
{

  ros::Rate loop_rate(freq);

  while(!m_camInfoIsInitialized) // && !m_camIsInitialized)
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Initialize Visual servoing variables
  this->initializationVS();

  vpDisplay::display(I);

  while(ros::ok()){

    vpDisplay::display(I);

    vpMouseButton::vpMouseButtonType button;
    bool ret = vpDisplay::getClick(I, button, false);

    // Display useful informations
    if (m_cMdh_isInitialized && m_state < RaiseArm) // Box frames
    {
      vpDisplay::displayFrame(I, m_cMdh, m_cam, 0.06, vpColor::red, 2);
      if (m_state > GoToInitialPosition)
        vpDisplay::displayFrame(I, m_cMdh * m_offset, m_cam, 0.06, vpColor::cyan);
      if (m_state == ServoBase)
        vpDisplay::displayFrame(I, m_cMdbox, m_cam, 0.06, vpColor::cyan);
    }

    if (m_statusPoseHand  && m_state < RaiseArm) // Hand frame
    {
      vpDisplay::displayFrame(I, m_cMh, m_cam, 0.06, vpColor::none , 2);
      for (unsigned int i = 0; i < m_points.size(); i++)
        vpDisplay::displayPoint(I, m_points[i], vpColor::red, 4 );


    }

    if (m_state == Learning)
    {
      if (m_mode == 1) // Test mode: save a desired pose
      {
        if (ret && button == vpMouseButton::button2)
        {
          vpDisplay::displayText(I, 30, 30, "Press middle button to save the desired pose of the hand", vpColor::green);
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

      if (m_mode == 3) // learn desired pose of the box wrt the camera frame (PBVS base)
      {
        vpDisplay::displayText(I, 30, 30, "Press middle button to save the offset", vpColor::green);
        if (ret && button == vpMouseButton::button2)
          this->saveDesiredBoxPosePBVS();
      }
    }

    if (m_state == Init)
    {

      vpDisplay::displayText(I, 30, 30, "Left click to start the base Servo, Middle click for the PBVS ", vpColor::green);

      if (ret && button == vpMouseButton::button1)
      {
        m_state = WaitForServoBase;
        ret = false;
      }
      if (ret && button == vpMouseButton::button2)
      {
        goToInitialPBVSPoseBase(m_pMotion);
        vpTime::wait(2000);
        m_state = GoToInitialPosition;
        ret = false;
      }

    }

    //    if (m_state == GoToInitialPoseBase)
    //    {
    //      // Disable anti-collision arm
    //      //robot->getProxy()->setExternalCollisionProtectionEnabled("Move", false);

    //      //if(goToInitialPBVSPoseBase(robot->getProxy()))
    //      if(goToInitialPoseBase(robot->getProxy()))
    //        m_state = WaitForServoBase; // DEBUG
    //    }

    if (m_state == AnswerRequest)
    {
      vpDisplay::displayText(I, 30, 30, "Waiting for command", vpColor::green);

      qi::AnyValue data_word_recognized = m_pMemory.call<qi::AnyValue>("getData", "WordRecognized");
      qi::AnyReferenceVector result_speech = data_word_recognized.asListValuePtr();
      static bool requeste_received = false;

      if (ret && button == vpMouseButton::button1)
      {
        requeste_received = true;
        ret = false;
      }

      if ( ((result_speech[0].content().toString()) == m_vocabulary[0] && (double (result_speech[1].content().toFloat()) > 0.2 ))|| requeste_received ) // Request: bring me the box
      {
        std::cout << "Recognized: " << result_speech[0].content().toString() << "with confidence of " << result_speech[1].content().toFloat() << std::endl;
        m_pSpeechRecognition->call<void>("unsubscribe:", "request_ASR");
        m_pTextToSpeech.async<void>("say", "OK, Give me one minute. Or two.");
        m_state = ServoBase;// WaitForServoBase; // HACK for VIDEO
      }

    }

    if (m_state == WaitForServoBase)
    {

      vpDisplay::displayText(I, 30, 30, "Left click to Start the base PBVS, middl click for the TLD tracking ", vpColor::green);

      if (ret && button == vpMouseButton::button1)
      {
        //robot->getPosition(m_bodyJointNames,m_bodyJointValues,true);
        // m_bodyJointValues = m_qiProxy.call< std::vector<double> >("getAngles", m_bodyJointNames, 1);
        if(goToInitialPBVSPoseBase(m_pMotion))
          m_state = ServoBase;
        ret = false;
      }

      if (ret && button == vpMouseButton::button2)
      {
        //robot->getPosition(m_bodyJointNames,m_bodyJointValues,true);
        // m_bodyJointValues = m_qiProxy.call< std::vector<double> >("getAngles", m_bodyJointNames, 1);
        if(goToInitialPoseBase(m_pMotion))
          m_state = ServoBaseTLD;
        ret = false;
      }

    }

    if (m_state == ServoBaseTLD)
    {

      if (ret && button == vpMouseButton::button2)
      {
        m_servo_enabled = !m_servo_enabled;
        ret = false;
      }

      if(m_status_obj_polygon == 1)
        //m_obj_polygon.display(I,vpColor::blue);

        //std::cout << "Area:" << m_obj_polygon.getArea() << std::endl;
        //std::cout << "u:" << m_obj_polygon.getCenter().get_u() << std::endl;
        // std::cout << "v:" << m_obj_polygon.getCenter().get_v() << std::endl;
        // std::cout << "m_obj_polygon.getCenter():" <<   m_obj_polygon.getCenter() << std::endl;

        //  double surface = 1./sqrt(m_obj_polygon.getArea()/(m_cam.get_px()*m_cam.get_py()));
        // std::cout << "surface: " << surface << std::endl;
        //  std::cout << "dist: " << surface*m_coeff << std::endl;
        //std::cout << "coeff: " << 0.58/surface << std::endl;

        if (m_servo_enabled && !this->computeBaseTLDControlLaw())
          vpDisplay::displayText(I, 30, 30, "Servo Base TDL enabled", vpColor::green);
        else
        {
          vpDisplay::displayText(I, 30, 30, "Middle click to enable the base VS, left click to start the PBVS", vpColor::green);
          robot->stopBase();
        }


      if (ret && button == vpMouseButton::button1)
      {
        robot->stopBase();
        ret = false;
        if(goToInitialPBVSPoseBase(m_pMotion))
          m_state = ServoBase;
      }
    }

    if (m_state == ServoBase)
    {

      /*      robot->stop(m_jointNames_head);
      robot->stop(m_jointNames_arm)*/;

      //prepare the list of joints
      //      qi::AnyValue names_qi = fromStringVectorToAnyValue(m_bodyJointNames);

      //      //prepare the list of joint angles
      //      qi::AnyValue angles_qi = fromDoubleVectorToAnyValue(m_bodyJointValues);

      //      m_qiProxy.async<void>("setAngles", names_qi, angles_qi, 0.7f);

      // robot->setPosition(m_bodyJointNames,m_bodyJointValues,1.0);

      if (ret && button == vpMouseButton::button2)
      {
        m_servo_enabled = !m_servo_enabled;
        ret = false;
      }

      if (m_servo_enabled && !this->computeBasePBVSControlLaw())
      {
        vpDisplay::displayText(I, 30, 30, "Servo Base enabled", vpColor::green);
      }
      else
      {
        vpDisplay::displayText(I, 30, 30, "Middle click to enable the base VS, left click to grasp", vpColor::green);
        robot->stopBase();
      }

      if (ret && button == vpMouseButton::button1)
      {
        robot->stopBase();
        ret = false;
        m_state = GoToInitialPosition;
      }
    }

    if (m_state == GoToInitialPosition)
    {
      vpDisplay::displayText(I, 30, 30, "Click to move the robot to the initial position", vpColor::green);

      if (ret && button == vpMouseButton::button1)
      {
        if(goTointialPose(m_pMotion))
          m_state = Servoing;
        ret = false;
      }
    }

    if (m_state == Servoing)
    {

      if (ret && button == vpMouseButton::button2)
      {
        m_servo_enabled = !m_servo_enabled;
        robot->setStiffness(m_jointNames_arm, 1.0);
        ret = false;
      }

      if (m_mode == 0 || m_mode == 1 )
      {
        if (m_servo_enabled && !this->computeArmControlLaw())
        {
          vpDisplay::displayText(I, 30, 30, "Servo enabled", vpColor::green);
        }
        else
        {
          vpDisplay::displayText(I, 30, 30, "Servo disable or finished: middle click to start the VS, left click to grasp", vpColor::green);
          robot->stop(m_jointNames_arm);
        }
      }

      if (ret && button == vpMouseButton::button1)
      {
        robot->stop(m_jointNames_arm);
        m_servo_enabled = false;
        m_state = Grasp;
        ret = false;
        vpColVector q(2,0.);
        robot->setVelocity(m_jointNames_head, q);

        // robot->stopPepperControl();
      }
    }

    if (m_state == Grasp)
    {
      vpDisplay::displayText(I, 30, 30, "Closing", vpColor::green);
      robot->setStiffness("RHand", 1.0);
      robot->getMotionProxy()->call<void>("setAngles","RHand", 0.0, 0.9);

      if (ret && button == vpMouseButton::button1)
      {
        m_state = RaiseArm;
        ret = false;
      }
    }

    if (m_state == RaiseArm)
    {
      bool static first_time = true;
      vpDisplay::displayText(I, 30, 30, "Middle click to follow a person, left click to open the hand", vpColor::green);
      if (first_time)
      {
        m_pTextToSpeech.async<void>("say", "I got it!");

           std::cout << " RaiseArm" << std::endl;

        std::string  joint_name = "RShoulderPitch";
        robot->setStiffness(joint_name, 1.0);
        vpColVector p = robot->getPosition(joint_name,true);
        std::cout << "Actual angle: " <<  p[0] << std::endl;
        float angle = p[0] - vpMath::rad(10.);
        std::cout << "Actual angle: " <<  angle << std::endl;
        //robot->getProxy()->setAngles(joint_name, angle, 0.02);
        robot->setPosition(joint_name, angle, 0.02);
        std::vector<float> q;
        q.resize(2);
        q[0] = 0.0;
        q[1] = vpMath::rad(-19.0);
        //robot->getProxy()->setAngles(m_jointNames_head, q, 0.04);
        robot->setPosition(m_jointNames_head, q, 0.04);

        first_time = false;
      }

      if (ret && button == vpMouseButton::button1)
      {
        m_state = OpenHand;
        ret = false;
      }
      if (ret && button == vpMouseButton::button2)
      {
        m_state = Rotate90;
        ret = false;
      }
    }

    if (m_state == Rotate90)
    {
       std::cout << " start rotate" << std::endl;
      vpDisplay::displayText(I, 30, 30, "left click to move the head", vpColor::green);

      //robot->startPepperControl();
      //robot->getProxy()->moveTo(-0.1,0.0,0.0);
      robot->moveTo(-0.1, 0.0 , vpMath::rad(-150.0));

      m_pTextToSpeech.async<void>("Where are you?");

        std::cout << " start follow person" << std::endl;
      m_state = FollowPerson;
      robot->setStiffness(m_jointNames_head, 1.f);
      m_follow_people = new vpPepperFollowPeople(m_session, robot, m_pSpeechRecognition, m_vocabulary);
              std::cout << " set follow person" << std::endl;
      m_follow_people->setDesiredDistance(0.8);
      m_follow_people->setReverse(false);
      m_servo_enabled = true;
      //vpTime::wait(300);
    }

    if (m_state == MoveHeadToZero)
    {
      vpDisplay::displayText(I, 30, 30, "Left click to follow a person", vpColor::green);
      //      bool static first_time = true;
      //      if (first_time)
      //      {
      //        std::vector<float> q;
      //        q.resize(2);
      //        q[0] = 0.0;
      //        q[1] = vpMath::rad(-19.0);
      //        robot->getProxy()->setAngles(m_jointNames_head, q, 0.04);
      //        first_time = false;
      //      }

      if (ret && button == vpMouseButton::button1)
      {
        //robot->startPepperControl();
        m_state = FollowPerson;
          std::cout << "MoveHeadToZero" << std::endl;
        ret = false;
      }
    }

    if (m_state == FollowPerson)
    {
        std::cout << "follow person" << std::endl;
      if (ret && button == vpMouseButton::button2)
      {
        m_servo_enabled = !m_servo_enabled;
        ret = false;
      }
      if (m_servo_enabled)
      {
        m_follow_people->computeAndApplyServo();
        vpDisplay::displayText(I, 30, 30, "Servo Base enabled", vpColor::green);
      }
      else
      {
        vpDisplay::displayText(I, 30, 30, "Middle click to enable/disable following a person, left click to opend the hand", vpColor::green);
        m_follow_people->stop();
      }

      qi::AnyValue data_word_recognized = m_pMemory.call<qi::AnyValue>("getData", "WordRecognized");
      qi::AnyReferenceVector result_speech = data_word_recognized.asListValuePtr();

      if ( ((result_speech[0].content().toString()) == m_vocabulary[1]) && (double (result_speech[1].content().toFloat()) > 0.3 )) // Give the box
      {
        std::cout << "Recognized: " << result_speech[0].content().toString() << "with confidence of " << result_speech[1].content().toFloat() << std::endl;
        m_command_give_box = true;
        m_pMemory.call<void>("removeData", "WordRecognized");
      }

      if ((ret && button == vpMouseButton::button1) || m_command_give_box )
      {
        m_follow_people->stop();
        robot->stopPepperControl();
        m_state = OpenHand;
        ret = false;
        //vpTime::wait(2000);
        std::cout << "Stop Following!: " << std::endl;

      }
    }
    //    if (m_state == LowerArm)
    //    {
    //      bool static first_time = true;

    //      vpDisplay::displayText(I, 30, 30, "Raise the box", vpColor::green);
    //      if (first_time)
    //      {
    //        std::string  joint_name = "RShoulderPitch";
    //        robot->getProxy()->setStiffnesses(joint_name, 1.0f);
    //        vpColVector p = robot->getPosition(joint_name,true);
    //        AL::ALValue angle = p[0] + vpMath::rad(3.);
    //        robot->getProxy()->setAngles(joint_name, angle, 0.02);
    //        first_time = false;
    //      }

    //      if (ret && button == vpMouseButton::button1)
    //      {
    //        m_state = OpenHand;
    //        ret = false;
    //      }
    //    }

    if (m_state == OpenHand)
    {
      vpDisplay::displayText(I, 30, 30, "Click to open the arm", vpColor::green);
      if ((ret && button == vpMouseButton::button1) || m_command_give_box)
      {
        m_pTextToSpeech.async<void>("OK! take it!");

        std::cout << "Opening Arm!: " << std::endl;
        std::string  hand = "RHand";
        robot->setStiffness(hand, 1.f);
        float angle = 1.00;
        robot->setPosition(hand, angle, 0.5);
        m_state = End;
        ret = false;
      }
    }

    if (m_state == End)
    {
      vpDisplay::displayText(I, 30, 30, "Click to move back the arm", vpColor::green);

      if ((ret && button == vpMouseButton::button1)|| m_command_give_box )

      {
        std::cout << "Clicked to back the arm: " << std::endl;
        backTointialPose(m_pMotion);
        vpTime::wait(2000);
        ros::shutdown();
      }
    }

    if (ret && button == vpMouseButton::button3)
      break;

    ret = false;
    ros::spinOnce();
    vpDisplay::flush(I);

    loop_rate.sleep();
  }

  robot->stop(m_jointNames_arm);
  robot->stopBase();
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

void vs_grasping_pepper::saveDesiredBoxPosePBVS()
{
  vpXmlParserHomogeneousMatrix xml;
  std::string path = "/tmp/m_cMdbox_pepper.xml";

  if ( m_cMdh_isInitialized && m_statusPoseDesired )
  {
    ROS_INFO_STREAM("cMdh = " << m_cMdh );
    if( xml.save(m_cMdh, path.c_str(),"cMdbox") == vpXmlParserHomogeneousMatrix::SEQUENCE_OK )
      std::cout << "Pose between the camera and the object successfully saved in \"" << path << "\"" << std::endl;
    else {
      std::cout << "Failed to save the pose in \"" << path << "\"" << std::endl;
      std::cout << "A file with the same name exists. Remove it to be able to save the parameters." << std::endl;
    }
    ros::shutdown();
  }
  else
  {
    std::cout << "m_cMdh_isInitialized: " << m_cMdh_isInitialized << std::endl
              << "m_statusPoseDesired: " << m_statusPoseDesired << std::endl;
  }
}

bool vs_grasping_pepper::computeArmControlLaw()
{
  bool vs_finished = false;

  if ( m_cMh_isInitialized && m_cMdh_isInitialized  && m_statusPoseHand && m_statusPoseDesired)
  {
    static bool first_time = true;
    if (first_time) {
      std::cout << "-- Start visual servoing of the arm" << std::endl;
      m_servo_time_init = vpTime::measureTimeSecond();
      first_time = false;
      vpAdaptiveGain lambda(0.6, 0.1, 3); //(0.4, 0.07, 3);
      m_servo_arm.setLambda(lambda);
    }

    m_servo_arm.set_eJe(robot->get_eJe(m_chain_name));
    vpHomogeneousMatrix cdMc = m_offset.inverse() * m_cMdh.inverse() * m_cMh;
    m_servo_arm.setCurrentFeature(cdMc) ;
    // Create twist matrix from target Frame to Arm end-effector (WristPitch)
    vpVelocityTwistMatrix oVe_LArm(m_eMh);
    m_servo_arm.m_task.set_cVe(oVe_LArm);

    //Compute velocities PBVS task
    m_q_dot = - m_servo_arm.computeControlLaw(vpTime::measureTimeSecond() - m_servo_time_init);

    //    m_q = robot->getPosition(m_jointNames_arm);
    //    m_q2_dot  = m_servo_arm.m_task.secondaryTaskJointLimitAvoidance(m_q, m_q_dot, m_jointMin, m_jointMax);

    // publishCmdVel(m_q_dot + m_q2_dot);
    robot->setVelocity(m_jointNames_arm, m_q_dot /*+ m_q2_dot*/);

    vpTranslationVector t_error_grasp = cdMc.getTranslationVector();
    vpRotationMatrix R_error_grasp;
    cdMc.extract(R_error_grasp);
    vpThetaUVector tu_error_grasp;
    tu_error_grasp.buildFrom(R_error_grasp);
    double theta_error_grasp;
    vpColVector u_error_grasp;
    tu_error_grasp.extract(theta_error_grasp, u_error_grasp);
    std::cout << "Error t: " << sqrt(t_error_grasp.sumSquare()) << "< 0.014 " << std::endl <<
                 "Error r: " << theta_error_grasp << "< " << vpMath::rad(10) << std::endl;

    if ( (sqrt(t_error_grasp.sumSquare()) < 0.001) && (theta_error_grasp < vpMath::rad(1)) )
    {
      vs_finished = true;
      m_state = Grasp;
      robot->stop(m_jointNames_arm);
    }

  }
  else
    robot->stop(m_jointNames_arm);

  return vs_finished;
}

bool vs_grasping_pepper::computeBasePBVSControlLaw()
{
  bool vs_finished = false;

  if (m_cMdh_isInitialized && m_statusPoseDesired)
  {
    static bool first_time = true;
    if (first_time) {
      std::cout << "-- Start visual servoing of the base (PBVS)" << std::endl;
      m_servo_time_init = vpTime::measureTimeSecond();
      first_time = false;
    }

    vpHomogeneousMatrix torsoMHeadPith( robot->getTransform("HeadPitch", 0));// get transformation  matrix between torso and HeadRoll
    vpVelocityTwistMatrix HeadPitchVLtorso(torsoMHeadPith.inverse());

    for(unsigned int i=0; i< 3; i++)
      for(unsigned int j=0; j< 3; j++)
        HeadPitchVLtorso[i][j+3] = 0;

    m_eJe = HeadPitchVLtorso * m_tJe;

    m_base_task.set_eJe(m_eJe);
    m_base_task.set_cVe( vpVelocityTwistMatrix(m_eMc.inverse()) );

    vpHomogeneousMatrix cdMc = m_cMdbox * m_cMdh.inverse();
    m_t.buildFrom(cdMc);
    m_tu.buildFrom(cdMc);

    //m_base_task.print();
    //    std::cout << "    m_base_task.getInteractionMatrix()" << m_base_task.getInteractionMatrix() << std::endl;
    //    std::cout << "m_base_task.getError()" << m_base_task.getError() << std::endl;
    //    std::cout << "m_base_task.getTaskJacobian()" << m_base_task.getTaskJacobian() << std::endl;

    //Compute velocities PBVS task
    m_base_vel = m_base_task.computeControlLaw(vpTime::measureTimeSecond() - m_servo_time_init);


    //    if (m_base_vel.euclideanNorm() > 0.01)
    //    {
    for (unsigned int i = 0; i < m_base_vel.size();  i++)
    {
      if(vpMath::abs(m_base_vel[i]) < 0.011)
        m_base_vel[i] = 0.0;
    }
    std::cout << "VEL:" << m_base_vel << std::endl;

    m_pMotion.async<void>("move", m_base_vel[0], m_base_vel[1], m_base_vel[2]);
    //      robot->setBaseVelocity(m_base_vel[0], m_base_vel[1], m_base_vel[2]);
    //    }
    //    else
    //    std::cout << "VEL too low:" << m_base_vel.euclideanNorm() << std::endl;
    //    vpTranslationVector t_error_grasp = cdMc.getTranslationVector();
    //    vpRotationMatrix R_error_grasp;
    //    cdMc.extract(R_error_grasp);
    //    vpThetaUVector tu_error_grasp;
    //    tu_error_grasp.buildFrom(R_error_grasp);
    //    double theta_error_grasp;
    //    vpColVector u_error_grasp;
    //    tu_error_grasp.extract(theta_error_grasp, u_error_grasp);
    //    std::cout << "Error t: " << sqrt(t_error_grasp.sumSquare()) << "< 0.024 " << std::endl <<
    //                 "Error r: " << theta_error_grasp << "< " << vpMath::rad(5) << std::endl;

    //    if ( (sqrt(t_error_grasp.sumSquare()) < 0.014) && (theta_error_grasp < vpMath::rad(10)) )
    //    {
    //      vs_finished = true;
    //      m_state = Grasp;
    //      robot->stop(m_jointNames_arm);
    //    }

  }
  else
    robot->stopBase();

  return vs_finished;

}

bool vs_grasping_pepper::computeBaseTLDControlLaw()
{
  bool vs_finished = false;

  if ( m_status_obj_polygon == 1 )
  {
    static bool first_time = true;
    if (first_time) {
      std::cout << "-- Start visual servoing of the base" << std::endl;
      m_servo_time_init = vpTime::measureTimeSecond();
      first_time = false;
    }

    //    vpPoint P;
    //    P.setWorldCoordinates(0.05/2, 0.05/2, -0.15/2);
    //    P.project(m_cMdh);
    //    double u=0, v=0;
    //    vpMeterPixelConversion::convertPoint(m_cam, P.get_x(), P.get_y(), u, v);
    //    if (u<= I.getWidth() && v <= I.getHeight())
    // m_head_cog_cur.set_uv(m_obj_polygon.getCenter().get_u(),m_obj_polygon.getCenter().get_v());

    m_head_cog_cur = m_obj_polygon.getCenter();

    vpDisplay::displayCross(I, m_head_cog_des, 10, vpColor::red);
    vpDisplay::displayCross(I, m_head_cog_cur, 10, vpColor::green);

    //std::cout << "m_head_cog_des" << m_head_cog_des << std::endl;
    //std::cout << "m_head_cog_cur" << m_head_cog_cur << std::endl;

    vpHomogeneousMatrix torsoMHeadPith( robot->getTransform("HeadPitch", 0));// get transformation  matrix between torso and HeadRoll
    vpVelocityTwistMatrix HeadPitchVLtorso(torsoMHeadPith.inverse());

    for(unsigned int i=0; i< 3; i++)
      for(unsigned int j=0; j< 3; j++)
        HeadPitchVLtorso[i][j+3] = 0;

    m_eJe = HeadPitchVLtorso * m_tJe;

    m_base_poly_task.set_eJe( m_eJe );
    m_base_poly_task.set_cVe( vpVelocityTwistMatrix(m_eMc.inverse()) );

    // Compute distanze box camera
    double surface = 1./sqrt(m_obj_polygon.getArea()/(m_cam.get_px()*m_cam.get_py()));
    m_Z = m_coeff * surface;

    std::cout << "Distance" << m_Z << std::endl;

    // Update the current x feature
    // double x,y;
    // vpPixelMeterConversion::convertPoint(m_cam, m_head_cog_cur, x, y);
    // m_s.buildFrom(y, x, 0.58);
    vpFeatureBuilder::create(m_s, m_cam, m_head_cog_cur);

    // Update log(Z/Z*) feature. Since the depth Z change, we need to update the intection matrix
    m_s_Z.buildFrom(m_s.get_x(), m_s.get_y(), m_Z, log(m_Z/m_Zd)) ;

    m_base_vel = m_base_poly_task.computeControlLaw(vpTime::measureTimeSecond() - m_servo_time_init);

    //std::cout << "  ERROR:  " <<   m_base_poly_task.getError() << std::endl;
    //std::cout << "  m_base_vel:  " <<   m_base_vel << std::endl;

    m_pMotion.async<void>("move", m_base_vel[0], m_base_vel[1], m_base_vel[2]);
  }
  else
    robot->stopBase();

  return vs_finished;
}


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

void vs_grasping_pepper::getStatusObjectPolygonCb(const std_msgs::Int8::ConstPtr  &status)
{
  m_status_obj_polygon = status->data;
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

void vs_grasping_pepper::getObjectPolygonCb(const geometry_msgs::Polygon::ConstPtr &msg)
{
  double x0 = msg->points[0].y;
  double y0 = msg->points[0].x;
  double w = msg->points[1].y;
  double h = msg->points[1].x;

  std::vector<vpImagePoint> corners;
  // Initialize the corners vector with 4 points
  corners.push_back( vpImagePoint(x0, y0) );
  corners.push_back( vpImagePoint(x0 , y0 + h) );
  corners.push_back( vpImagePoint(x0 + w, y0 + h) );
  corners.push_back( vpImagePoint(x0 + w, y0 ) );
  m_obj_polygon.buildFrom(corners);
}


void vs_grasping_pepper::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  boost::mutex::scoped_lock(m_lock);
  try
  {
    I = visp_bridge::toVispImageRGBa(*msg); //make sure the image isn't worked on by locking a mutex
  }
  catch(...)
  {
    ROS_ERROR("Could not convert from '%s' to Visp 'RGB'.", msg->encoding.c_str());
  }
  m_camIsInitialized = 1;
}

void vs_grasping_pepper::getPointArrayCb(const whycon::PointArrayConstPtr &msg)
{
  m_points.clear();
  for (unsigned int i = 0; i < msg->points.size(); i++)
  {
    vpImagePoint p(msg->points[i].y, msg->points[i].x );

    m_points.push_back(p);
  }
}

qi::AnyValue vs_grasping_pepper::fromStringVectorToAnyValue(const std::vector<std::string> &vector)
{
  qi::AnyValue res;
  try
  {
    std::vector<qi::AnyValue> vector_qi;
    vector_qi.reserve(vector.size());
    vector_qi.resize(vector.size());

    std::vector<std::string>::const_iterator it = vector.begin();
    std::vector<qi::AnyValue>::iterator it_qi = vector_qi.begin();
    for(; it != vector.end(); ++it, ++it_qi)
    {
      *it_qi = qi::AnyValue(qi::AnyReference::from(*it), false, false);
    }
    res = qi::AnyValue(qi::AnyReference::from(vector_qi), false, false);
  }
  catch(const std::exception& e)
  {
    std::cout << "Could not convert to qi::AnyValue \n\tTrace: " << e.what() << std::endl;
  }
  return res;
}

qi::AnyValue vs_grasping_pepper::fromDoubleVectorToAnyValue(const std::vector<double> &vector)
{
  qi::AnyValue res;
  try
  {
    std::vector<qi::AnyValue> vector_qi;
    vector_qi.reserve(vector.size());
    vector_qi.resize(vector.size());

    std::vector<double>::const_iterator it = vector.begin();
    std::vector<qi::AnyValue>::iterator it_qi = vector_qi.begin();
    for(; it != vector.end(); ++it, ++it_qi)
    {
      *it_qi = qi::AnyValue(qi::AnyReference::from(static_cast<float>(*it)), false, false);
    }
    res = qi::AnyValue(qi::AnyReference::from(vector_qi), false, false);
  }
  catch(const std::exception& e)
  {
    std::cout << "Could not convert to qi::AnyValue \n\tTrace: " << e.what() << std::endl;
  }
  return res;
}

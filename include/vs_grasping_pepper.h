#include <visp_naoqi/vpNaoqiGrabber.h>
#include <visp3/gui/vpDisplayX.h>
#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>

#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Polygon.h>
#include <std_msgs/Int8.h>
#include <whycon/PointArray.h>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpQuaternionVector.h>
#include <visp/vpXmlParserCamera.h>
#include <visp/vpXmlParserHomogeneousMatrix.h>

#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/visual_features/vpFeatureDepth.h>
#include <visp/vpFeatureTranslation.h>

#include <visp_bridge/3dpose.h>
#include <visp_bridge/image.h>
#include <visp_bridge/camera.h>

#include <visp_naoqi/vpNaoqiRobot.h>
//#include <visp_naoqi/vpNaoqiConfig.h>

#include <visp_naoqi/common/vpServoArm.h>
#include <visp_naoqi/common/vpPepperFollowPeople.h>

class vs_grasping_pepper
{
public:

  vs_grasping_pepper(ros::NodeHandle &nh);
  ~vs_grasping_pepper();
  void initializationVS();
  bool computeArmControlLaw();
  bool computeBaseControlLaw();
  bool computeBasePBVSControlLaw();
  bool computeBaseTLDControlLaw();
  //bool computeFollowPersonControlLaw();
  void saveOffset();
  void saveDesiredBoxPosePBVS();
  void spin();
  void getActualPoseCb(const geometry_msgs::PoseStampedConstPtr &msg);
  void getCameraInfoCb(const sensor_msgs::CameraInfoConstPtr &msg);
  void getDesiredPoseCb(const geometry_msgs::TransformStampedConstPtr &msg);
  void getObjectPolygonCb(const geometry_msgs::Polygon::ConstPtr &msg);
  void getStatusPoseHandCb(const std_msgs::Int8::ConstPtr &status);
  void getStatusPoseDesiredCb(const std_msgs::Int8::ConstPtr &status);
  void getStatusObjectPolygonCb(const std_msgs::Int8::ConstPtr  &status);
  void getPointArrayCb(const whycon::PointArrayConstPtr &msg);
  void getLaserCb(const sensor_msgs::LaserScanConstPtr &msg);
  void imageCb(const sensor_msgs::ImageConstPtr& msg);
  void getRobotJoints();
  void publishCmdVel(const vpColVector &q);
  std::vector<double> obs_avoidance_control(double vx, double vy, double wz);


  qi::AnyValue fromStringVectorToAnyValue(const std::vector<std::string> &vector);
  qi::AnyValue fromDoubleVectorToAnyValue(const std::vector<double> &vector);

  typedef enum {
    Learning,
    Init,
    GoToInitialPoseBase,
    WaitForServoBase,
    AnswerRequest,
    ServoBaseTLD,
    ServoBase,
    GoToInitialPosition,
    Servoing,
    Grasp,
    RaiseArm,
    LowerArm,
    Rotate90,
    MoveHeadToZero,
    FollowPerson,
    OpenHand,
    End,
    WaitEnd
  } State_t;

protected:

  // Robot
  vpNaoqiRobot * robot;
  qi::SessionPtr m_session;
  std::vector<std::string> m_jointNames_arm;
  std::vector<std::string> m_bodyJointNames;
  std::vector<double> m_bodyJointValues;

  int m_numJoints;
  std::string m_chain_name;
  vpColVector m_jointMin;
  vpColVector m_jointMax;
  std::string m_eMh_pepper_path;
  std::string m_ip;

  //Display
  vpImage<vpRGBa> I;
  vpDisplayX d;
  boost::mutex m_lock;
  std::vector< vpImagePoint> m_points;

  // ROS
  ros::NodeHandle n;
  std::string cmdVelTopicName;
  std::string actualPoseTopicName;
  std::string desiredPoseTopicName;
  std::string statusPoseHandTopicName;
  std::string statusPoseDesiredTopicName;
  std::string m_statusObjectPolygonTopicName;
  std::string m_objectPolygonTopicName;
  std::string m_opt_arm;
  std::string m_cameraInfoName;
  std::string m_cameraTopicName;
  std::string m_pointArrayName;
  std::string m_laserTopicName;
  ros::Subscriber actualPoseSub;
  ros::Subscriber desiredPoseSub;
  ros::Subscriber statusPoseHandSub;
  ros::Subscriber statusPoseDesiredSub;
  ros::Subscriber m_cameraInfoSub;
  ros::Subscriber m_ObjectPolygonSub;
  ros::Subscriber m_statusObjectPolygonSub;
  ros::Subscriber m_pointArraySub;
  ros::Subscriber m_laserSub;
  image_transport::ImageTransport m_it;
  image_transport::Subscriber m_itSub;

  ros::Publisher cmdVelPub;
  int freq;
  int m_mode;
  State_t m_state;

  // Messages
  sensor_msgs::JointState m_q_dot_msg;

  //Servo Arm
  vpServoArm m_servo_arm;
  vpColVector m_q;
  vpColVector m_q_dot;
  vpColVector m_q2_dot;
  vpCameraParameters m_cam;

  std::string m_offsetFileName;

  double m_servo_time_init;
  int m_statusPoseHand;
  int m_statusPoseDesired;

  vpHomogeneousMatrix m_cMh;
  vpHomogeneousMatrix m_cMdh;
  vpHomogeneousMatrix m_eMh;
  vpHomogeneousMatrix m_offset;

  // Servo Base PBVS
  bool m_pbvs_base;
  vpServo m_base_task;
  vpFeatureTranslation m_t;
  vpFeatureThetaU m_tu;
  vpFeatureTranslation m_s_star_t;
  vpFeatureThetaU m_s_star_tu;
  std::vector<std::string> m_jointNames_head;
  vpColVector m_q_head;
  vpHomogeneousMatrix m_cMdbox;

  // Servo Follow people
  vpPepperFollowPeople * m_follow_people;

  // Obstacle avoidance using laser
  std::vector<float> m_laser_data;
  bool m_laserInit;
  float m_angleMin;
  float m_angleMax;
  float m_angleIncrement;
  float m_min_dist;
  vpMatrix m_L;
  std::vector <vpMatrix> m_A;


  std::vector< vpHomogeneousMatrix> m_Hlaser;
  std::vector< vpMatrix> m_Vlaser;

  // Servo Base to track an object
  vpPolygon m_obj_polygon;
  int m_status_obj_polygon;
  vpServo m_base_poly_task;

  vpFeaturePoint m_s;
  vpFeaturePoint m_sd;
  vpFeatureDepth m_s_Z;
  vpFeatureDepth m_s_Zd;
  vpMatrix m_tJe;
  vpMatrix m_eJe;
  vpImagePoint m_head_cog_des;
  vpImagePoint m_head_cog_cur;
  vpHomogeneousMatrix m_eMc;
  double m_Z;
  double m_Zd;
  vpColVector m_base_vel;
  double m_coeff;


  // New Proxy
  //qi::AnyObject m_qiProxy;
  qi::AnyObject m_pMotion; //!< Motion proxy
  qi::AnyObject m_pMemory; //!< Memory proxy
  qi::AnyObject m_pTextToSpeech; //!< TextToSpeech proxy
  qi::AnyObject * m_pSpeechRecognition; //!< ALSpeechRecognition proxy

  //AL::ALMemoryProxy * m_mem_proxy;
  //AL::ALSpeechRecognitionProxy * m_asr_proxy;
  std::vector<std::string> m_vocabulary;
  //AL::ALTextToSpeechProxy * m_tts_proxy;


  //conditions
  bool m_cMh_isInitialized;
  bool m_cMdh_isInitialized;
  bool m_statusPoseDesired_isEnable;
  bool m_servo_enabled;
  bool m_camInfoIsInitialized;
  bool m_command_give_box;
  bool m_camIsInitialized;

};

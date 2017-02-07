#include <visp_naoqi/vpNaoqiGrabber.h>
#include <visp3/gui/vpDisplayX.h>
#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Int8.h>

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
#include <visp_naoqi/vpNaoqiConfig.h>

#include <vpServoArm.h>
#include <vpPepperFollowPeople.h>

class vs_grasping_pepper
{
public:

  vs_grasping_pepper(ros::NodeHandle &nh);
  ~vs_grasping_pepper();
  bool computeArmControlLaw();
  bool computeBaseControlLaw();
  bool computeBasePBVSControlLaw();
  //bool computeFollowPersonControlLaw();
  void saveOffset();
  void saveDesiredBoxPosePBVS();
  void spin();
  void getActualPoseCb(const geometry_msgs::PoseStampedConstPtr &msg);
  void getDesiredPoseCb(const geometry_msgs::TransformStampedConstPtr &msg);
  void getStatusPoseHandCb(const std_msgs::Int8::ConstPtr &status);
  void getStatusPoseDesiredCb(const std_msgs::Int8::ConstPtr &status);
  void getCameraInfoCb(const sensor_msgs::CameraInfoConstPtr &msg);
  void getRobotJoints();
  void publishCmdVel(const vpColVector &q);

  qi::AnyValue fromStringVectorToAnyValue(const std::vector<std::string> &vector);
  qi::AnyValue fromDoubleVectorToAnyValue(const std::vector<double> &vector);

  typedef enum {
    Learning,
    Init,
    GoToInitialPoseBase,
    WaitForServoBase,
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
    End
  } State_t;

protected:

  // Robot
  vpNaoqiRobot robot;
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
  vpImage<unsigned char> I;
  vpDisplayX d;

  // ROS
  ros::NodeHandle n;
  std::string actualPoseTopicName;
  std::string desiredPoseTopicName;
  std::string statusPoseHandTopicName;
  std::string statusPoseDesiredTopicName;
  std::string cmdVelTopicName;
  std::string m_opt_arm;
  std::string m_cameraInfoName;
  ros::Subscriber actualPoseSub;
  ros::Subscriber desiredPoseSub;
  ros::Subscriber statusPoseHandSub;
  ros::Subscriber statusPoseDesiredSub;
  ros::Subscriber m_cameraInfoSub;
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

  // Servo Base IBVS
  bool m_pbvs_base;
  vpServo m_base_fp_task;
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

  // Servo Base PBVS
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

  // New Proxy
  qi::SessionPtr m_session; //!< Session to connect to Pepper
  qi::AnyObject m_qiProxy;
  // Proxy
  AL::ALMemoryProxy * m_mem_proxy;
  AL::ALSpeechRecognitionProxy * m_asr_proxy;
  std::vector<std::string> m_vocabulary;


  //conditions
  bool m_cMh_isInitialized;
  bool m_cMdh_isInitialized;
  bool m_statusPoseDesired_isEnable;
  bool m_servo_enabled;
  bool m_camInfoIsInitialized;
  bool m_command_give_box;

};

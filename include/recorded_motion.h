#include <iostream>
#include <vector>

//#include <alproxies/almotionproxy.h>

#include <qi/anyobject.hpp>

//bool goTointialPose(qi::AnyObject *motion)
//{
//  // Choregraphe bezier export in c++.
//  // Add #include <alproxies/almotionproxy.h> at the beginning of this file.
//  std::vector<std::string> names;
//  qi::AnyValueVector times, keys;
//  names.reserve(17);
//  times.resize(17);
//  keys.resize(17);

//  names.push_back("HeadPitch");
//  times[0].resize(2);
//  keys[0].resize(2);

//  times[0][0] = 0.08;
//  keys[0][0] = AL::ALValue::array(-0.265379, AL::ALValue::array(3, -0.0266667, 0), AL::ALValue::array(3, 2.30667, 0));
//  times[0][1] = 7;
//  keys[0][1] = AL::ALValue::array(-0.162602, AL::ALValue::array(3, -2.30667, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("HeadYaw");
//  times[1].arraySetSize(2);
//  keys[1].arraySetSize(2);

//  times[1][0] = 0.08;
//  keys[1][0] = AL::ALValue::array(-0.250039, AL::ALValue::array(3, -0.0266667, 0), AL::ALValue::array(3, 2.30667, 0));
//  times[1][1] = 7;
//  keys[1][1] = AL::ALValue::array(-0.564504, AL::ALValue::array(3, -2.30667, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("HipPitch");
//  times[2].arraySetSize(5);
//  keys[2].arraySetSize(5);

//  times[2][0] = 0.08;
//  keys[2][0] = AL::ALValue::array(0, AL::ALValue::array(3, -0.0266667, 0), AL::ALValue::array(3, 0.826667, 0));
//  times[2][1] = 2.56;
//  keys[2][1] = AL::ALValue::array(0, AL::ALValue::array(3, -0.826667, 0), AL::ALValue::array(3, 0.72, 0));
//  times[2][2] = 4.72;
//  keys[2][2] = AL::ALValue::array(0, AL::ALValue::array(3, -0.72, 0), AL::ALValue::array(3, 0.24, 0));
//  times[2][3] = 5.44;
//  keys[2][3] = AL::ALValue::array(0, AL::ALValue::array(3, -0.24, 0), AL::ALValue::array(3, 0.32, 0));
//  times[2][4] = 6.4;
//  keys[2][4] = AL::ALValue::array(0, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("HipRoll");
//  times[3].arraySetSize(5);
//  keys[3].arraySetSize(5);

//  times[3][0] = 0.08;
//  keys[3][0] = AL::ALValue::array(0, AL::ALValue::array(3, -0.0266667, 0), AL::ALValue::array(3, 0.826667, 0));
//  times[3][1] = 2.56;
//  keys[3][1] = AL::ALValue::array(-0.00153399, AL::ALValue::array(3, -0.826667, 0), AL::ALValue::array(3, 0.72, 0));
//  times[3][2] = 4.72;
//  keys[3][2] = AL::ALValue::array(0, AL::ALValue::array(3, -0.72, 0), AL::ALValue::array(3, 0.24, 0));
//  times[3][3] = 5.44;
//  keys[3][3] = AL::ALValue::array(0, AL::ALValue::array(3, -0.24, 0), AL::ALValue::array(3, 0.32, 0));
//  times[3][4] = 6.4;
//  keys[3][4] = AL::ALValue::array(0, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("KneePitch");
//  times[4].arraySetSize(5);
//  keys[4].arraySetSize(5);

//  times[4][0] = 0.08;
//  keys[4][0] = AL::ALValue::array(0, AL::ALValue::array(3, -0.0266667, 0), AL::ALValue::array(3, 0.826667, 0));
//  times[4][1] = 2.56;
//  keys[4][1] = AL::ALValue::array(0, AL::ALValue::array(3, -0.826667, 0), AL::ALValue::array(3, 0.72, 0));
//  times[4][2] = 4.72;
//  keys[4][2] = AL::ALValue::array(0, AL::ALValue::array(3, -0.72, 0), AL::ALValue::array(3, 0.24, 0));
//  times[4][3] = 5.44;
//  keys[4][3] = AL::ALValue::array(0, AL::ALValue::array(3, -0.24, 0), AL::ALValue::array(3, 0.32, 0));
//  times[4][4] = 6.4;
//  keys[4][4] = AL::ALValue::array(0, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("LElbowRoll");
//  times[5].arraySetSize(5);
//  keys[5].arraySetSize(5);

//  times[5][0] = 0.08;
//  keys[5][0] = AL::ALValue::array(-0.524622, AL::ALValue::array(3, -0.0266667, 0), AL::ALValue::array(3, 0.826667, 0));
//  times[5][1] = 2.56;
//  keys[5][1] = AL::ALValue::array(-0.524622, AL::ALValue::array(3, -0.826667, 0), AL::ALValue::array(3, 0.72, 0));
//  times[5][2] = 4.72;
//  keys[5][2] = AL::ALValue::array(-0.524622, AL::ALValue::array(3, -0.72, 0), AL::ALValue::array(3, 0.24, 0));
//  times[5][3] = 5.44;
//  keys[5][3] = AL::ALValue::array(-0.524622, AL::ALValue::array(3, -0.24, 0), AL::ALValue::array(3, 0.32, 0));
//  times[5][4] = 6.4;
//  keys[5][4] = AL::ALValue::array(-0.524622, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("LElbowYaw");
//  times[6].arraySetSize(5);
//  keys[6].arraySetSize(5);

//  times[6][0] = 0.08;
//  keys[6][0] = AL::ALValue::array(-1.22872, AL::ALValue::array(3, -0.0266667, 0), AL::ALValue::array(3, 0.826667, 0));
//  times[6][1] = 2.56;
//  keys[6][1] = AL::ALValue::array(-1.22872, AL::ALValue::array(3, -0.826667, 0), AL::ALValue::array(3, 0.72, 0));
//  times[6][2] = 4.72;
//  keys[6][2] = AL::ALValue::array(-1.22872, AL::ALValue::array(3, -0.72, 0), AL::ALValue::array(3, 0.24, 0));
//  times[6][3] = 5.44;
//  keys[6][3] = AL::ALValue::array(-1.22872, AL::ALValue::array(3, -0.24, 0), AL::ALValue::array(3, 0.32, 0));
//  times[6][4] = 6.4;
//  keys[6][4] = AL::ALValue::array(-1.22872, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("LHand");
//  times[7].arraySetSize(5);
//  keys[7].arraySetSize(5);

//  times[7][0] = 0.08;
//  keys[7][0] = AL::ALValue::array(0.598418, AL::ALValue::array(3, -0.0266667, 0), AL::ALValue::array(3, 0.826667, 0));
//  times[7][1] = 2.56;
//  keys[7][1] = AL::ALValue::array(0.598418, AL::ALValue::array(3, -0.826667, 0), AL::ALValue::array(3, 0.72, 0));
//  times[7][2] = 4.72;
//  keys[7][2] = AL::ALValue::array(0.598418, AL::ALValue::array(3, -0.72, 0), AL::ALValue::array(3, 0.24, 0));
//  times[7][3] = 5.44;
//  keys[7][3] = AL::ALValue::array(0.598418, AL::ALValue::array(3, -0.24, 0), AL::ALValue::array(3, 0.32, 0));
//  times[7][4] = 6.4;
//  keys[7][4] = AL::ALValue::array(0.598418, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("LShoulderPitch");
//  times[8].arraySetSize(5);
//  keys[8].arraySetSize(5);

//  times[8][0] = 0.08;
//  keys[8][0] = AL::ALValue::array(1.56619, AL::ALValue::array(3, -0.0266667, 0), AL::ALValue::array(3, 0.826667, 0));
//  times[8][1] = 2.56;
//  keys[8][1] = AL::ALValue::array(1.56619, AL::ALValue::array(3, -0.826667, 0), AL::ALValue::array(3, 0.72, 0));
//  times[8][2] = 4.72;
//  keys[8][2] = AL::ALValue::array(1.56619, AL::ALValue::array(3, -0.72, 0), AL::ALValue::array(3, 0.24, 0));
//  times[8][3] = 5.44;
//  keys[8][3] = AL::ALValue::array(1.56619, AL::ALValue::array(3, -0.24, 0), AL::ALValue::array(3, 0.32, 0));
//  times[8][4] = 6.4;
//  keys[8][4] = AL::ALValue::array(1.56619, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("LShoulderRoll");
//  times[9].arraySetSize(5);
//  keys[9].arraySetSize(5);

//  times[9][0] = 0.08;
//  keys[9][0] = AL::ALValue::array(0.138058, AL::ALValue::array(3, -0.0266667, 0), AL::ALValue::array(3, 0.826667, 0));
//  times[9][1] = 2.56;
//  keys[9][1] = AL::ALValue::array(0.138058, AL::ALValue::array(3, -0.826667, 0), AL::ALValue::array(3, 0.72, 0));
//  times[9][2] = 4.72;
//  keys[9][2] = AL::ALValue::array(0.138058, AL::ALValue::array(3, -0.72, 0), AL::ALValue::array(3, 0.24, 0));
//  times[9][3] = 5.44;
//  keys[9][3] = AL::ALValue::array(0.138058, AL::ALValue::array(3, -0.24, 0), AL::ALValue::array(3, 0.32, 0));
//  times[9][4] = 6.4;
//  keys[9][4] = AL::ALValue::array(0.138058, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("LWristYaw");
//  times[10].arraySetSize(5);
//  keys[10].arraySetSize(5);

//  times[10][0] = 0.08;
//  keys[10][0] = AL::ALValue::array(0.0352399, AL::ALValue::array(3, -0.0266667, 0), AL::ALValue::array(3, 0.826667, 0));
//  times[10][1] = 2.56;
//  keys[10][1] = AL::ALValue::array(0.024502, AL::ALValue::array(3, -0.826667, 0), AL::ALValue::array(3, 0.72, 0));
//  times[10][2] = 4.72;
//  keys[10][2] = AL::ALValue::array(0.024502, AL::ALValue::array(3, -0.72, 0), AL::ALValue::array(3, 0.24, 0));
//  times[10][3] = 5.44;
//  keys[10][3] = AL::ALValue::array(0.024502, AL::ALValue::array(3, -0.24, 0), AL::ALValue::array(3, 0.32, 0));
//  times[10][4] = 6.4;
//  keys[10][4] = AL::ALValue::array(0.024502, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("RElbowRoll");
//  times[11].arraySetSize(5);
//  keys[11].arraySetSize(5);

//  times[11][0] = 0.08;
//  keys[11][0] = AL::ALValue::array(0.139592, AL::ALValue::array(3, -0.0266667, 0), AL::ALValue::array(3, 0.826667, 0));
//  times[11][1] = 2.56;
//  keys[11][1] = AL::ALValue::array(0.51235, AL::ALValue::array(3, -0.826667, 0), AL::ALValue::array(3, 0.72, 0));
//  times[11][2] = 4.72;
//  keys[11][2] = AL::ALValue::array(0.147262, AL::ALValue::array(3, -0.72, 0), AL::ALValue::array(3, 0.24, 0));
//  times[11][3] = 5.44;
//  keys[11][3] = AL::ALValue::array(0.866699, AL::ALValue::array(3, -0.24, -0.0310625), AL::ALValue::array(3, 0.32, 0.0414167));
//  times[11][4] = 6.4;
//  keys[11][4] = AL::ALValue::array(0.908116, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("RElbowYaw");
//  times[12].arraySetSize(5);
//  keys[12].arraySetSize(5);

//  times[12][0] = 0.08;
//  keys[12][0] = AL::ALValue::array(0.383496, AL::ALValue::array(3, -0.0266667, 0), AL::ALValue::array(3, 0.826667, 0));
//  times[12][1] = 2.56;
//  keys[12][1] = AL::ALValue::array(1.22718, AL::ALValue::array(3, -0.826667, -0.2405), AL::ALValue::array(3, 0.72, 0.209468));
//  times[12][2] = 4.72;
//  keys[12][2] = AL::ALValue::array(1.7334, AL::ALValue::array(3, -0.72, 0), AL::ALValue::array(3, 0.24, 0));
//  times[12][3] = 5.44;
//  keys[12][3] = AL::ALValue::array(1.61375, AL::ALValue::array(3, -0.24, 0.119651), AL::ALValue::array(3, 0.32, -0.159535));
//  times[12][4] = 6.4;
//  keys[12][4] = AL::ALValue::array(0.319067, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("RHand");
//  times[13].arraySetSize(5);
//  keys[13].arraySetSize(5);

//  times[13][0] = 0.08;
//  keys[13][0] = AL::ALValue::array(0.622144, AL::ALValue::array(3, -0.0266667, 0), AL::ALValue::array(3, 0.826667, 0));
//  times[13][1] = 2.56;
//  keys[13][1] = AL::ALValue::array(0.587873, AL::ALValue::array(3, -0.826667, 0.00939336), AL::ALValue::array(3, 0.72, -0.00818131));
//  times[13][2] = 4.72;
//  keys[13][2] = AL::ALValue::array(0.56942, AL::ALValue::array(3, -0.72, 0), AL::ALValue::array(3, 0.24, 0));
//  times[13][3] = 5.44;
//  keys[13][3] = AL::ALValue::array(0.884886, AL::ALValue::array(3, -0.24, 0), AL::ALValue::array(3, 0.32, 0));
//  times[13][4] = 6.4;
//  keys[13][4] = AL::ALValue::array(0.884886, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("RShoulderPitch");
//  times[14].arraySetSize(5);
//  keys[14].arraySetSize(5);

//  times[14][0] = 0.08;
//  keys[14][0] = AL::ALValue::array(1.44041, AL::ALValue::array(3, -0.0266667, 0), AL::ALValue::array(3, 0.826667, 0));
//  times[14][1] = 2.56;
//  keys[14][1] = AL::ALValue::array(1.55852, AL::ALValue::array(3, -0.826667, 0), AL::ALValue::array(3, 0.72, 0));
//  times[14][2] = 4.72;
//  keys[14][2] = AL::ALValue::array(1.51557, AL::ALValue::array(3, -0.72, 0.0429508), AL::ALValue::array(3, 0.24, -0.0143169));
//  times[14][3] = 5.44;
//  keys[14][3] = AL::ALValue::array(0.237766, AL::ALValue::array(3, -0.24, 0.121951), AL::ALValue::array(3, 0.32, -0.162601));
//  times[14][4] = 6.4;
//  keys[14][4] = AL::ALValue::array(0.075165, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("RShoulderRoll");
//  times[15].arraySetSize(5);
//  keys[15].arraySetSize(5);

//  times[15][0] = 0.08;
//  keys[15][0] = AL::ALValue::array(-0.0905049, AL::ALValue::array(3, -0.0266667, 0), AL::ALValue::array(3, 0.826667, 0));
//  times[15][1] = 2.56;
//  keys[15][1] = AL::ALValue::array(-0.162602, AL::ALValue::array(3, -0.826667, 0.0720971), AL::ALValue::array(3, 0.72, -0.0627943));
//  times[15][2] = 4.72;
//  keys[15][2] = AL::ALValue::array(-1.12134, AL::ALValue::array(3, -0.72, 0), AL::ALValue::array(3, 0.24, 0));
//  times[15][3] = 5.44;
//  keys[15][3] = AL::ALValue::array(-1.1106, AL::ALValue::array(3, -0.24, -0.010739), AL::ALValue::array(3, 0.32, 0.0143187));
//  times[15][4] = 6.4;
//  keys[15][4] = AL::ALValue::array(-0.785398, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("RWristYaw");
//  times[16].arraySetSize(5);
//  keys[16].arraySetSize(5);

//  times[16][0] = 0.08;
//  keys[16][0] = AL::ALValue::array(0.961776, AL::ALValue::array(3, -0.0266667, 0), AL::ALValue::array(3, 0.826667, 0));
//  times[16][1] = 2.56;
//  keys[16][1] = AL::ALValue::array(-0.0245859, AL::ALValue::array(3, -0.826667, 0), AL::ALValue::array(3, 0.72, 0));
//  times[16][2] = 4.72;
//  keys[16][2] = AL::ALValue::array(0.400331, AL::ALValue::array(3, -0.72, -0.0368194), AL::ALValue::array(3, 0.24, 0.0122731));
//  times[16][3] = 5.44;
//  keys[16][3] = AL::ALValue::array(0.412605, AL::ALValue::array(3, -0.24, -0.0122731), AL::ALValue::array(3, 0.32, 0.0163642));
//  times[16][4] = 6.4;
//  keys[16][4] = AL::ALValue::array(1.29465, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

//  try
//  {
//    //motion->post.angleInterpolationBezier(names, times, keys);
//    motion.async<void>("angleInterpolationBezier", names, times, keys);
//  }
//  catch(const std::exception&)
//  {
//    std::cout << "Error during the motion" << std::endl;
//    return false;
//  }

//  return true;

//}



bool goTointialPose(qi::AnyObject &motion)
{
  // Choregraphe bezier export in c++.
  std::vector<std::string> names;
  std::vector<std::vector<float>> times;
  std::vector<std::vector<float>> keys;



  names.push_back("HeadPitch");
  std::vector<float> t1 = {0.08, 7};
  times.push_back(t1);
  std::vector<float> k1 = {-0.265379, -0.162602};
  keys.push_back(k1);

  names.push_back("HeadYaw");
  times.push_back(t1);
  k1[0] = -0.250039;
  k1[1] = -0.564504;
  keys.push_back(k1);

  names.push_back("HipPitch");
  std::vector<float> t2 = {0.08, 2.56, 4.72, 5.44, 6.4 };
  times.push_back(t2);
  std::vector<float> k2 = {0.0, 0.0, 0.0, 0.0, 0.0};
  keys.push_back(k2);


  names.push_back("HipRoll");
  times.push_back(t2);
  keys.push_back(k2);


  names.push_back("KneePitch");
  times.push_back(t2);
  keys.push_back(k2);


  names.push_back("LElbowRoll");
  times.push_back(t2);
  k2[0] = -0.524622;
  k2[1] = -0.524622;
  k2[2] = -0.524622;
  k2[3] = -0.524622;
  k2[4] = -0.524622;
  keys.push_back(k2);

  names.push_back("LElbowYaw");
  times.push_back(t2);
  k2[0] = -1.22872;
  k2[1] = -1.22872;
  k2[2] = -1.22872;
  k2[3] = -1.22872;
  k2[4] = -1.22872;
  keys.push_back(k2);

  names.push_back("LHand");
  times.push_back(t2);
  k2[0] = 0.598418;
  k2[1] = 0.598418;
  k2[2] = 0.598418;
  k2[3] = 0.598418;
  k2[4] = 0.598418;
  keys.push_back(k2);

  names.push_back("LShoulderPitch");
  times.push_back(t2);
  k2[0] = 1.56619;
  k2[1] = 1.56619;
  k2[2] = 1.56619;
  k2[3] = 1.56619;
  k2[4] = 1.56619;
  keys.push_back(k2);

  names.push_back("LShoulderRoll");
  times.push_back(t2);
  k2[0] = 0.138058;
  k2[1] = 0.138058;
  k2[2] = 0.138058;
  k2[3] = 0.138058;
  k2[4] = 0.138058;
  keys.push_back(k2);

  names.push_back("LWristYaw");
  times.push_back(t2);
  k2[0] = 0.0352399;
  k2[1] = 0.024502;
  k2[2] = 0.024502;
  k2[3] = 0.024502;
  k2[4] = 0.024502;
  keys.push_back(k2);

  names.push_back("RElbowRoll");
  times.push_back(t2);
  k2[0] = 0.139592;
  k2[1] = 0.51235;
  k2[2] = 0.147262;
  k2[3] = 0.866699;
  k2[4] = 0.908116;
  keys.push_back(k2);

  names.push_back("RElbowYaw");
  times.push_back(t2);
  k2[0] = 0.383496;
  k2[1] = 1.22718;
  k2[2] = 1.7334;
  k2[3] = 1.61375;
  k2[4] = 0.319067;
  keys.push_back(k2);

  names.push_back("RHand");
  times.push_back(t2);
  k2[0] = 0.622144;
  k2[1] = 0.587873;
  k2[2] = 0.56942;
  k2[3] = 0.884886;
  k2[4] = 0.884886;
  keys.push_back(k2);

  names.push_back("RShoulderPitch");
  times.push_back(t2);
  k2[0] = 1.4404;
  k2[1] = 1.55852;
  k2[2] = 1.51557;
  k2[3] = 0.237766;
  k2[4] = 0.075165;
  keys.push_back(k2);

  names.push_back("RShoulderRoll");
  times.push_back(t2);
  k2[0] = -0.0905049;
  k2[1] = -0.162602;
  k2[2] = -1.12134;
  k2[3] = -1.1106;
  k2[4] = -0.785398;
  keys.push_back(k2);

  names.push_back("RWristYaw");
  times.push_back(t2);
  k2[0] = 0.961776;
  k2[1] = 0.961776;
  k2[2] = 0.400331;
  k2[3] = 0.412605;
  k2[4] = 1.29465;
  keys.push_back(k2);

  try
  {
    motion.async<void>("angleInterpolationBezier", names, times, keys);
  }
  catch(const std::exception&)
  {
    std::cout << "Error during the motion" << std::endl;
    return false;
  }

  return true;

}







//bool backTointialPose(qi::AnyObject *motion)
//{
//  // Choregraphe bezier export in c++.
//  // Add #include <alproxies/almotionproxy.h> at the beginning of this file.
//  std::vector<std::string> names;
//  AL::ALValue times, keys;
//  names.reserve(17);
//  times.arraySetSize(17);
//  keys.arraySetSize(17);

//  names.push_back("HeadPitch");
//  times[0].arraySetSize(4);
//  keys[0].arraySetSize(4);

//  times[0][0] = 0.84;
//  keys[0][0] = AL::ALValue::array(-0.167204, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.293333, 0));
//  times[0][1] = 1.72;
//  keys[0][1] = AL::ALValue::array(-0.174533, AL::ALValue::array(3, -0.293333, 0), AL::ALValue::array(3, 0.32, 0));
//  times[0][2] = 2.68;
//  keys[0][2] = AL::ALValue::array(0, AL::ALValue::array(3, -0.32, -0.00306797), AL::ALValue::array(3, 0.32, 0.00306797));
//  times[0][3] = 3.64;
//  keys[0][3] = AL::ALValue::array(0.00306797, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("HeadYaw");
//  times[1].arraySetSize(4);
//  keys[1].arraySetSize(4);

//  times[1][0] = 0.84;
//  keys[1][0] = AL::ALValue::array(-0.562972, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.293333, 0));
//  times[1][1] = 1.72;
//  keys[1][1] = AL::ALValue::array(-0.349066, AL::ALValue::array(3, -0.293333, -0.0895046), AL::ALValue::array(3, 0.32, 0.0976413));
//  times[1][2] = 2.68;
//  keys[1][2] = AL::ALValue::array(-0.00153399, AL::ALValue::array(3, -0.32, -0.00613595), AL::ALValue::array(3, 0.32, 0.00613595));
//  times[1][3] = 3.64;
//  keys[1][3] = AL::ALValue::array(0.00460196, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("HipPitch");
//  times[2].arraySetSize(4);
//  keys[2].arraySetSize(4);

//  times[2][0] = 0.84;
//  keys[2][0] = AL::ALValue::array(-0.0260777, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.293333, 0));
//  times[2][1] = 1.72;
//  keys[2][1] = AL::ALValue::array(-0.0153399, AL::ALValue::array(3, -0.293333, 0), AL::ALValue::array(3, 0.32, 0));
//  times[2][2] = 2.68;
//  keys[2][2] = AL::ALValue::array(-0.0260777, AL::ALValue::array(3, -0.32, 0.00306797), AL::ALValue::array(3, 0.32, -0.00306797));
//  times[2][3] = 3.64;
//  keys[2][3] = AL::ALValue::array(-0.0337477, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("HipRoll");
//  times[3].arraySetSize(4);
//  keys[3].arraySetSize(4);

//  times[3][0] = 0.84;
//  keys[3][0] = AL::ALValue::array(0, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.293333, 0));
//  times[3][1] = 1.72;
//  keys[3][1] = AL::ALValue::array(0, AL::ALValue::array(3, -0.293333, 0), AL::ALValue::array(3, 0.32, 0));
//  times[3][2] = 2.68;
//  keys[3][2] = AL::ALValue::array(0, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0.32, 0));
//  times[3][3] = 3.64;
//  keys[3][3] = AL::ALValue::array(-0.00766992, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("KneePitch");
//  times[4].arraySetSize(4);
//  keys[4].arraySetSize(4);

//  times[4][0] = 0.84;
//  keys[4][0] = AL::ALValue::array(0.0122719, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.293333, 0));
//  times[4][1] = 1.72;
//  keys[4][1] = AL::ALValue::array(0.0230097, AL::ALValue::array(3, -0.293333, 0), AL::ALValue::array(3, 0.32, 0));
//  times[4][2] = 2.68;
//  keys[4][2] = AL::ALValue::array(0.0122719, AL::ALValue::array(3, -0.32, 0.00306796), AL::ALValue::array(3, 0.32, -0.00306796));
//  times[4][3] = 3.64;
//  keys[4][3] = AL::ALValue::array(0.00460196, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("LElbowRoll");
//  times[5].arraySetSize(4);
//  keys[5].arraySetSize(4);

//  times[5][0] = 0.84;
//  keys[5][0] = AL::ALValue::array(-0.520019, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.293333, 0));
//  times[5][1] = 1.72;
//  keys[5][1] = AL::ALValue::array(-0.520019, AL::ALValue::array(3, -0.293333, 0), AL::ALValue::array(3, 0.32, 0));
//  times[5][2] = 2.68;
//  keys[5][2] = AL::ALValue::array(-0.520019, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0.32, 0));
//  times[5][3] = 3.64;
//  keys[5][3] = AL::ALValue::array(-0.523087, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("LElbowYaw");
//  times[6].arraySetSize(4);
//  keys[6].arraySetSize(4);

//  times[6][0] = 0.84;
//  keys[6][0] = AL::ALValue::array(-1.22718, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.293333, 0));
//  times[6][1] = 1.72;
//  keys[6][1] = AL::ALValue::array(-1.22718, AL::ALValue::array(3, -0.293333, 0), AL::ALValue::array(3, 0.32, 0));
//  times[6][2] = 2.68;
//  keys[6][2] = AL::ALValue::array(-1.22718, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0.32, 0));
//  times[6][3] = 3.64;
//  keys[6][3] = AL::ALValue::array(-1.23332, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("LHand");
//  times[7].arraySetSize(4);
//  keys[7].arraySetSize(4);

//  times[7][0] = 0.84;
//  keys[7][0] = AL::ALValue::array(0.601054, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.293333, 0));
//  times[7][1] = 1.72;
//  keys[7][1] = AL::ALValue::array(0.601054, AL::ALValue::array(3, -0.293333, 0), AL::ALValue::array(3, 0.32, 0));
//  times[7][2] = 2.68;
//  keys[7][2] = AL::ALValue::array(0.601054, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0.32, 0));
//  times[7][3] = 3.64;
//  keys[7][3] = AL::ALValue::array(0.601054, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("LShoulderPitch");
//  times[8].arraySetSize(4);
//  keys[8].arraySetSize(4);

//  times[8][0] = 0.84;
//  keys[8][0] = AL::ALValue::array(1.56619, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.293333, 0));
//  times[8][1] = 1.72;
//  keys[8][1] = AL::ALValue::array(1.56619, AL::ALValue::array(3, -0.293333, 0), AL::ALValue::array(3, 0.32, 0));
//  times[8][2] = 2.68;
//  keys[8][2] = AL::ALValue::array(1.56619, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0.32, 0));
//  times[8][3] = 3.64;
//  keys[8][3] = AL::ALValue::array(1.56159, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("LShoulderRoll");
//  times[9].arraySetSize(4);
//  keys[9].arraySetSize(4);

//  times[9][0] = 0.84;
//  keys[9][0] = AL::ALValue::array(0.139592, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.293333, 0));
//  times[9][1] = 1.72;
//  keys[9][1] = AL::ALValue::array(0.139592, AL::ALValue::array(3, -0.293333, 0), AL::ALValue::array(3, 0.32, 0));
//  times[9][2] = 2.68;
//  keys[9][2] = AL::ALValue::array(0.139592, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0.32, 0));
//  times[9][3] = 3.64;
//  keys[9][3] = AL::ALValue::array(0.138058, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("LWristYaw");
//  times[10].arraySetSize(4);
//  keys[10].arraySetSize(4);

//  times[10][0] = 0.84;
//  keys[10][0] = AL::ALValue::array(0.030638, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.293333, 0));
//  times[10][1] = 1.72;
//  keys[10][1] = AL::ALValue::array(0.030638, AL::ALValue::array(3, -0.293333, 0), AL::ALValue::array(3, 0.32, 0));
//  times[10][2] = 2.68;
//  keys[10][2] = AL::ALValue::array(0.030638, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0.32, 0));
//  times[10][3] = 3.64;
//  keys[10][3] = AL::ALValue::array(0.0106959, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("RElbowRoll");
//  times[11].arraySetSize(4);
//  keys[11].arraySetSize(4);

//  times[11][0] = 0.84;
//  keys[11][0] = AL::ALValue::array(0.903515, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.293333, 0));
//  times[11][1] = 1.72;
//  keys[11][1] = AL::ALValue::array(0.872836, AL::ALValue::array(3, -0.293333, 0.0306794), AL::ALValue::array(3, 0.32, -0.0334685));
//  times[11][2] = 2.68;
//  keys[11][2] = AL::ALValue::array(0.145728, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0.32, 0));
//  times[11][3] = 3.64;
//  keys[11][3] = AL::ALValue::array(0.521553, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("RElbowYaw");
//  times[12].arraySetSize(4);
//  keys[12].arraySetSize(4);

//  times[12][0] = 0.84;
//  keys[12][0] = AL::ALValue::array(0.319067, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.293333, 0));
//  times[12][1] = 1.72;
//  keys[12][1] = AL::ALValue::array(1.62295, AL::ALValue::array(3, -0.293333, -0.0998361), AL::ALValue::array(3, 0.32, 0.108912));
//  times[12][2] = 2.68;
//  keys[12][2] = AL::ALValue::array(1.73186, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0.32, 0));
//  times[12][3] = 3.64;
//  keys[12][3] = AL::ALValue::array(1.22412, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("RHand");
//  times[13].arraySetSize(4);
//  keys[13].arraySetSize(4);

//  times[13][0] = 0.84;
//  keys[13][0] = AL::ALValue::array(0.98, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.293333, 0));
//  times[13][1] = 1.72;
//  keys[13][1] = AL::ALValue::array(0.98, AL::ALValue::array(3, -0.293333, 0), AL::ALValue::array(3, 0.32, 0));
//  times[13][2] = 2.68;
//  keys[13][2] = AL::ALValue::array(0.566784, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0.32, 0));
//  times[13][3] = 3.64;
//  keys[13][3] = AL::ALValue::array(0.59754, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("RShoulderPitch");
//  times[14].arraySetSize(4);
//  keys[14].arraySetSize(4);

//  times[14][0] = 0.84;
//  keys[14][0] = AL::ALValue::array(0.0690292, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.293333, 0));
//  times[14][1] = 1.72;
//  keys[14][1] = AL::ALValue::array(0.231631, AL::ALValue::array(3, -0.293333, -0.162602), AL::ALValue::array(3, 0.32, 0.177384));
//  times[14][2] = 2.68;
//  keys[14][2] = AL::ALValue::array(1.51251, AL::ALValue::array(3, -0.32, -0.0506198), AL::ALValue::array(3, 0.32, 0.0506198));
//  times[14][3] = 3.64;
//  keys[14][3] = AL::ALValue::array(1.56313, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("RShoulderRoll");
//  times[15].arraySetSize(4);
//  keys[15].arraySetSize(4);

//  times[15][0] = 0.84;
//  keys[15][0] = AL::ALValue::array(-0.779262, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.293333, 0));
//  times[15][1] = 1.72;
//  keys[15][1] = AL::ALValue::array(-1.10907, AL::ALValue::array(3, -0.293333, 0.0210928), AL::ALValue::array(3, 0.32, -0.0230104));
//  times[15][2] = 2.68;
//  keys[15][2] = AL::ALValue::array(-1.13208, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0.32, 0));
//  times[15][3] = 3.64;
//  keys[15][3] = AL::ALValue::array(-0.138058, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("RWristYaw");
//  times[16].arraySetSize(4);
//  keys[16].arraySetSize(4);

//  times[16][0] = 0.84;
//  keys[16][0] = AL::ALValue::array(1.30846, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.293333, 0));
//  times[16][1] = 1.72;
//  keys[16][1] = AL::ALValue::array(0.42641, AL::ALValue::array(3, -0.293333, 0.00562518), AL::ALValue::array(3, 0.32, -0.00613656));
//  times[16][2] = 2.68;
//  keys[16][2] = AL::ALValue::array(0.420274, AL::ALValue::array(3, -0.32, 0.00613656), AL::ALValue::array(3, 0.32, -0.00613656));
//  times[16][3] = 3.64;
//  keys[16][3] = AL::ALValue::array(-0.016916, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));


//  try
//  {
//    motion->angleInterpolationBezier(names, times, keys);
//  }
//  catch(const std::exception&)
//  {
//    std::cout << "Error during the motion" << std::endl;
//    return false;
//  }

//  return true;
//}




bool backTointialPose(qi::AnyObject &motion)
{
  // Choregraphe bezier export in c++.
  std::vector<std::string> names;
  std::vector<std::vector<float>> times;
  std::vector<std::vector<float>> keys;

  names.push_back("HeadPitch");
  std::vector<float> t1 = {0.84, 1.72, 2.68, 3.64};
  times.push_back(t1);
  std::vector<float> k1 = {-0.167204, -0.174533, 0.0, 0.00306797};
  keys.push_back(k1);

  names.push_back("HeadYaw");
  times.push_back(t1);
  k1[0] = -0.562972;
  k1[1] = -0.349066;
  k1[2] = -0.00153399;
  k1[3] = 0.00460196;
  keys.push_back(k1);



  names.push_back("HipPitch");
  times.push_back(t1);
  k1[0] = -0.0260777;
  k1[1] = -0.0153399;
  k1[2] = -0.0260777;
  k1[3] = -0.0337477;
  keys.push_back(k1);

  names.push_back("HipRoll");
  times.push_back(t1);
  k1[0] = 0.0;
  k1[1] = 0.0;
  k1[2] = 0.0;
  k1[3] = -0.00766992;
  keys.push_back(k1);

  names.push_back("KneePitch");
  times.push_back(t1);
  k1[0] = 0.0122719;
  k1[1] = 0.0230097;
  k1[2] = 0.0122719;
  k1[3] = 0.00460196;
  keys.push_back(k1);

  names.push_back("LElbowRoll");
  times.push_back(t1);
  k1[0] = -0.520019;
  k1[1] = -0.520019;
  k1[2] = -0.520019;
  k1[3] = -0.523087;
  keys.push_back(k1);


  names.push_back("LElbowYaw");
  times.push_back(t1);
  k1[0] = -1.22718;
  k1[1] = -1.22718;
  k1[2] = -1.22718;
  k1[3] = -1.23332;
  keys.push_back(k1);

  names.push_back("LHand");
  times.push_back(t1);
  k1[0] = 0.601054;
  k1[1] = 0.601054;
  k1[2] = 0.601054;
  k1[3] = 0.601054;
  keys.push_back(k1);

  names.push_back("LShoulderPitch");
  times.push_back(t1);
  k1[0] = 1.56619;
  k1[1] = 1.56619;
  k1[2] = 1.56619;
  k1[3] = 1.56619;
  keys.push_back(k1);

  names.push_back("LShoulderRoll");
  times.push_back(t1);
  k1[0] = 0.139592;
  k1[1] = 0.139592;
  k1[2] = 0.139592;
  k1[3] = 0.139592;
  keys.push_back(k1);

  names.push_back("LWristYaw");
  times.push_back(t1);
  k1[0] = 0.030638;
  k1[1] = 0.030638;
  k1[2] = 0.030638;
  k1[3] = 0.0106959;
  keys.push_back(k1);

  names.push_back("RElbowRoll");
  times.push_back(t1);
  k1[0] = 0.903515;
  k1[1] = 0.872836;
  k1[2] = 0.145728;
  k1[3] = 0.521553;
  keys.push_back(k1);

  names.push_back("RElbowYaw");
  times.push_back(t1);
  k1[0] = 0.319067;
  k1[1] = 1.62295;
  k1[2] = 1.73186;
  k1[3] = 1.22412;
  keys.push_back(k1);

  names.push_back("RHand");
  times.push_back(t1);
  k1[0] = 0.98;
  k1[1] = 0.98;
  k1[2] = 0.566784;
  k1[3] = 0.59754;
  keys.push_back(k1);

  names.push_back("RShoulderPitch");
  times.push_back(t1);
  k1[0] = 0.0690292;
  k1[1] = 0.231631;
  k1[2] = 1.51251;
  k1[3] = 1.56313;
  keys.push_back(k1);

  names.push_back("RShoulderRoll");
  times.push_back(t1);
  k1[0] = -0.779262;
  k1[1] = -1.10907;
  k1[2] = -1.13208;
  k1[3] = -0.138058;
  keys.push_back(k1);

  names.push_back("RWristYaw");
  times.push_back(t1);
  k1[0] = 1.30846;
  k1[1] = 0.42641;
  k1[2] = 0.420274;
  k1[3] = -0.016916;
  keys.push_back(k1);


  try
  {
    motion.async<void>("angleInterpolationBezier", names, times, keys);
  }
  catch(const std::exception&)
  {
    std::cout << "Error during the motion" << std::endl;
    return false;
  }

  return true;
}





//bool goToInitialPoseBase(qi::AnyObject *motion)
//{
//  // Choregraphe bezier export in c++.
//  // Add #include <alproxies/almotionproxy.h> at the beginning of this file.
//  std::vector<std::string> names;
//  AL::ALValue times, keys;
//  names.reserve(17);
//  times.arraySetSize(17);
//  keys.arraySetSize(17);

//  names.push_back("HeadPitch");
//  times[0].arraySetSize(1);
//  keys[0].arraySetSize(1);

//  times[0][0] = 1.76;
//  keys[0][0] = AL::ALValue::array(-0.360486, AL::ALValue::array(3, -0.586667, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("HeadYaw");
//  times[1].arraySetSize(1);
//  keys[1].arraySetSize(1);

//  times[1][0] = 1.76;
//  keys[1][0] = AL::ALValue::array(-0.250039, AL::ALValue::array(3, -0.586667, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("HipPitch");
//  times[2].arraySetSize(1);
//  keys[2].arraySetSize(1);

//  times[2][0] = 1.76;
//  keys[2][0] = AL::ALValue::array(0.00920391, AL::ALValue::array(3, -0.586667, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("HipRoll");
//  times[3].arraySetSize(1);
//  keys[3].arraySetSize(1);

//  times[3][0] = 1.76;
//  keys[3][0] = AL::ALValue::array(0.00460196, AL::ALValue::array(3, -0.586667, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("KneePitch");
//  times[4].arraySetSize(1);
//  keys[4].arraySetSize(1);

//  times[4][0] = 1.76;
//  keys[4][0] = AL::ALValue::array(0.0184078, AL::ALValue::array(3, -0.586667, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("LElbowRoll");
//  times[5].arraySetSize(1);
//  keys[5].arraySetSize(1);

//  times[5][0] = 1.76;
//  keys[5][0] = AL::ALValue::array(-0.530757, AL::ALValue::array(3, -0.586667, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("LElbowYaw");
//  times[6].arraySetSize(1);
//  keys[6].arraySetSize(1);

//  times[6][0] = 1.76;
//  keys[6][0] = AL::ALValue::array(-1.23485, AL::ALValue::array(3, -0.586667, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("LHand");
//  times[7].arraySetSize(1);
//  keys[7].arraySetSize(1);

//  times[7][0] = 1.76;
//  keys[7][0] = AL::ALValue::array(0.588752, AL::ALValue::array(3, -0.586667, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("LShoulderPitch");
//  times[8].arraySetSize(1);
//  keys[8].arraySetSize(1);

//  times[8][0] = 1.76;
//  keys[8][0] = AL::ALValue::array(1.56619, AL::ALValue::array(3, -0.586667, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("LShoulderRoll");
//  times[9].arraySetSize(1);
//  keys[9].arraySetSize(1);

//  times[9][0] = 1.76;
//  keys[9][0] = AL::ALValue::array(0.141126, AL::ALValue::array(3, -0.586667, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("LWristYaw");
//  times[10].arraySetSize(1);
//  keys[10].arraySetSize(1);

//  times[10][0] = 1.76;
//  keys[10][0] = AL::ALValue::array(0.05825, AL::ALValue::array(3, -0.586667, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("RElbowRoll");
//  times[11].arraySetSize(1);
//  keys[11].arraySetSize(1);

//  times[11][0] = 1.76;
//  keys[11][0] = AL::ALValue::array(0.0168738, AL::ALValue::array(3, -0.586667, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("RElbowYaw");
//  times[12].arraySetSize(1);
//  keys[12].arraySetSize(1);

//  times[12][0] = 1.76;
//  keys[12][0] = AL::ALValue::array(0.483204, AL::ALValue::array(3, -0.586667, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("RHand");
//  times[13].arraySetSize(1);
//  keys[13].arraySetSize(1);

//  times[13][0] = 1.76;
//  keys[13][0] = AL::ALValue::array(0.601054, AL::ALValue::array(3, -0.586667, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("RShoulderPitch");
//  times[14].arraySetSize(1);
//  keys[14].arraySetSize(1);

//  times[14][0] = 1.76;
//  keys[14][0] = AL::ALValue::array(1.40359, AL::ALValue::array(3, -0.586667, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("RShoulderRoll");
//  times[15].arraySetSize(1);
//  keys[15].arraySetSize(1);

//  times[15][0] = 1.76;
//  keys[15][0] = AL::ALValue::array(-0.0153399, AL::ALValue::array(3, -0.586667, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("RWristYaw");
//  times[16].arraySetSize(1);
//  keys[16].arraySetSize(1);

//  times[16][0] = 1.76;
//  keys[16][0] = AL::ALValue::array(0.753152, AL::ALValue::array(3, -0.586667, 0), AL::ALValue::array(3, 0, 0));

//  try
//  {
//    motion->post.angleInterpolationBezier(names, times, keys);
//  }
//  catch(const std::exception&)
//  {
//    std::cout << "Error during the motion" << std::endl;
//    return false;
//  }

//  return true;
//}



bool goToInitialPoseBase(qi::AnyObject &motion)
{
  // Choregraphe bezier export in c++.
  std::vector<std::string> names;
  std::vector<std::vector<float>> times;
  std::vector<std::vector<float>> keys;

  names.push_back("HeadPitch");
  std::vector<float> t1 = {1.76};
  times.push_back(t1);
  std::vector<float> k1 = {-0.360486};
  keys.push_back(k1);


  names.push_back("HeadYaw");
  times.push_back(t1);
  k1[0] = -0.250039;
  keys.push_back(k1);


  names.push_back("HipPitch");
  times.push_back(t1);
  k1[0] = 0.00920391;
  keys.push_back(k1);

  names.push_back("HipRoll");
  times.push_back(t1);
  k1[0] = 0.00460196;
  keys.push_back(k1);

  names.push_back("KneePitch");
  times.push_back(t1);
  k1[0] = 0.0184078;
  keys.push_back(k1);

  names.push_back("LElbowRoll");
  times.push_back(t1);
  k1[0] = -0.530757;
  keys.push_back(k1);

  names.push_back("LElbowYaw");
  times.push_back(t1);
  k1[0] = -1.23485;
  keys.push_back(k1);

  names.push_back("LHand");
  times.push_back(t1);
  k1[0] = 0.588752;
  keys.push_back(k1);


  names.push_back("LShoulderPitch");
  times.push_back(t1);
  k1[0] = 1.56619;
  keys.push_back(k1);

  names.push_back("LShoulderRoll");
  times.push_back(t1);
  k1[0] = 0.141126;
  keys.push_back(k1);

  names.push_back("LWristYaw");
  times.push_back(t1);
  k1[0] = 0.05825;
  keys.push_back(k1);

  names.push_back("RElbowRoll");
  times.push_back(t1);
  k1[0] = 0.0168738;
  keys.push_back(k1);

  names.push_back("RElbowYaw");
  times.push_back(t1);
  k1[0] = 0.483204;
  keys.push_back(k1);

  names.push_back("RHand");
  times.push_back(t1);
  k1[0] = 0.601054;
  keys.push_back(k1);

  names.push_back("RShoulderPitch");
  times.push_back(t1);
  k1[0] = 1.40359;
  keys.push_back(k1);

  names.push_back("RShoulderRoll");
  times.push_back(t1);
  k1[0] = -0.0153399;
  keys.push_back(k1);

  names.push_back("RWristYaw");
  times.push_back(t1);
  k1[0] = 0.753152;
  keys.push_back(k1);

  try
  {
    motion.async<void>("angleInterpolationBezier", names, times, keys);
  }
  catch(const std::exception&)
  {
    std::cout << "Error during the motion" << std::endl;
    return false;
  }

  return true;
}



//bool goToInitialPBVSPoseBase(qi::AnyObject *motion)
//{
//  // Choregraphe bezier export in c++.
//  // Add #include <alproxies/almotionproxy.h> at the beginning of this file.
//  std::vector<std::string> names;
//  AL::ALValue times, keys;
//  names.reserve(17);
//  times.arraySetSize(17);
//  keys.arraySetSize(17);

//  names.push_back("HeadPitch");
//  times[0].arraySetSize(1);
//  keys[0].arraySetSize(1);

//  times[0][0] = 1.52;
//  keys[0][0] = AL::ALValue::array(-0.265379, AL::ALValue::array(3, -0.506667, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("HeadYaw");
//  times[1].arraySetSize(1);
//  keys[1].arraySetSize(1);

//  times[1][0] = 1.52;
//  keys[1][0] = AL::ALValue::array(-0.250039, AL::ALValue::array(3, -0.506667, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("HipPitch");
//  times[2].arraySetSize(1);
//  keys[2].arraySetSize(1);

//  times[2][0] = 1.52;
//  keys[2][0] = AL::ALValue::array(0, AL::ALValue::array(3, -0.506667, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("HipRoll");
//  times[3].arraySetSize(1);
//  keys[3].arraySetSize(1);

//  times[3][0] = 1.52;
//  keys[3][0] = AL::ALValue::array(0, AL::ALValue::array(3, -0.506667, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("KneePitch");
//  times[4].arraySetSize(1);
//  keys[4].arraySetSize(1);

//  times[4][0] = 1.52;
//  keys[4][0] = AL::ALValue::array(0, AL::ALValue::array(3, -0.506667, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("LElbowRoll");
//  times[5].arraySetSize(1);
//  keys[5].arraySetSize(1);

//  times[5][0] = 1.52;
//  keys[5][0] = AL::ALValue::array(-0.524621, AL::ALValue::array(3, -0.506667, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("LElbowYaw");
//  times[6].arraySetSize(1);
//  keys[6].arraySetSize(1);

//  times[6][0] = 1.52;
//  keys[6][0] = AL::ALValue::array(-1.22872, AL::ALValue::array(3, -0.506667, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("LHand");
//  times[7].arraySetSize(1);
//  keys[7].arraySetSize(1);

//  times[7][0] = 1.52;
//  keys[7][0] = AL::ALValue::array(0.598418, AL::ALValue::array(3, -0.506667, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("LShoulderPitch");
//  times[8].arraySetSize(1);
//  keys[8].arraySetSize(1);

//  times[8][0] = 1.52;
//  keys[8][0] = AL::ALValue::array(1.56619, AL::ALValue::array(3, -0.506667, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("LShoulderRoll");
//  times[9].arraySetSize(1);
//  keys[9].arraySetSize(1);

//  times[9][0] = 1.52;
//  keys[9][0] = AL::ALValue::array(0.138058, AL::ALValue::array(3, -0.506667, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("LWristYaw");
//  times[10].arraySetSize(1);
//  keys[10].arraySetSize(1);

//  times[10][0] = 1.52;
//  keys[10][0] = AL::ALValue::array(0.0352399, AL::ALValue::array(3, -0.506667, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("RElbowRoll");
//  times[11].arraySetSize(1);
//  keys[11].arraySetSize(1);

//  times[11][0] = 1.52;
//  keys[11][0] = AL::ALValue::array(0.139592, AL::ALValue::array(3, -0.506667, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("RElbowYaw");
//  times[12].arraySetSize(1);
//  keys[12].arraySetSize(1);

//  times[12][0] = 1.52;
//  keys[12][0] = AL::ALValue::array(0.383495, AL::ALValue::array(3, -0.506667, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("RHand");
//  times[13].arraySetSize(1);
//  keys[13].arraySetSize(1);

//  times[13][0] = 1.52;
//  keys[13][0] = AL::ALValue::array(0.622144, AL::ALValue::array(3, -0.506667, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("RShoulderPitch");
//  times[14].arraySetSize(1);
//  keys[14].arraySetSize(1);

//  times[14][0] = 1.52;
//  keys[14][0] = AL::ALValue::array(1.44041, AL::ALValue::array(3, -0.506667, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("RShoulderRoll");
//  times[15].arraySetSize(1);
//  keys[15].arraySetSize(1);

//  times[15][0] = 1.52;
//  keys[15][0] = AL::ALValue::array(-0.0905049, AL::ALValue::array(3, -0.506667, 0), AL::ALValue::array(3, 0, 0));

//  names.push_back("RWristYaw");
//  times[16].arraySetSize(1);
//  keys[16].arraySetSize(1);

//  times[16][0] = 1.52;
//  keys[16][0] = AL::ALValue::array(0.961776, AL::ALValue::array(3, -0.506667, 0), AL::ALValue::array(3, 0, 0));
//  try
//  {
//    motion->post.angleInterpolationBezier(names, times, keys);
//  }
//  catch(const std::exception&)
//  {
//    std::cout << "Error during the motion" << std::endl;
//    return false;
//  }

//  return true;
//}




bool goToInitialPBVSPoseBase(qi::AnyObject &motion)
{
  // Choregraphe bezier export in c++.
  std::vector<std::string> names;
  std::vector<std::vector<float>> times;
  std::vector<std::vector<float>> keys;

  names.push_back("HeadPitch");
  std::vector<float> t1 = {1.52};
  times.push_back(t1);
  std::vector<float> k1 = {-0.265379};
  keys.push_back(k1);


  names.push_back("HeadYaw");
  times.push_back(t1);
  k1[0] = -0.250039;
  keys.push_back(k1);


  names.push_back("HipPitch");
  times.push_back(t1);
  k1[0] = 0.0;
  keys.push_back(k1);

  names.push_back("HipRoll");
  times.push_back(t1);
  k1[0] = 0.0;
  keys.push_back(k1);

  names.push_back("KneePitch");
  times.push_back(t1);
  k1[0] = 0.0;
  keys.push_back(k1);

  names.push_back("LElbowRoll");
  times.push_back(t1);
  k1[0] = -0.524621;
  keys.push_back(k1);

  names.push_back("LElbowYaw");
  times.push_back(t1);
  k1[0] = -1.22872;
  keys.push_back(k1);

  names.push_back("LHand");
  times.push_back(t1);
  k1[0] = 0.598418;
  keys.push_back(k1);


  names.push_back("LShoulderPitch");
  times.push_back(t1);
  k1[0] = 1.56619;
  keys.push_back(k1);

  names.push_back("LShoulderRoll");
  times.push_back(t1);
  k1[0] = 0.138058;
  keys.push_back(k1);

  names.push_back("LWristYaw");
  times.push_back(t1);
  k1[0] = 0.0352399;
  keys.push_back(k1);

  names.push_back("RElbowRoll");
  times.push_back(t1);
  k1[0] = 0.139592;
  keys.push_back(k1);

  names.push_back("RElbowYaw");
  times.push_back(t1);
  k1[0] = 0.383495;
  keys.push_back(k1);

  names.push_back("RHand");
  times.push_back(t1);
  k1[0] = 0.622144;
  keys.push_back(k1);

  names.push_back("RShoulderPitch");
  times.push_back(t1);
  k1[0] = 1.44041;
  keys.push_back(k1);

  names.push_back("RShoulderRoll");
  times.push_back(t1);
  k1[0] = -0.0905049;
  keys.push_back(k1);

  names.push_back("RWristYaw");
  times.push_back(t1);
  k1[0] = 0.961776;
  keys.push_back(k1);

  try
  {
    motion.async<void>("angleInterpolationBezier", names, times, keys);
  }
  catch(const std::exception&)
  {
    std::cout << "Error during the motion" << std::endl;
    return false;
  }

  return true;
}

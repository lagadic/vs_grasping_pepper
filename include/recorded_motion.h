#include <iostream>
#include <vector>

#include <alproxies/almotionproxy.h>



bool goTointialPose(AL::ALMotionProxy *motion)
{
  // Choregraphe bezier export in c++.
  // Add #include <alproxies/almotionproxy.h> at the beginning of this file.
  std::vector<std::string> names;
  AL::ALValue times, keys;
  names.reserve(17);
  times.arraySetSize(17);
  keys.arraySetSize(17);

  names.push_back("HeadPitch");
  times[0].arraySetSize(4);
  keys[0].arraySetSize(4);

  times[0][0] = 1;
  keys[0][0] = AL::ALValue::array(0.00306797, AL::ALValue::array(3, -0.333333, 0), AL::ALValue::array(3, 0.28, 0));
  times[0][1] = 1.84;
  keys[0][1] = AL::ALValue::array(0, AL::ALValue::array(3, -0.28, 0.00306797), AL::ALValue::array(3, 0.333333, -0.00365235));
  times[0][2] = 2.84;
  keys[0][2] = AL::ALValue::array(-0.174533, AL::ALValue::array(3, -0.333333, 0), AL::ALValue::array(3, 0.306667, 0));
  times[0][3] = 3.76;
  keys[0][3] = AL::ALValue::array(-0.167204, AL::ALValue::array(3, -0.306667, 0), AL::ALValue::array(3, 0, 0));

  names.push_back("HeadYaw");
  times[1].arraySetSize(4);
  keys[1].arraySetSize(4);

  times[1][0] = 1;
  keys[1][0] = AL::ALValue::array(0.00460196, AL::ALValue::array(3, -0.333333, 0), AL::ALValue::array(3, 0.28, 0));
  times[1][1] = 1.84;
  keys[1][1] = AL::ALValue::array(-0.00153399, AL::ALValue::array(3, -0.28, 0.00613595), AL::ALValue::array(3, 0.333333, -0.0073047));
  times[1][2] = 2.84;
  keys[1][2] = AL::ALValue::array(-0.349066, AL::ALValue::array(3, -0.333333, 0.00153399), AL::ALValue::array(3, 0.306667, -0.00141127));
  times[1][3] = 3.76;
  keys[1][3] = AL::ALValue::array(-0.562971, AL::ALValue::array(3, -0.306667, 0), AL::ALValue::array(3, 0, 0));

  names.push_back("HipPitch");
  times[2].arraySetSize(4);
  keys[2].arraySetSize(4);

  times[2][0] = 1;
  keys[2][0] = AL::ALValue::array(-0.0337477, AL::ALValue::array(3, -0.333333, 0), AL::ALValue::array(3, 0.28, 0));
  times[2][1] = 1.84;
  keys[2][1] = AL::ALValue::array(-0.0260777, AL::ALValue::array(3, -0.28, -0.00280119), AL::ALValue::array(3, 0.333333, 0.00333475));
  times[2][2] = 2.84;
  keys[2][2] = AL::ALValue::array(-0.0153399, AL::ALValue::array(3, -0.333333, 0), AL::ALValue::array(3, 0.306667, 0));
  times[2][3] = 3.76;
  keys[2][3] = AL::ALValue::array(-0.0260777, AL::ALValue::array(3, -0.306667, 0), AL::ALValue::array(3, 0, 0));

  names.push_back("HipRoll");
  times[3].arraySetSize(4);
  keys[3].arraySetSize(4);

  times[3][0] = 1;
  keys[3][0] = AL::ALValue::array(-0.00766992, AL::ALValue::array(3, -0.333333, 0), AL::ALValue::array(3, 0.28, 0));
  times[3][1] = 1.84;
  keys[3][1] = AL::ALValue::array(0, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.333333, 0));
  times[3][2] = 2.84;
  keys[3][2] = AL::ALValue::array(0, AL::ALValue::array(3, -0.333333, 0), AL::ALValue::array(3, 0.306667, 0));
  times[3][3] = 3.76;
  keys[3][3] = AL::ALValue::array(0, AL::ALValue::array(3, -0.306667, 0), AL::ALValue::array(3, 0, 0));

  names.push_back("KneePitch");
  times[4].arraySetSize(4);
  keys[4].arraySetSize(4);

  times[4][0] = 1;
  keys[4][0] = AL::ALValue::array(0.00460196, AL::ALValue::array(3, -0.333333, 0), AL::ALValue::array(3, 0.28, 0));
  times[4][1] = 1.84;
  keys[4][1] = AL::ALValue::array(0.0122719, AL::ALValue::array(3, -0.28, -0.00280119), AL::ALValue::array(3, 0.333333, 0.00333475));
  times[4][2] = 2.84;
  keys[4][2] = AL::ALValue::array(0.0230098, AL::ALValue::array(3, -0.333333, 0), AL::ALValue::array(3, 0.306667, 0));
  times[4][3] = 3.76;
  keys[4][3] = AL::ALValue::array(0.0122719, AL::ALValue::array(3, -0.306667, 0), AL::ALValue::array(3, 0, 0));

  names.push_back("LElbowRoll");
  times[5].arraySetSize(4);
  keys[5].arraySetSize(4);

  times[5][0] = 1;
  keys[5][0] = AL::ALValue::array(-0.523087, AL::ALValue::array(3, -0.333333, 0), AL::ALValue::array(3, 0.28, 0));
  times[5][1] = 1.84;
  keys[5][1] = AL::ALValue::array(-0.52002, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.333333, 0));
  times[5][2] = 2.84;
  keys[5][2] = AL::ALValue::array(-0.52002, AL::ALValue::array(3, -0.333333, 0), AL::ALValue::array(3, 0.306667, 0));
  times[5][3] = 3.76;
  keys[5][3] = AL::ALValue::array(-0.52002, AL::ALValue::array(3, -0.306667, 0), AL::ALValue::array(3, 0, 0));

  names.push_back("LElbowYaw");
  times[6].arraySetSize(4);
  keys[6].arraySetSize(4);

  times[6][0] = 1;
  keys[6][0] = AL::ALValue::array(-1.23332, AL::ALValue::array(3, -0.333333, 0), AL::ALValue::array(3, 0.28, 0));
  times[6][1] = 1.84;
  keys[6][1] = AL::ALValue::array(-1.22718, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.333333, 0));
  times[6][2] = 2.84;
  keys[6][2] = AL::ALValue::array(-1.22718, AL::ALValue::array(3, -0.333333, 0), AL::ALValue::array(3, 0.306667, 0));
  times[6][3] = 3.76;
  keys[6][3] = AL::ALValue::array(-1.22718, AL::ALValue::array(3, -0.306667, 0), AL::ALValue::array(3, 0, 0));

  names.push_back("LHand");
  times[7].arraySetSize(4);
  keys[7].arraySetSize(4);

  times[7][0] = 1;
  keys[7][0] = AL::ALValue::array(0.601054, AL::ALValue::array(3, -0.333333, 0), AL::ALValue::array(3, 0.28, 0));
  times[7][1] = 1.84;
  keys[7][1] = AL::ALValue::array(0.601054, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.333333, 0));
  times[7][2] = 2.84;
  keys[7][2] = AL::ALValue::array(0.601054, AL::ALValue::array(3, -0.333333, 0), AL::ALValue::array(3, 0.306667, 0));
  times[7][3] = 3.76;
  keys[7][3] = AL::ALValue::array(0.601054, AL::ALValue::array(3, -0.306667, 0), AL::ALValue::array(3, 0, 0));

  names.push_back("LShoulderPitch");
  times[8].arraySetSize(4);
  keys[8].arraySetSize(4);

  times[8][0] = 1;
  keys[8][0] = AL::ALValue::array(1.56159, AL::ALValue::array(3, -0.333333, 0), AL::ALValue::array(3, 0.28, 0));
  times[8][1] = 1.84;
  keys[8][1] = AL::ALValue::array(1.56619, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.333333, 0));
  times[8][2] = 2.84;
  keys[8][2] = AL::ALValue::array(1.56619, AL::ALValue::array(3, -0.333333, 0), AL::ALValue::array(3, 0.306667, 0));
  times[8][3] = 3.76;
  keys[8][3] = AL::ALValue::array(1.56619, AL::ALValue::array(3, -0.306667, 0), AL::ALValue::array(3, 0, 0));

  names.push_back("LShoulderRoll");
  times[9].arraySetSize(4);
  keys[9].arraySetSize(4);

  times[9][0] = 1;
  keys[9][0] = AL::ALValue::array(0.138058, AL::ALValue::array(3, -0.333333, 0), AL::ALValue::array(3, 0.28, 0));
  times[9][1] = 1.84;
  keys[9][1] = AL::ALValue::array(0.139592, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.333333, 0));
  times[9][2] = 2.84;
  keys[9][2] = AL::ALValue::array(0.139592, AL::ALValue::array(3, -0.333333, 0), AL::ALValue::array(3, 0.306667, 0));
  times[9][3] = 3.76;
  keys[9][3] = AL::ALValue::array(0.139592, AL::ALValue::array(3, -0.306667, 0), AL::ALValue::array(3, 0, 0));

  names.push_back("LWristYaw");
  times[10].arraySetSize(4);
  keys[10].arraySetSize(4);

  times[10][0] = 1;
  keys[10][0] = AL::ALValue::array(0.0106959, AL::ALValue::array(3, -0.333333, 0), AL::ALValue::array(3, 0.28, 0));
  times[10][1] = 1.84;
  keys[10][1] = AL::ALValue::array(0.030638, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.333333, 0));
  times[10][2] = 2.84;
  keys[10][2] = AL::ALValue::array(0.030638, AL::ALValue::array(3, -0.333333, 0), AL::ALValue::array(3, 0.306667, 0));
  times[10][3] = 3.76;
  keys[10][3] = AL::ALValue::array(0.030638, AL::ALValue::array(3, -0.306667, 0), AL::ALValue::array(3, 0, 0));

  names.push_back("RElbowRoll");
  times[11].arraySetSize(4);
  keys[11].arraySetSize(4);

  times[11][0] = 1;
  keys[11][0] = AL::ALValue::array(0.521553, AL::ALValue::array(3, -0.333333, 0), AL::ALValue::array(3, 0.28, 0));
  times[11][1] = 1.84;
  keys[11][1] = AL::ALValue::array(0.145728, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.333333, 0));
  times[11][2] = 2.84;
  keys[11][2] = AL::ALValue::array(0.872835, AL::ALValue::array(3, -0.333333, -0.0333475), AL::ALValue::array(3, 0.306667, 0.0306797));
  times[11][3] = 3.76;
  keys[11][3] = AL::ALValue::array(0.903515, AL::ALValue::array(3, -0.306667, 0), AL::ALValue::array(3, 0, 0));

  names.push_back("RElbowYaw");
  times[12].arraySetSize(4);
  keys[12].arraySetSize(4);

  times[12][0] = 1;
  keys[12][0] = AL::ALValue::array(1.22412, AL::ALValue::array(3, -0.333333, 0), AL::ALValue::array(3, 0.28, 0));
  times[12][1] = 1.84;
  keys[12][1] = AL::ALValue::array(1.73186, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.333333, 0));
  times[12][2] = 2.84;
  keys[12][2] = AL::ALValue::array(1.62295, AL::ALValue::array(3, -0.333333, 0.108913), AL::ALValue::array(3, 0.306667, -0.100199));
  times[12][3] = 3.76;
  keys[12][3] = AL::ALValue::array(0.319068, AL::ALValue::array(3, -0.306667, 0), AL::ALValue::array(3, 0, 0));

  names.push_back("RHand");
  times[13].arraySetSize(4);
  keys[13].arraySetSize(4);

  times[13][0] = 1;
  keys[13][0] = AL::ALValue::array(0.59754, AL::ALValue::array(3, -0.333333, 0), AL::ALValue::array(3, 0.28, 0));
  times[13][1] = 1.84;
  keys[13][1] = AL::ALValue::array(0.566784, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.333333, 0));
  times[13][2] = 2.84;
  keys[13][2] = AL::ALValue::array(0.98, AL::ALValue::array(3, -0.333333, 0), AL::ALValue::array(3, 0.306667, 0));
  times[13][3] = 3.76;
  keys[13][3] = AL::ALValue::array(0.98, AL::ALValue::array(3, -0.306667, 0), AL::ALValue::array(3, 0, 0));

  names.push_back("RShoulderPitch");
  times[14].arraySetSize(4);
  keys[14].arraySetSize(4);

  times[14][0] = 1;
  keys[14][0] = AL::ALValue::array(1.56313, AL::ALValue::array(3, -0.333333, 0), AL::ALValue::array(3, 0.28, 0));
  times[14][1] = 1.84;
  keys[14][1] = AL::ALValue::array(1.51251, AL::ALValue::array(3, -0.28, 0.0506203), AL::ALValue::array(3, 0.333333, -0.0602623));
  times[14][2] = 2.84;
  keys[14][2] = AL::ALValue::array(0.231631, AL::ALValue::array(3, -0.333333, 0.176741), AL::ALValue::array(3, 0.306667, -0.162602));
  times[14][3] = 3.76;
  keys[14][3] = AL::ALValue::array(0.0690291, AL::ALValue::array(3, -0.306667, 0), AL::ALValue::array(3, 0, 0));

  names.push_back("RShoulderRoll");
  times[15].arraySetSize(4);
  keys[15].arraySetSize(4);

  times[15][0] = 1;
  keys[15][0] = AL::ALValue::array(-0.138058, AL::ALValue::array(3, -0.333333, 0), AL::ALValue::array(3, 0.28, 0));
  times[15][1] = 1.84;
  keys[15][1] = AL::ALValue::array(-1.13208, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.333333, 0));
  times[15][2] = 2.84;
  keys[15][2] = AL::ALValue::array(-1.10907, AL::ALValue::array(3, -0.333333, -0.0230098), AL::ALValue::array(3, 0.306667, 0.021169));
  times[15][3] = 3.76;
  keys[15][3] = AL::ALValue::array(-0.779262, AL::ALValue::array(3, -0.306667, 0), AL::ALValue::array(3, 0, 0));

  names.push_back("RWristYaw");
  times[16].arraySetSize(4);
  keys[16].arraySetSize(4);

  times[16][0] = 1;
  keys[16][0] = AL::ALValue::array(-0.016916, AL::ALValue::array(3, -0.333333, 0), AL::ALValue::array(3, 0.28, 0));
  times[16][1] = 1.84;
  keys[16][1] = AL::ALValue::array(0.420274, AL::ALValue::array(3, -0.28, -0.0051542), AL::ALValue::array(3, 0.333333, 0.00613596));
  times[16][2] = 2.84;
  keys[16][2] = AL::ALValue::array(0.42641, AL::ALValue::array(3, -0.333333, -0.00613596), AL::ALValue::array(3, 0.306667, 0.00564508));
  times[16][3] = 3.76;
  keys[16][3] = AL::ALValue::array(1.30846, AL::ALValue::array(3, -0.306667, 0), AL::ALValue::array(3, 0, 0));
  try
  {
    motion->angleInterpolationBezier(names, times, keys);
  }
  catch(const std::exception&)
  {
    std::cout << "Error during the motion" << std::endl;
    return false;
  }

  return true;

}

bool backTointialPose(AL::ALMotionProxy *motion)
{
  // Choregraphe bezier export in c++.
  // Add #include <alproxies/almotionproxy.h> at the beginning of this file.
  std::vector<std::string> names;
  AL::ALValue times, keys;
  names.reserve(17);
  times.arraySetSize(17);
  keys.arraySetSize(17);

  names.push_back("HeadPitch");
  times[0].arraySetSize(4);
  keys[0].arraySetSize(4);

  times[0][0] = 0.84;
  keys[0][0] = AL::ALValue::array(-0.167204, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.293333, 0));
  times[0][1] = 1.72;
  keys[0][1] = AL::ALValue::array(-0.174533, AL::ALValue::array(3, -0.293333, 0), AL::ALValue::array(3, 0.32, 0));
  times[0][2] = 2.68;
  keys[0][2] = AL::ALValue::array(0, AL::ALValue::array(3, -0.32, -0.00306797), AL::ALValue::array(3, 0.32, 0.00306797));
  times[0][3] = 3.64;
  keys[0][3] = AL::ALValue::array(0.00306797, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

  names.push_back("HeadYaw");
  times[1].arraySetSize(4);
  keys[1].arraySetSize(4);

  times[1][0] = 0.84;
  keys[1][0] = AL::ALValue::array(-0.562972, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.293333, 0));
  times[1][1] = 1.72;
  keys[1][1] = AL::ALValue::array(-0.349066, AL::ALValue::array(3, -0.293333, -0.0895046), AL::ALValue::array(3, 0.32, 0.0976413));
  times[1][2] = 2.68;
  keys[1][2] = AL::ALValue::array(-0.00153399, AL::ALValue::array(3, -0.32, -0.00613595), AL::ALValue::array(3, 0.32, 0.00613595));
  times[1][3] = 3.64;
  keys[1][3] = AL::ALValue::array(0.00460196, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

  names.push_back("HipPitch");
  times[2].arraySetSize(4);
  keys[2].arraySetSize(4);

  times[2][0] = 0.84;
  keys[2][0] = AL::ALValue::array(-0.0260777, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.293333, 0));
  times[2][1] = 1.72;
  keys[2][1] = AL::ALValue::array(-0.0153399, AL::ALValue::array(3, -0.293333, 0), AL::ALValue::array(3, 0.32, 0));
  times[2][2] = 2.68;
  keys[2][2] = AL::ALValue::array(-0.0260777, AL::ALValue::array(3, -0.32, 0.00306797), AL::ALValue::array(3, 0.32, -0.00306797));
  times[2][3] = 3.64;
  keys[2][3] = AL::ALValue::array(-0.0337477, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

  names.push_back("HipRoll");
  times[3].arraySetSize(4);
  keys[3].arraySetSize(4);

  times[3][0] = 0.84;
  keys[3][0] = AL::ALValue::array(0, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.293333, 0));
  times[3][1] = 1.72;
  keys[3][1] = AL::ALValue::array(0, AL::ALValue::array(3, -0.293333, 0), AL::ALValue::array(3, 0.32, 0));
  times[3][2] = 2.68;
  keys[3][2] = AL::ALValue::array(0, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0.32, 0));
  times[3][3] = 3.64;
  keys[3][3] = AL::ALValue::array(-0.00766992, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

  names.push_back("KneePitch");
  times[4].arraySetSize(4);
  keys[4].arraySetSize(4);

  times[4][0] = 0.84;
  keys[4][0] = AL::ALValue::array(0.0122719, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.293333, 0));
  times[4][1] = 1.72;
  keys[4][1] = AL::ALValue::array(0.0230097, AL::ALValue::array(3, -0.293333, 0), AL::ALValue::array(3, 0.32, 0));
  times[4][2] = 2.68;
  keys[4][2] = AL::ALValue::array(0.0122719, AL::ALValue::array(3, -0.32, 0.00306796), AL::ALValue::array(3, 0.32, -0.00306796));
  times[4][3] = 3.64;
  keys[4][3] = AL::ALValue::array(0.00460196, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

  names.push_back("LElbowRoll");
  times[5].arraySetSize(4);
  keys[5].arraySetSize(4);

  times[5][0] = 0.84;
  keys[5][0] = AL::ALValue::array(-0.520019, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.293333, 0));
  times[5][1] = 1.72;
  keys[5][1] = AL::ALValue::array(-0.520019, AL::ALValue::array(3, -0.293333, 0), AL::ALValue::array(3, 0.32, 0));
  times[5][2] = 2.68;
  keys[5][2] = AL::ALValue::array(-0.520019, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0.32, 0));
  times[5][3] = 3.64;
  keys[5][3] = AL::ALValue::array(-0.523087, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

  names.push_back("LElbowYaw");
  times[6].arraySetSize(4);
  keys[6].arraySetSize(4);

  times[6][0] = 0.84;
  keys[6][0] = AL::ALValue::array(-1.22718, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.293333, 0));
  times[6][1] = 1.72;
  keys[6][1] = AL::ALValue::array(-1.22718, AL::ALValue::array(3, -0.293333, 0), AL::ALValue::array(3, 0.32, 0));
  times[6][2] = 2.68;
  keys[6][2] = AL::ALValue::array(-1.22718, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0.32, 0));
  times[6][3] = 3.64;
  keys[6][3] = AL::ALValue::array(-1.23332, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

  names.push_back("LHand");
  times[7].arraySetSize(4);
  keys[7].arraySetSize(4);

  times[7][0] = 0.84;
  keys[7][0] = AL::ALValue::array(0.601054, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.293333, 0));
  times[7][1] = 1.72;
  keys[7][1] = AL::ALValue::array(0.601054, AL::ALValue::array(3, -0.293333, 0), AL::ALValue::array(3, 0.32, 0));
  times[7][2] = 2.68;
  keys[7][2] = AL::ALValue::array(0.601054, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0.32, 0));
  times[7][3] = 3.64;
  keys[7][3] = AL::ALValue::array(0.601054, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

  names.push_back("LShoulderPitch");
  times[8].arraySetSize(4);
  keys[8].arraySetSize(4);

  times[8][0] = 0.84;
  keys[8][0] = AL::ALValue::array(1.56619, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.293333, 0));
  times[8][1] = 1.72;
  keys[8][1] = AL::ALValue::array(1.56619, AL::ALValue::array(3, -0.293333, 0), AL::ALValue::array(3, 0.32, 0));
  times[8][2] = 2.68;
  keys[8][2] = AL::ALValue::array(1.56619, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0.32, 0));
  times[8][3] = 3.64;
  keys[8][3] = AL::ALValue::array(1.56159, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

  names.push_back("LShoulderRoll");
  times[9].arraySetSize(4);
  keys[9].arraySetSize(4);

  times[9][0] = 0.84;
  keys[9][0] = AL::ALValue::array(0.139592, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.293333, 0));
  times[9][1] = 1.72;
  keys[9][1] = AL::ALValue::array(0.139592, AL::ALValue::array(3, -0.293333, 0), AL::ALValue::array(3, 0.32, 0));
  times[9][2] = 2.68;
  keys[9][2] = AL::ALValue::array(0.139592, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0.32, 0));
  times[9][3] = 3.64;
  keys[9][3] = AL::ALValue::array(0.138058, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

  names.push_back("LWristYaw");
  times[10].arraySetSize(4);
  keys[10].arraySetSize(4);

  times[10][0] = 0.84;
  keys[10][0] = AL::ALValue::array(0.030638, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.293333, 0));
  times[10][1] = 1.72;
  keys[10][1] = AL::ALValue::array(0.030638, AL::ALValue::array(3, -0.293333, 0), AL::ALValue::array(3, 0.32, 0));
  times[10][2] = 2.68;
  keys[10][2] = AL::ALValue::array(0.030638, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0.32, 0));
  times[10][3] = 3.64;
  keys[10][3] = AL::ALValue::array(0.0106959, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

  names.push_back("RElbowRoll");
  times[11].arraySetSize(4);
  keys[11].arraySetSize(4);

  times[11][0] = 0.84;
  keys[11][0] = AL::ALValue::array(0.903515, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.293333, 0));
  times[11][1] = 1.72;
  keys[11][1] = AL::ALValue::array(0.872836, AL::ALValue::array(3, -0.293333, 0.0306794), AL::ALValue::array(3, 0.32, -0.0334685));
  times[11][2] = 2.68;
  keys[11][2] = AL::ALValue::array(0.145728, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0.32, 0));
  times[11][3] = 3.64;
  keys[11][3] = AL::ALValue::array(0.521553, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

  names.push_back("RElbowYaw");
  times[12].arraySetSize(4);
  keys[12].arraySetSize(4);

  times[12][0] = 0.84;
  keys[12][0] = AL::ALValue::array(0.319067, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.293333, 0));
  times[12][1] = 1.72;
  keys[12][1] = AL::ALValue::array(1.62295, AL::ALValue::array(3, -0.293333, -0.0998361), AL::ALValue::array(3, 0.32, 0.108912));
  times[12][2] = 2.68;
  keys[12][2] = AL::ALValue::array(1.73186, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0.32, 0));
  times[12][3] = 3.64;
  keys[12][3] = AL::ALValue::array(1.22412, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

  names.push_back("RHand");
  times[13].arraySetSize(4);
  keys[13].arraySetSize(4);

  times[13][0] = 0.84;
  keys[13][0] = AL::ALValue::array(0.98, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.293333, 0));
  times[13][1] = 1.72;
  keys[13][1] = AL::ALValue::array(0.98, AL::ALValue::array(3, -0.293333, 0), AL::ALValue::array(3, 0.32, 0));
  times[13][2] = 2.68;
  keys[13][2] = AL::ALValue::array(0.566784, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0.32, 0));
  times[13][3] = 3.64;
  keys[13][3] = AL::ALValue::array(0.59754, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

  names.push_back("RShoulderPitch");
  times[14].arraySetSize(4);
  keys[14].arraySetSize(4);

  times[14][0] = 0.84;
  keys[14][0] = AL::ALValue::array(0.0690292, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.293333, 0));
  times[14][1] = 1.72;
  keys[14][1] = AL::ALValue::array(0.231631, AL::ALValue::array(3, -0.293333, -0.162602), AL::ALValue::array(3, 0.32, 0.177384));
  times[14][2] = 2.68;
  keys[14][2] = AL::ALValue::array(1.51251, AL::ALValue::array(3, -0.32, -0.0506198), AL::ALValue::array(3, 0.32, 0.0506198));
  times[14][3] = 3.64;
  keys[14][3] = AL::ALValue::array(1.56313, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

  names.push_back("RShoulderRoll");
  times[15].arraySetSize(4);
  keys[15].arraySetSize(4);

  times[15][0] = 0.84;
  keys[15][0] = AL::ALValue::array(-0.779262, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.293333, 0));
  times[15][1] = 1.72;
  keys[15][1] = AL::ALValue::array(-1.10907, AL::ALValue::array(3, -0.293333, 0.0210928), AL::ALValue::array(3, 0.32, -0.0230104));
  times[15][2] = 2.68;
  keys[15][2] = AL::ALValue::array(-1.13208, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0.32, 0));
  times[15][3] = 3.64;
  keys[15][3] = AL::ALValue::array(-0.138058, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));

  names.push_back("RWristYaw");
  times[16].arraySetSize(4);
  keys[16].arraySetSize(4);

  times[16][0] = 0.84;
  keys[16][0] = AL::ALValue::array(1.30846, AL::ALValue::array(3, -0.28, 0), AL::ALValue::array(3, 0.293333, 0));
  times[16][1] = 1.72;
  keys[16][1] = AL::ALValue::array(0.42641, AL::ALValue::array(3, -0.293333, 0.00562518), AL::ALValue::array(3, 0.32, -0.00613656));
  times[16][2] = 2.68;
  keys[16][2] = AL::ALValue::array(0.420274, AL::ALValue::array(3, -0.32, 0.00613656), AL::ALValue::array(3, 0.32, -0.00613656));
  times[16][3] = 3.64;
  keys[16][3] = AL::ALValue::array(-0.016916, AL::ALValue::array(3, -0.32, 0), AL::ALValue::array(3, 0, 0));


  try
  {
    motion->angleInterpolationBezier(names, times, keys);
  }
  catch(const std::exception&)
  {
    std::cout << "Error during the motion" << std::endl;
    return false;
  }

  return true;
}


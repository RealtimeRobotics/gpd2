/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Luca Colasanto
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <gpd/candidate/hand_constrain.h>

using namespace std;

void test_1(string yaml_file)
{
  cout << std::endl << "Test 1: load from file. ------------------------------" << std::endl;

  cout << "Running test on: " << yaml_file << endl << endl;

  HandConstrain test_contrain(yaml_file);

  cout << "Printing... " << endl;
  test_contrain.print();
}

void test_2(HandConstrain::Parameters param)
{
  cout << std::endl << "Test 2: load from parameter. ------------------------------" << std::endl;

  HandConstrain test_constrain(param);

  cout << "Printing... " << endl;
  test_constrain.print();
}

void test_3(HandConstrain::Parameters param, gpd::candidate::Hand hand)
{
  cout << std::endl << "Test 3: hand comparison. -----------------------------------" << std::endl;

  cout << "Hand parameters:" << endl;
  cout << "position: " << hand.getPosition()[0] << " " <<
                          hand.getPosition()[1] << " " <<
                          hand.getPosition()[2] << endl;
  cout << "approach: " << hand.getApproach()[0] << " " <<
                          hand.getApproach()[1] << " " <<
                          hand.getApproach()[2] << endl;
  cout << "binormal: " << hand.getBinormal()[0] << " " <<
                          hand.getBinormal()[1] << " " <<
                          hand.getBinormal()[2] << endl;
  cout << "axis: " << hand.getAxis()[0] << " " <<
                      hand.getAxis()[1] << " " <<
                      hand.getAxis()[2] << endl << endl;

  HandConstrain test_constrain(param);
  cout << "Constrain:" << endl;
  test_constrain.print();
  cout << "Hand inside the constrain limits: " << (test_constrain.is_hand_valid(hand) ? "true" : "false") << endl << endl;

  param.position_constrained = true;
  test_constrain = HandConstrain(param);
  cout << "Constrain:" << endl;
  test_constrain.print();
  cout << "Hand inside the constrain limits: " << (test_constrain.is_hand_valid(hand) ? "true" : "false") << endl << endl;
  param.position_constrained = false;

  param.approach_constrained = true;
  test_constrain = HandConstrain(param);
  cout << "Constrain:" << endl;
  test_constrain.print();
  cout << "Hand inside the constrain limits: " << (test_constrain.is_hand_valid(hand) ? "true" : "false") << endl << endl;
  param.approach_constrained = false;

  param.binormal_constrained = true;
  test_constrain = HandConstrain(param);
  cout << "Constrain:" << endl;
  test_constrain.print();
  cout << "Hand inside the constrain limits: " << (test_constrain.is_hand_valid(hand) ? "true" : "false") << endl << endl;
  param.binormal_constrained = false;

  param.axis_constrained = true;
  test_constrain = HandConstrain(param);
  cout << "Constrain:" << endl;
  test_constrain.print();
  cout << "Hand inside the constrain limits: " << (test_constrain.is_hand_valid(hand) ? "true" : "false") << endl << endl;
  param.axis_constrained = false;

  param.position_constrained = true;
  param.approach_constrained = true;
  param.binormal_constrained = true;
  param.axis_constrained = true;
  test_constrain = HandConstrain(param);
  cout << "Constrain:" << endl;
  test_constrain.print();
  cout << "Hand inside the constrain limits: " << (test_constrain.is_hand_valid(hand) ? "true" : "false") << endl << endl;
}


int main(int argc, char* argv[]) {
  if(argc<2)
  {
    cout << "Not enough arguments for this test." << endl;
    cout << "Run:" << endl;
    cout << "./gpd_test_hand_constrain ../cfg/test_hand_consrains.yaml" << endl;
    return 1;
  }

  //TEST 1 --------------------------------------------------------------
  string yaml_file = argv[1];
  test_1(yaml_file);

  //TEST 2 --------------------------------------------------------------
  HandConstrain::Parameters param = {true,
                                     std::vector<std::vector<double>>{std::vector<double>(6, 0.0),{std::vector<double>(6, 1.0)}},
                                     false,
                                     std::vector<std::vector<double>>{std::vector<double>(6, 2.0)},
                                     true,
                                     std::vector<std::vector<double>>{std::vector<double>(6, 3.0),{std::vector<double>(6, 4.0)},std::vector<double>(6, 5.0)},
                                     false,
                                     std::vector<std::vector<double>>{}};
  test_2(param);

  //TEST 3 --------------------------------------------------------------

  Eigen::Vector3d position_vector(0.0,0.0,0.0);
  Eigen::Vector3d approach_vector(1.0,0.0,0.0);
  Eigen::Vector3d binormal_vector(0.0,0.5,0.0);
  Eigen::Vector3d axis_vector(0.0,0.0,4.0);

  Eigen::Matrix3d frame;
  frame.col(0) = approach_vector;
  frame.col(1) = binormal_vector;
  frame.col(2) = axis_vector;

  gpd::candidate::Hand hand(Eigen::Vector3d(0.0,0.0,0.0),frame,gpd::candidate::FingerHand(),0.0);
  hand.setPosition(position_vector);

  param = {false,
           std::vector<std::vector<double>>{{-1.0, 1.0, -1.0, 1.0, -1.0, 1.0 }},
           false,
           std::vector<std::vector<double>>{{-1.0, 1.0, -1.0, 1.0, -1.0, 1.0 }},
           false,
           std::vector<std::vector<double>>{{-1.0, 1.0, -1.0, 1.0, -1.0, 1.0 }},
           false,
           std::vector<std::vector<double>>{{-1.0, 1.0, -1.0, 1.0, -1.0, 1.0 }}};
  test_3(param, hand);


  param = {//limits DO NOT overlap
           false,
           std::vector<std::vector<double>>{{-2.0, -1.0, -2.0, -1.0, -2.0, -1.0 },
                                             {1.0, 2.0 ,  1.0, 2.0 ,  1.0,  2.0 }},
           //limits are coincident
           false,
           std::vector<std::vector<double>>{{-1.0, 1.0, -1.0, 1.0, -1.0, 1.0 },
                                            {-1.0, 1.0, -1.0, 1.0, -1.0, 1.0 }},
           //limits overlap
           false,
           std::vector<std::vector<double>>{{-2.0, 1.0, -2.0, 1.0, -2.0, 1.0 },
                                            {-1.0, 2.0 , -1.0, 2.0 , -1.0,  2.0 }},
           //Single constrain
           false,
           std::vector<std::vector<double>>{{-1.0, 1.0, -1.0, 1.0, -1.0, 1.0 }}};
  test_3(param, hand);

  return 0;
}

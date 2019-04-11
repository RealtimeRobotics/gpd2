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

#include <boost/lexical_cast.hpp>

#include <gpd/candidate/candidates_generator.h>
#include <gpd/candidate/hand.h>
#include <gpd/candidate/hand_geometry.h>
#include <gpd/descriptor/image_12_channels_strategy.h>
#include <gpd/descriptor/image_15_channels_strategy.h>
#include <gpd/descriptor/image_1_channels_strategy.h>
#include <gpd/descriptor/image_3_channels_strategy.h>
#include <gpd/descriptor/image_generator.h>
#include <gpd/net/classifier.h>
#include <gpd/util/config_file.h>
#include <gpd/util/plot.h>

#include <gpd/candidate/hand.h>

#include <gpd/candidate/hand_constrain.h>

#include <iostream>
/*
namespace gpd {
namespace test {
namespace {

int DoMain(int argc, char* argv[]) {
  if (argc < 4) {
    std::cout << "ERROR: Not enough arguments given!\n";
    std::cout << "Usage: rosrun gpd test_grasp_image INPUT_FILE SAMPLE_INDEX "
                 "DRAW_GRASP_IMAGES [IMAGE_CHANNELS] [HAND_AXES]\n";
    return -1;
  }

  // View point from which the camera sees the point cloud.
  Eigen::Matrix3Xd view_points(3, 1);
  view_points.setZero();

  // Load point cloud from file
  std::string filename = argv[1];
  util::Cloud cloud(filename, view_points);
  if (cloud.getCloudOriginal()->size() == 0) {
    std::cout << "Error: Input point cloud is empty or does not exist!\n";
    return (-1);
  }

  // Read the sample index from the terminal.
  int sample_idx = boost::lexical_cast<int>(argv[2]);
  if (sample_idx >= cloud.getCloudOriginal()->size()) {
    std::cout << "Error: Sample index is larger than the number of points in "
                 "the cloud!\n";
    return -1;
  }
  std::vector<int> sample_indices;
  sample_indices.push_back(sample_idx);
  cloud.setSampleIndices(sample_indices);

  // Create objects to store parameters.
  candidate::CandidatesGenerator::Parameters generator_params;
  candidate::HandSearch::Parameters hand_search_params;

  // Hand geometry parameters
  candidate::HandGeometry hand_geom;
  hand_geom.finger_width_ = 0.01;
  hand_geom.outer_diameter_ = 0.12;
  hand_geom.depth_ = 0.06;
  hand_geom.height_ = 0.02;
  hand_geom.init_bite_ = 0.015;
  hand_search_params.hand_geometry_ = hand_geom;

  // Local hand search parameters
  hand_search_params.num_samples_ = 1;
  hand_search_params.num_threads_ = 1;
  hand_search_params.nn_radius_frames_ = 0.01;
  hand_search_params.num_orientations_ = 8;
  hand_search_params.num_finger_placements_ = 10;
  hand_search_params.deepen_hand_ = true;
  hand_search_params.friction_coeff_ = 20.0;
  hand_search_params.min_viable_ = 6;

  std::vector<int> hand_axes;
  if (argc >= 6) {
    for (int i = 5; i < argc; i++) {
      hand_axes.push_back(boost::lexical_cast<int>(argv[i]));
    }
  } else {
    hand_axes.push_back(2);
  }
  hand_search_params.hand_axes_ = hand_axes;
  std::cout << "hand_axes: ";
  for (int i = 0; i < hand_search_params.hand_axes_.size(); i++) {
    std::cout << hand_search_params.hand_axes_[i] << " ";
  }
  std::cout << "\n";

  // General parameters
  generator_params.num_samples_ = hand_search_params.num_samples_;
  generator_params.num_threads_ = hand_search_params.num_threads_;

  // Preprocessing parameters
  generator_params.remove_statistical_outliers_ = false;
  generator_params.voxelize_ = false;
  generator_params.workspace_.resize(6);
  generator_params.workspace_[0] = -1.0;
  generator_params.workspace_[1] = 1.0;
  generator_params.workspace_[2] = -1.0;
  generator_params.workspace_[3] = 1.0;
  generator_params.workspace_[4] = -1.0;
  generator_params.workspace_[5] = 1.0;

  // Image parameters
  descriptor::ImageGeometry image_geom;
  image_geom.depth_ = 0.06;
  image_geom.height_ = 0.02;
  image_geom.outer_diameter_ = 0.12;
  image_geom.size_ = 60;
  image_geom.num_channels_ = 15;
  if (argc >= 5) {
    image_geom.num_channels_ = boost::lexical_cast<int>(argv[4]);
  }

  // Calculate surface normals.
  cloud.calculateNormals(4);
  cloud.setNormals(cloud.getNormals() * (-1.0));

  // Plot the normals.
  util::Plot plot(hand_search_params.hand_axes_.size(),
                  hand_search_params.num_orientations_);
  plot.plotNormals(cloud.getCloudProcessed(), cloud.getNormals());

  // Generate grasp candidates.
  candidate::CandidatesGenerator candidates_generator(generator_params,
                                                      hand_search_params);
  std::vector<std::unique_ptr<candidate::HandSet>> hand_set_list =
      candidates_generator.generateGraspCandidateSets(cloud);
  if (hand_set_list.size() == 0) {
    return -1;
  }

  plot.plotFingers3D(hand_set_list, cloud.getCloudProcessed(),
                     "Grasp candidates", hand_geom);

  const candidate::Hand& hand = *hand_set_list[0]->getHands()[0];
  std::cout << "sample: " << hand.getSample().transpose() << std::endl;
  std::cout << "grasp orientation:\n" << hand.getFrame() << std::endl;
  std::cout << "grasp position: " << hand.getPosition().transpose()
            << std::endl;

  // Create the image for this grasp candidate.
  bool plot_images = true;
  plot_images = boost::lexical_cast<bool>(argv[3]);
  printf("Creating grasp image ...\n");
  printf("plot images: %d\n", plot_images);
  const int kNumThreads = 1;
  descriptor::ImageGenerator learn(image_geom, kNumThreads,
                                   hand_search_params.num_orientations_,
                                   plot_images, false);
  std::vector<std::unique_ptr<cv::Mat>> images;
  std::vector<std::unique_ptr<candidate::Hand>> hands;
  learn.createImages(cloud, hand_set_list, images, hands);

  // Evaluate if the grasp candidates are antipodal.
  std::cout << "Antipodal: ";
  for (int i = 0; i < hands.size(); i++) {
    std::cout << hands[i]->isFullAntipodal() << " ";
  }
  std::cout << "\n";

  plot.plotVolumes3D(
      hands, cloud.getCloudProcessed(), "Volumes", hand_geom.outer_diameter_,
      hand_geom.finger_width_, hand_geom.depth_, hand_geom.height_,
      image_geom.outer_diameter_, image_geom.depth_, image_geom.height_);

  return 0;
}

}  // namespace
}  // namespace test
}  // namespace gpd

int main(int argc, char* argv[]) { return gpd::test::DoMain(argc, argv); }
*/

using namespace std;

void test_1(string yaml_file)
{
  cout << "Test 1: load from file. ------------------------------" << std::endl;

  cout << "Running test on: " << yaml_file << endl << endl;

  HandConstrain test_contrain(yaml_file);

  cout << "Printing... " << endl;
  test_contrain.print();
}

void test_2(HandConstrain::Parameters param)
{
  cout << "Test 2: load from parameter. ------------------------------" << std::endl;

  HandConstrain test_constrain(param);

  cout << "Printing... " << endl;
  test_constrain.print();
}

void test_3(HandConstrain::Parameters param, gpd::candidate::Hand hand)
{
  cout << "Test 3: hand comparison. -----------------------------------" << std::endl;

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

  string yaml_file = argv[1];
  test_1(yaml_file);


  HandConstrain::Parameters param = {true,
                                     std::vector<double>(6, 0.0),
                                     false,
                                     std::vector<double>(6, 1.0),
                                     true,
                                     std::vector<double>(6, 2.0),
                                     false,
                                     std::vector<double>(6, 3.0)};
  test_2(param);


  param = {false,
           std::vector<double>{-1.0, 1.0, -1.0, 1.0, -1.0, 1.0 },
           false,
           std::vector<double>{-1.0, 1.0, -1.0, 1.0, -1.0, 1.0 },
           false,
           std::vector<double>{-1.0, 1.0, -1.0, 1.0, -1.0, 1.0 },
           false,
           std::vector<double>{-1.0, 1.0, -1.0, 1.0, -1.0, 1.0 }};
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

  test_3(param, hand);


  return 0;
}

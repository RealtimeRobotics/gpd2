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

#ifndef HAND_CONSTRAIN_H
#define HAND_SEARCH_H

#include <boost/filesystem.hpp>
#include "yaml-cpp/yaml.h"

//TODO clean includes
#include <Eigen/Dense>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/random_sample.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>

#include <omp.h>

#include <memory>

#include <gpd/candidate/antipodal.h>
#include <gpd/candidate/finger_hand.h>
#include <gpd/candidate/frame_estimator.h>
#include <gpd/candidate/hand.h>
#include <gpd/candidate/hand_geometry.h>
#include <gpd/candidate/hand_set.h>
#include <gpd/candidate/local_frame.h>
#include <gpd/util/plot.h>
#include <gpd/util/point_list.h>

/**
 *
 * \brief Define nd check hand position/orientation constrain.
 *
 * TODO: add long description
 *
 */
class HandConstrain {
 public:
  /**
   * \brief Parameters for the hand constrian.
   */
  struct Parameters {
    bool approach_constrained;             ///< on/off the limits on the hand Approach vector
    std::vector<double> approach_limits;   ///< x, y, z limits of the hand Approach 3d vector

    bool binormal_constrained;             ///< on/off the limits on the hand Binormal vector
    std::vector<double> binormal_limits;   ///< x, y, z limits of the hand Binormal 3d vector

    bool axis_constrained;                 ///< on/off the limits on the hand Axis vector
    std::vector<double> axis_limits;       ///< x, y, z limits of the hand Axis 3d vector

    bool position_constrained;             ///< on/off the limits on the hand position vector
    std::vector<double> position_limits;   ///< x, y, z limits of the hand position 3d vector

  };

  /**
   * \brief Constructor.
   * \param params parameters for the hand constrain
   */
  HandConstrain(Parameters params);

  /**
   * \brief Constructor.
   * \param yaml_file load parameters for the hand constrain
   */
  HandConstrain(std::string yaml_file );

  /**
   * \brief Constructor.
   */
  HandConstrain();

  /**
   * \brief Check if hand respect the constrains.
   * \param hand the candidate to check
   * \return true if the hand is inside the limits
   */
  bool is_hand_valid(const gpd::candidate::Hand &hand) const;

  /**
   * \brief Score the proximity of the hand to the constrain
   * \note Not Implemented yet
   * \param hand the candidate to check
   * \return proximity score
   */
  std::vector<double> score_hand(const gpd::candidate::Hand &hand) const;

  /**
   * \brief Return the parameters for the hand constrain.
   * \return params the hand constrain parameters
   */
  const Parameters& getParams() const { return params_; }

  /**
   * \brief Set the parameters for the hand constrain.
   * \param params the hand constrain parameters
   */
  void setParameters(const Parameters& params) { params_ = params; }

  /**
   * \brief Print the limits.
   */
  void print();


 private:

  Parameters params_;  ///< parameters for the hand constrain

};

#endif /* HAND_CONSTRAIN_H */

#include <gpd/candidate/hand_constrain.h>

HandConstrain::HandConstrain(Parameters params)
    : params_(params) {}

HandConstrain::HandConstrain(){
  params_.approach_constrained = false;
  params_.approach_limits.assign(6, 0.0);

  params_.binormal_constrained = false;
  params_.binormal_limits.assign(6, 0.0);

  params_.axis_constrained = false;
  params_.axis_limits.assign(6, 0.0);

  params_.position_constrained = false;
  params_.position_limits.assign(6, 0.0);
}

HandConstrain::HandConstrain(std::string yaml_file):
  HandConstrain(){
  if (boost::filesystem::exists(yaml_file))    // does path p actually exist?
  {
    YAML::Node base_node = YAML::LoadFile(yaml_file);
    if(base_node["constrains"])
    {
      short int N_constrains = base_node["constrains"]["constrain_names"].size();
      for (unsigned short i = 0; i < N_constrains; ++i) {
        std::string cur_constrain_name = base_node["constrains"]["constrain_names"].as<std::vector<std::string>>().at(i);
        YAML::Node cur_constrain_node = base_node[ cur_constrain_name ];

        if(cur_constrain_node["approach_limits"])
        {
          unsigned short totalFrames = cur_constrain_node["approach_limits"].size();
          for (unsigned short f = 0; f < totalFrames; ++f)
            params_.approach_limits.push_back(cur_constrain_node["approach_limits"].as<std::vector<double>>().at(f));
        }

        if(cur_constrain_node["binormal_limits"])
        {
          unsigned short totalFrames = cur_constrain_node["binormal_limits"].size();
          for (unsigned short f = 0; f < totalFrames; ++f)
            params_.binormal_limits.push_back(cur_constrain_node["binormal_limits"].as<std::vector<double>>().at(f));
        }

        if(cur_constrain_node["axis_limits"])
        {
          unsigned short totalFrames = cur_constrain_node["axis_limits"].size();
          for (unsigned short f = 0; f < totalFrames; ++f)
            params_.axis_limits.push_back(cur_constrain_node["axis_limits"].as<std::vector<double>>().at(f));
        }

        if(cur_constrain_node["approach_limits"])
        {
          unsigned short totalFrames = cur_constrain_node["position_limits"].size();
          for (unsigned short f = 0; f < totalFrames; ++f)
            params_.position_limits.push_back(cur_constrain_node["position_limits"].as<std::vector<double>>().at(f));
        }
      }
    }
  }
  else
    cout << yaml_file << " does not exist\n";
}


bool HandConstrain::is_hand_valid(const gpd::candidate::Hand &hand) const{
  if (!(params_.approach_constrained || params_.binormal_constrained ||
      params_.axis_constrained || params_.position_constrained ))
    return false;

  if (params_.approach_constrained)
  {
    Eigen::Vector3d hand_approach = hand.getApproach();
    if (hand_approach[0] < params_.approach_limits[0] || hand_approach[0] > params_.approach_limits[1])
        return false;
    if (hand_approach[1] < params_.approach_limits[2] || hand_approach[1] > params_.approach_limits[3])
        return false;
    if (hand_approach[2] < params_.approach_limits[4] || hand_approach[2] > params_.approach_limits[5])
        return false;
  }

  if (params_.binormal_constrained)
  {
    Eigen::Vector3d hand_binormal = hand.getBinormal();
    if (hand_binormal[0] < params_.binormal_limits[0] || hand_binormal[0] > params_.binormal_limits[1])
        return false;
    if (hand_binormal[1] < params_.binormal_limits[2] || hand_binormal[1] > params_.binormal_limits[3])
        return false;
    if (hand_binormal[2] < params_.binormal_limits[4] || hand_binormal[2] > params_.binormal_limits[5])
        return false;
  }

  if (params_.axis_constrained)
  {
    Eigen::Vector3d hand_axis = hand.getAxis();
    if (hand_axis[0] < params_.axis_limits[0] || hand_axis[0] > params_.axis_limits[1])
        return false;
    if (hand_axis[1] < params_.axis_limits[2] || hand_axis[1] > params_.axis_limits[3])
        return false;
    if (hand_axis[2] < params_.axis_limits[4] || hand_axis[2] > params_.axis_limits[5])
        return false;
  }

  if (params_.position_constrained)
  {
    Eigen::Vector3d hand_position = hand.getPosition();
    if (hand_position[0] < params_.position_limits[0] || hand_position[0] > params_.position_limits[1])
        return false;
    if (hand_position[1] < params_.position_limits[2] || hand_position[1] > params_.position_limits[3])
        return false;
    if (hand_position[2] < params_.position_limits[4] || hand_position[2] > params_.position_limits[5])
        return false;
  }

  return true;
}

std::vector<double> HandConstrain::score_hand(const gpd::candidate::Hand &hand) const{
  std::vector<double> return_vect;
  return return_vect;
}

void HandConstrain::print(){
  if (!(params_.approach_constrained || params_.binormal_constrained ||
      params_.axis_constrained || params_.position_constrained ))
  {
    std::cout << "Empty constrain." << std::endl;
    return;
  }

  std::cout << "Approach_limits[" << (params_.approach_constrained? "true":"false") << "]" << std::endl;
  std::cout << "[";
  for (std::vector<double>::const_iterator it = params_.approach_limits.begin(); it!= params_.approach_limits.end(); it++)
    std::cout <<" " << std::to_string(*it);
  std::cout << " ]" << std::endl;

  std::cout << "Binormal_limits[" << (params_.binormal_constrained? "true":"false") << "]" << std::endl;
  std::cout << "[";
  for (std::vector<double>::const_iterator it = params_.binormal_limits.begin(); it!= params_.binormal_limits.end(); it++)
    std::cout <<" " << std::to_string(*it);
  std::cout << " ]" << std::endl;

  std::cout << "Axis_limits[" << (params_.axis_constrained? "true":"false") << "]" << std::endl;
  std::cout << "[";
  for (std::vector<double>::const_iterator it = params_.axis_limits.begin(); it!= params_.axis_limits.end(); it++)
    std::cout <<" " << std::to_string(*it);
  std::cout << " ]" << std::endl;

  std::cout << "Position_limits[" << (params_.position_constrained? "true":"false") << "]" << std::endl;
  std::cout << "[";
  for (std::vector<double>::const_iterator it = params_.position_limits.begin(); it!= params_.position_limits.end(); it++)
    std::cout <<" " << std::to_string(*it);
  std::cout << " ]" << std::endl;
}

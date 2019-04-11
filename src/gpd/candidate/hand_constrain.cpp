#include <gpd/candidate/hand_constrain.h>


HandConstrain::HandConstrain(){
  params_.position_constrained = false;
  params_.position_limits.assign(6, 0.0);

  params_.approach_constrained = false;
  params_.approach_limits.assign(6, 0.0);

  params_.binormal_constrained = false;
  params_.binormal_limits.assign(6, 0.0);

  params_.axis_constrained = false;
  params_.axis_limits.assign(6, 0.0);
}


HandConstrain::HandConstrain(Parameters params)
    : params_(params) {}


HandConstrain::HandConstrain(std::string yaml_file)
  : HandConstrain()
{
  if (boost::filesystem::exists(yaml_file))
  {
    YAML::Node base_node = YAML::LoadFile(yaml_file);
    if(base_node["constrains"])
    {
      short int N_constrains = base_node["constrains"]["constrain_names"].size();
      //std::cout << N_constrains << std::endl;
      for (unsigned short i = 0; i < N_constrains; ++i) {
        std::string cur_constrain_name = base_node["constrains"]["constrain_names"].as<std::vector<std::string>>().at(i);
        //std::cout << cur_constrain_name << std::endl;
        YAML::Node cur_constrain_node = base_node["constrains"][ cur_constrain_name ];

        if(cur_constrain_node["position_limits"])
        {
          params_.position_constrained = true;

          unsigned short N_limits = cur_constrain_node["position_limits"].size();
          params_.position_limits.clear();
          for (unsigned short j = 0; j < N_limits; j++)
          {
            params_.position_limits.push_back(cur_constrain_node["position_limits"].as<std::vector<double>>().at(j));
          }
          //std::cout << params_.approach_limits.size() << std::endl;
        }

        if(cur_constrain_node["approach_limits"])
        {
          params_.approach_constrained = true;

          unsigned short N_limits = cur_constrain_node["approach_limits"].size();
          params_.approach_limits.clear();
          for (unsigned short j = 0; j < N_limits; j++)
          {
            params_.approach_limits.push_back(cur_constrain_node["approach_limits"].as<std::vector<double>>().at(j));
          }
          //std::cout << params_.approach_limits.size() << std::endl;
        }

        if(cur_constrain_node["binormal_limits"])
        {
          params_.binormal_constrained = true;

          unsigned short N_limits = cur_constrain_node["binormal_limits"].size();
          params_.binormal_limits.clear();
          for (unsigned short j = 0; j < N_limits; j++)
          {
            params_.binormal_limits.push_back(cur_constrain_node["binormal_limits"].as<std::vector<double>>().at(j));
          }
          //std::cout << params_.approach_limits.size() << std::endl;
        }

        if(cur_constrain_node["axis_limits"])
        {
          params_.axis_constrained = true;

          unsigned short N_limits = cur_constrain_node["axis_limits"].size();
          params_.axis_limits.clear();
          for (unsigned short j = 0; j < N_limits; j++)
          {
            params_.axis_limits.push_back(cur_constrain_node["axis_limits"].as<std::vector<double>>().at(j));
          }
          //std::cout << params_.approach_limits.size() << std::endl;
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
  /*if (!(params_.approach_constrained || params_.binormal_constrained ||
      params_.axis_constrained || params_.position_constrained ))
  {
    std::cout << "Empty constrain." << std::endl;
    return;
  }*/

  std::cout << "Position_limits[" << (params_.position_constrained? "true":"false") << "]" << std::endl;
  std::cout << "[";
  for (std::vector<double>::const_iterator it = params_.position_limits.begin(); it!= params_.position_limits.end(); it++)
    std::cout <<" " << std::to_string(*it);
  std::cout << " ]" << std::endl;

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

}

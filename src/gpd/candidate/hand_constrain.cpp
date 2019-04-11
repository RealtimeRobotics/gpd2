#include <gpd/candidate/hand_constrain.h>

HandConstrain::HandConstrain(){
  params_.position_constrained = false;
  params_.approach_constrained = false;
  params_.binormal_constrained = false;
  params_.axis_constrained = false;
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
          std::vector<double> new_limit;
          for (unsigned short j = 0; j < N_limits; j++)
          {
            new_limit.push_back(cur_constrain_node["position_limits"].as<std::vector<double>>().at(j));
          }
          params_.position_limits.push_back(new_limit);
          //std::cout << params_.approach_limits.size() << std::endl;
        }

        if(cur_constrain_node["approach_limits"])
        {
          params_.approach_constrained = true;

          unsigned short N_limits = cur_constrain_node["approach_limits"].size();
          std::vector<double> new_limit;
          for (unsigned short j = 0; j < N_limits; j++)
          {
            new_limit.push_back(cur_constrain_node["approach_limits"].as<std::vector<double>>().at(j));
          }
          params_.approach_limits.push_back(new_limit);
          //std::cout << params_.approach_limits.size() << std::endl;
        }

        if(cur_constrain_node["binormal_limits"])
        {
          params_.binormal_constrained = true;

          unsigned short N_limits = cur_constrain_node["binormal_limits"].size();
          std::vector<double> new_limit;
          for (unsigned short j = 0; j < N_limits; j++)
          {
            new_limit.push_back(cur_constrain_node["binormal_limits"].as<std::vector<double>>().at(j));
          }
          params_.binormal_limits.push_back(new_limit);
          //std::cout << params_.approach_limits.size() << std::endl;
        }

        if(cur_constrain_node["axis_limits"])
        {
          params_.axis_constrained = true;

          unsigned short N_limits = cur_constrain_node["axis_limits"].size();
          std::vector<double> new_limit;
          for (unsigned short j = 0; j < N_limits; j++)
          {
            new_limit.push_back(cur_constrain_node["axis_limits"].as<std::vector<double>>().at(j));
          }
          params_.axis_limits.push_back(new_limit);
          //std::cout << params_.approach_limits.size() << std::endl;
        }
      }
    }
  }
  else
    std::cout << yaml_file << " does not exist\n";
}


bool HandConstrain::is_hand_valid(const gpd::candidate::Hand &hand) const{
  if (!(params_.approach_constrained || params_.binormal_constrained ||
      params_.axis_constrained || params_.position_constrained ))
    return false;

  if (params_.position_constrained)
  {
    Eigen::Vector3d hand_vector = hand.getPosition();

    bool UNION_X_limits = false;
    bool UNION_Y_limits = false;
    bool UNION_Z_limits = false;
    for(std::vector<std::vector<double>>::const_iterator it_lim = params_.position_limits.begin();
        it_lim!= params_.position_limits.end(); it_lim++)
    {
      UNION_X_limits = UNION_X_limits || (hand_vector[0] >= (*it_lim)[0] && hand_vector[0] <= (*it_lim)[1]);
      UNION_Y_limits = UNION_Y_limits || (hand_vector[1] >= (*it_lim)[2] && hand_vector[1] <= (*it_lim)[3]);
      UNION_Z_limits = UNION_Z_limits || (hand_vector[2] >= (*it_lim)[4] && hand_vector[2] <= (*it_lim)[5]);
    }

    if ((UNION_X_limits && UNION_Y_limits && UNION_Z_limits) == false)
      return false;
  }

  if (params_.approach_constrained)
  {
    Eigen::Vector3d hand_vector = hand.getApproach();

    bool UNION_X_limits = false;
    bool UNION_Y_limits = false;
    bool UNION_Z_limits = false;
    for(std::vector<std::vector<double>>::const_iterator it_lim = params_.approach_limits.begin();
        it_lim!= params_.approach_limits.end(); it_lim++)
    {
      UNION_X_limits = UNION_X_limits || (hand_vector[0] >= (*it_lim)[0] && hand_vector[0] <= (*it_lim)[1]);
      UNION_Y_limits = UNION_Y_limits || (hand_vector[1] >= (*it_lim)[2] && hand_vector[1] <= (*it_lim)[3]);
      UNION_Z_limits = UNION_Z_limits || (hand_vector[2] >= (*it_lim)[4] && hand_vector[2] <= (*it_lim)[5]);
    }

    if ((UNION_X_limits && UNION_Y_limits && UNION_Z_limits) == false)
      return false;
  }

  if (params_.binormal_constrained)
  {
    Eigen::Vector3d hand_vector = hand.getBinormal();

    bool UNION_X_limits = false;
    bool UNION_Y_limits = false;
    bool UNION_Z_limits = false;
    for(std::vector<std::vector<double>>::const_iterator it_lim = params_.binormal_limits.begin();
        it_lim!= params_.binormal_limits.end(); it_lim++)
    {
      UNION_X_limits = UNION_X_limits || (hand_vector[0] >= (*it_lim)[0] && hand_vector[0] <= (*it_lim)[1]);
      UNION_Y_limits = UNION_Y_limits || (hand_vector[1] >= (*it_lim)[2] && hand_vector[1] <= (*it_lim)[3]);
      UNION_Z_limits = UNION_Z_limits || (hand_vector[2] >= (*it_lim)[4] && hand_vector[2] <= (*it_lim)[5]);
    }

    if ((UNION_X_limits && UNION_Y_limits && UNION_Z_limits) == false)
      return false;
  }

  if (params_.axis_constrained)
  {
    Eigen::Vector3d hand_vector = hand.getAxis();

    bool UNION_X_limits = false;
    bool UNION_Y_limits = false;
    bool UNION_Z_limits = false;
    for(std::vector<std::vector<double>>::const_iterator it_lim = params_.axis_limits.begin();
        it_lim!= params_.axis_limits.end(); it_lim++)
    {
      UNION_X_limits = UNION_X_limits || (hand_vector[0] >= (*it_lim)[0] && hand_vector[0] <= (*it_lim)[1]);
      UNION_Y_limits = UNION_Y_limits || (hand_vector[1] >= (*it_lim)[2] && hand_vector[1] <= (*it_lim)[3]);
      UNION_Z_limits = UNION_Z_limits || (hand_vector[2] >= (*it_lim)[4] && hand_vector[2] <= (*it_lim)[5]);
    }

    if ((UNION_X_limits && UNION_Y_limits && UNION_Z_limits) == false)
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

  std::cout << "Position_limits: [" << (params_.position_constrained? "true":"false") << "]" << std::endl;
  for (std::vector<std::vector<double>>::const_iterator it_lim = params_.position_limits.begin(); it_lim!= params_.position_limits.end(); it_lim++)
  {
    std::cout << "[";
    for (std::vector<double>::const_iterator it_elem = (*it_lim).begin(); it_elem!= (*it_lim).end(); it_elem++)
      std::cout <<" " << std::to_string(*it_elem);
    std::cout << " ]" << std::endl;
  }

  std::cout << "Approach_limits: [" << (params_.approach_constrained? "true":"false") << "]" << std::endl;
  for (std::vector<std::vector<double>>::const_iterator it_lim = params_.approach_limits.begin(); it_lim!= params_.approach_limits.end(); it_lim++)
  {
    std::cout << "[";
    for (std::vector<double>::const_iterator it_elem = (*it_lim).begin(); it_elem!= (*it_lim).end(); it_elem++)
      std::cout <<" " << std::to_string(*it_elem);
    std::cout << " ]" << std::endl;
  }

  std::cout << "Binormal_limits: [" << (params_.binormal_constrained? "true":"false") << "]" << std::endl;
  for (std::vector<std::vector<double>>::const_iterator it_lim = params_.binormal_limits.begin(); it_lim!= params_.binormal_limits.end(); it_lim++)
  {
    std::cout << "[";
    for (std::vector<double>::const_iterator it_elem = (*it_lim).begin(); it_elem!= (*it_lim).end(); it_elem++)
      std::cout <<" " << std::to_string(*it_elem);
    std::cout << " ]" << std::endl;
  }

  std::cout << "Axis_limits: [" << (params_.axis_constrained? "true":"false") << "]" << std::endl;
  for (std::vector<std::vector<double>>::const_iterator it_lim = params_.axis_limits.begin(); it_lim!= params_.axis_limits.end(); it_lim++)
  {
    std::cout << "[";
    for (std::vector<double>::const_iterator it_elem = (*it_lim).begin(); it_elem!= (*it_lim).end(); it_elem++)
      std::cout <<" " << std::to_string(*it_elem);
    std::cout << " ]" << std::endl;
  }

}

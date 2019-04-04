#include <string>

#include <gpd/grasp_detector.h>

namespace gpd {
namespace apps {
namespace detect_grasps {

bool checkFileExists(const std::string &file_name) {
  std::ifstream file;
  file.open(file_name.c_str());
  if (!file) {
    std::cout << "File " + file_name + " could not be found!\n";
    return false;
  }
  file.close();
  return true;
}

// function to read in a double array from a single line of a configuration file
std::vector<double> stringToDouble(const std::string &str) {
  std::vector<double> values;
  std::stringstream ss(str);
  double v;

  while (ss >> v) {
    values.push_back(v);
    if (ss.peek() == ' ') {
      ss.ignore();
    }
  }

  return values;
}

int DoMain(int argc, char *argv[]) {
  // Read arguments from command line.
  if (argc < 3) {
    std::cout << "Error: Not enough input arguments!\n\n";
    std::cout << "Usage: detect_grasps CONFIG_FILE PCD_FILE [NORMALS_FILE]\n\n";
    std::cout << "Detect grasp poses for a point cloud, PCD_FILE (*.pcd), "
                 "using parameters from CONFIG_FILE (*.cfg).\n\n";
    std::cout << "[NORMALS_FILE] (optional) contains a surface normal for each "
                 "point in the cloud (*.csv).\n";
    return (-1);
  }

  std::string config_filename = argv[1];
  std::string pcd_filename = argv[2];
  if (!checkFileExists(config_filename)) {
    printf("Error: config file not found!\n");
    return (-1);
  }
  if (!checkFileExists(pcd_filename)) {
    printf("Error: PCD file not found!\n");
    return (-1);
  }

  // View point from which the camera sees the point cloud.
  Eigen::Matrix3Xd view_points(3, 1);
  view_points.setZero();

  //Load extra parameters:
  util::ConfigFile config_file(config_filename);
  config_file.ExtractKeys();
  bool load_pointcloud_cfg = config_file.getValueOfKey<bool>("load_pointcloud_cfg", false);

  if(load_pointcloud_cfg)
  {
    boost::filesystem::path pc_cfg_filename(pcd_filename);
    pc_cfg_filename.replace_extension( boost::filesystem::path(".cfg"));
    util::ConfigFile pc_cfg_file(pc_cfg_filename.string());
    pc_cfg_file.ExtractKeys();

    std::cout << "============ " << pc_cfg_filename.filename() << " ============\n";

    //Update view_points
    if(pc_cfg_file.keyExists("camera_position"))
    {
      std::string camera_position_str = pc_cfg_file.getValueOfKeyAsString("camera_position", "0.0 0.0 0.0");
      std::vector<double> camera_position = stringToDouble(camera_position_str);
      view_points(0,0) = camera_position[0];
      view_points(1,0) = camera_position[1];
      view_points(2,0) = camera_position[2];
      std::cout << "camera_position: " << camera_position_str << "\n";
    }

    std::cout << "==============================================\n";
  }

  // Load point cloud from file
  util::Cloud cloud(pcd_filename, view_points);
  if (cloud.getCloudOriginal()->size() == 0) {
    std::cout << "Error: Input point cloud is empty or does not exist!\n";
    return (-1);
  }

  //Filtering nan and inf
  std::vector<double> cloud_limits{std::numeric_limits<double>::lowest(),std::numeric_limits<double>::max(),
                                   std::numeric_limits<double>::lowest(),std::numeric_limits<double>::max(),
                                   std::numeric_limits<double>::lowest(),std::numeric_limits<double>::max()};
  cloud.filterWorkspace(cloud_limits);
  printf("Cloud without nan/inf: %zu points\n", cloud.getCloudProcessed()->size());


  // Load surface normals from file.
  if (argc > 3) {
    std::string normals_filename = argv[3];
    cloud.setNormalsFromFile(normals_filename);
    std::cout << "Loaded surface normals from file: " << normals_filename
              << "\n";
  }

  GraspDetector detector(config_filename);

  // Prepare the point cloud.
  detector.preprocessPointCloud(cloud);

  // Detect grasp poses.
  detector.detectGrasps(cloud);

  return 0;
}

}  // namespace detect_grasps
}  // namespace apps
}  // namespace gpd

int main(int argc, char *argv[]) {
  return gpd::apps::detect_grasps::DoMain(argc, argv);
}

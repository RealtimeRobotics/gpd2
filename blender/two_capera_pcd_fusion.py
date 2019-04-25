import sys

import numpy as np

import pypcd

from shutil import copyfile

def concatenate_pcd(pcd_1, pcd_2, pcd_out):
  pc_1 = pypcd.PointCloud.from_path(pcd_1)
  pc_2 = pypcd.PointCloud.from_path(pcd_2)
  pc_conc = pypcd.cat_point_clouds(pc_1, pc_2)
  pc_conc.save_pcd(pcd_out, compression='binary_compressed')



if __name__ == '__main__': 
  if len(sys.argv)<3:
    print "Not enough arguments."
    print "usage:"
    print "python two_capera_pcd_fusion.py pcd_file_prefix N_view"
    quit()
  else:
    # parameters
    in_filename_prefix = sys.argv[1]
    N_views = int(sys.argv[2])

    in_filename_suffixes = ['a.pcd', 'b.pcd']

    out_filename_suffix = '.pcd'

    # loop 
    for i_view in range(1,N_views+1):
      print("Processing view: " + str(i_view))
  
      # concatenate the two camera views
      pcd_1 = in_filename_prefix + str(i_view) + in_filename_suffixes[0]
      pcd_2 = in_filename_prefix + str(i_view) + in_filename_suffixes[1]
      pcd_out = in_filename_prefix + str(i_view) + out_filename_suffix
      concatenate_pcd(pcd_1, pcd_2, pcd_out)

      # generated fake camera view cfg
      in_cfg = in_filename_prefix + str(i_view) + "a.cfg"
      out_cfg = in_filename_prefix + str(i_view) + ".cfg"
      copyfile(in_cfg, out_cfg)


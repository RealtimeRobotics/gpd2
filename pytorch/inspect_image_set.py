# Use tensors to speed up loading data onto the GPU during training.

import h5py
import numpy as np
import sys,os
import cv2

import matplotlib.pyplot as plt

import random

def load_dataset(file_path):
  with h5py.File(file_path, 'r') as h5_file:
    imgs_ = np.array(h5_file.get('images'))
    labels_ = np.array(h5_file.get('labels'))

  print str(len(imgs_)) + " image sets loaded"
  print str(len(imgs_)) + " lables loaded"
  
  return (imgs_,labels_)

def plot_image(img):
  plt.imshow(img)
  plt.show()
  cv2.waitKey(0)
  cv2.destroyAllWindows()

def plot_image_set(img_set, title, ch_names):
  img_size = len(img_set)
  img_chanels = len(img_set[0][0])

  fig = plt.figure(figsize=(img_size, img_size))
  fig.text(0.5, 0.95, title, ha='center', fontsize=30)
  columns = 4
  rows = img_chanels // columns
  if (img_chanels % columns > 0):
    rows += 1

  for i in range(0, img_chanels):
    #print i
    fig.add_subplot(rows, columns, i+1)
    plt.imshow(img_set[:,:,i], cmap='gray', vmin=0, vmax=255)
    if len(ch_names) ==0:
      fig_title = 'ch_' + str(i)
    else:
      fig_title = str(ch_names[i]) + '[ch_' + str(i) + ']'
    plt.title(fig_title, fontsize=20)
  plt.show(block=False)
  #cv2.waitKey(0)
  #cv2.destroyAllWindows()

def is_image_set_empty(img_set):
  img_size = len(img_set)
  img_chanels = len(img_set[0][0])
  
  #we check just 1 channel to make this check quicker
  for i in range(0, img_chanels):
  #for i in range(0, 1):
    img = img_set[:,:,i]
    img = np.reshape(img,img_size*img_size)

    for j in range(0, len(img)):
      if img[j] > 0:
        return False
  return True

if __name__ == '__main__': 
#PARAMS:
  if len(sys.argv)<3:
    print "Not enough arguments."
    print "usage:"
    print "python inspect_image_set.py h5_file N_random_indexes [plot_positive_grasps:{True|False}] [plot_negatve_grasps={True|False}]"
    quit()
  else:
    h5_file_ = str(sys.argv[1])
    N_selected_indexes_ = int(sys.argv[2])
    plot_positive_selected_indexes_ = True
    plot_negative_selected_indexes_ = True
    
    if len(sys.argv)>=4:
      if sys.argv[3]=="True":
        plot_positive_selected_indexes_ = True
      else:
        plot_positive_selected_indexes_ = False

    if len(sys.argv)==5:
      if sys.argv[4]=="True":
        plot_negative_selected_indexes_ = True
      else:
        plot_negative_selected_indexes_ = False

  print "h5_file: "+ str(h5_file_)   
  print "N_selected_indexes_: "+ str(N_selected_indexes_)
  print "plot_positive_selected_indexes_: "+ str(plot_positive_selected_indexes_)
  print "plot_negative_selected_indexes_: "+ str(plot_negative_selected_indexes_)
  print "-----------------------------------------------------------------------"
  #h5_file_ = "/home/luca/gpd_data/330ml_can/test_google.h5"
  #N_selected_indexes_ = 100
  #plot_positive_selected_indexes_ = False
  #plot_negative_selected_indexes_ = True

#Load archive
  (imgs_,labels_) = load_dataset(h5_file_)

  positive_indexes = []
  negative_indexes = []
  for i in range(0, len(labels_)):
    if is_image_set_empty(imgs_[i]):
      break
    if labels_[i]==1:
      positive_indexes.append(i)
    else:
      negative_indexes.append(i)
  print str(len(positive_indexes)) + " positive grasps"
  print str(len(negative_indexes)) + " negative grasps"

  print "img size: " + str(len(imgs_[0][0]))
  print "number of channels per img: " + str(len(imgs_[0][0][0]))

  if len(imgs_[0][0][0])==12:
    channel_names=['avg_normal_X',
                   'avg_normal_Y',
                   'avg_normal_Z',
                   'avg_depth',
                   'avg_normal_X',
                   'avg_normal_Y',
                   'avg_normal_Z',
                   'avg_depth',
                   'avg_normal_X',
                   'avg_normal_Y',
                   'avg_normal_Z',
                   'avg_depth']
  else:
    channel_names = []


#Generating random indexes
  rnd_positive_indexes = []
  rnd_negative_indexes = []
  for i in range(0, N_selected_indexes_):
    rnd_positive_indexes.append(positive_indexes[random.randint(0, len(positive_indexes)-1)]) 
    rnd_negative_indexes.append(negative_indexes[random.randint(0, len(negative_indexes)-1)]) 
  print "selected negative grasps :"
  print rnd_negative_indexes
  print "selected positive grasps :"
  print rnd_positive_indexes


#Plotting
  print "Plotting..."
  if plot_positive_selected_indexes_:
    for i in range(0, N_selected_indexes_):
      cur_index = rnd_positive_indexes[i]
      image_title = "POSITIVE GRASP ["+str(cur_index)+"]"
      plot_image_set(imgs_[cur_index],image_title,channel_names)
      #print "is image_set[" + str(cur_index) + "] enpty: " + str(is_image_set_empty(imgs_[cur_index]))

  if plot_negative_selected_indexes_:
    for i in range(0, N_selected_indexes_):
      cur_index = rnd_negative_indexes[i]
      image_title = "NEGITIVE GRASP [" + str(cur_index) + "]"
      plot_image_set(imgs_[cur_index],image_title,channel_names)
      #print "is image_set[" + str(cur_index) + "] enpty: " + str(is_image_set_empty(imgs_[cur_index]))

  python2 = sys.version_info[0] == 2
  if python2:
    raw_input("Press ENTER to quit.")
  else:
    input("Press ENTER to quit.")
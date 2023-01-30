##########################################
# MLX90640 Thermal Camera w Raspberry Pi
# -- 2fps with Interpolation and Blitting
##########################################
#
import time,board,busio
import numpy as np
import adafruit_mlx90640
import matplotlib.pyplot as plt
from scipy import ndimage

from PIL import Image as im
# from pil_video import make_video
import os
from PIL import Image  # noqa
import tempfile
import shutil
import imageio
from datetime import datetime

import time
import uuid
#from PIL import image
from skimage import io
import numpy as np

import digitalio

# Define input
input_1 = digitalio.DigitalInOut(board.D16)
input_1.direction = digitalio.Direction.INPUT
input_1.pull = digitalio.Pull.UP


# Create an empty list to store images
img_list = []
exportFolder=r"/home/Pi/Desktop/Test/FH_Ku_Praxisprojekt/exportedVideos/"

exportTmpFolder=tempfile.mkdtemp()

i2c = busio.I2C(board.SCL, board.SDA, frequency=400000) # setup I2C
mlx = adafruit_mlx90640.MLX90640(i2c) # begin MLX90640 with I2C comm
mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_16_HZ # set refresh rate
mlx_shape = (24,32) # mlx90640 shape

mlx_interp_val = 1 # interpolate # on each dimension
mlx_interp_shape = (mlx_shape[0]*mlx_interp_val,
                    mlx_shape[1]*mlx_interp_val) # new shape

fig = plt.figure(figsize=(12,9)) # start figure
ax = fig.add_subplot(111) # add subplot
fig.subplots_adjust(0.05,0.05,0.95,0.95) # get rid of unnecessary padding
therm1 = ax.imshow(np.zeros(mlx_interp_shape),interpolation='none',
                   cmap=plt.cm.bwr,vmin=25,vmax=45) # preemptive image
cbar = fig.colorbar(therm1) # setup colorbar
cbar.set_label('Temperature [$^{\circ}$C]',fontsize=14) # colorbar label

fig.canvas.draw() # draw figure to copy background
ax_background = fig.canvas.copy_from_bbox(ax.bbox) # copy background
# fig.show() # show the figure before blitting

frame = np.zeros(mlx_shape[0]*mlx_shape[1]) # 768 pts

def plot_update():
    fig.canvas.restore_region(ax_background) # restore background
    mlx.getFrame(frame) # read mlx90640
    data_array = np.fliplr(np.reshape(frame,mlx_shape)) # reshape, flip data
    data_array = ndimage.zoom(data_array,mlx_interp_val) # interpolate
    therm1.set_array(data_array) # set data
    therm1.set_clim(vmin=np.min(data_array),vmax=np.max(data_array)) # set bounds
    cbar.on_mappable_changed(therm1) # update colorbar range

    ax.draw_artist(therm1) # draw new thermal image   
    fig.canvas.blit(ax.bbox) # draw background
    fig.canvas.flush_events() # show the new image
    
    return im.frombytes('RGB', fig.canvas.get_width_height(), fig.canvas.tostring_rgb())
    
     
    #return data_array

def make_video(image_list: list, fps: int, exportVideoDir:str):
    # Make an empty directort in temp, which we are gonna delete later
    dirpath = tempfile.mkdtemp()  # Example: '/tmp/tmpacxadh7t'
    video_filenames = []
    for i, each_image in enumerate(image_list):
        filename = "{}/{}.png".format(dirpath, i)
        video_filenames.append(filename)
        each_image.save("{}".format(filename))
        
    # writer = imageio.get_writer("{}/test.mp4".format(dirpath), fps=fps)
    timestr=datetime.now().strftime("%d-%m-%Y-%H-%M-%S-%f")
    videoPath=f"{exportVideoDir}{timestr}.mp4"
    
    writer = imageio.get_writer(videoPath, fps=fps)
    
    for each_image in video_filenames:
        writer.append_data(imageio.imread(each_image))
    writer.close()
    
    # delete the tmp folder
    shutil.rmtree(dirpath)
    return videoPath

while True:  
  t_array = []
  counter=0
  while input_1.value == True:
      t1 = time.monotonic() # for determining frame rate
      try:
          pic= plot_update() # update plot

          img_list.append(pic)

          counter=counter+1

          print(f"Counter {counter}")


      except:
          continue
      # approximating frame rate
      t_array.append(time.monotonic()-t1)
      if len(t_array)>10:
          t_array = t_array[1:] # recent times for frame rate approx
      print('Frame Rate: {0:2.1f}fps'.format(len(t_array)/np.sum(t_array)))

  ## End of While
  make_video(img_list,fps=1,exportVideoDir= exportFolder)
  img_list.clear()

# Import the necessary libraries
import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes

import numpy as np
#import sensor_msgs.msg as sensor_msgs
from std_msgs.msg import Float64MultiArray

import time
import os

from openni import openni2
from openni import _openni2 as c_api

import open3d as o3d

# optional: when the device's OpenNI path not included in the system
dist = "/home/fdai6135/AstraOpenNI2Drivers/OpenNI-Linux-x64-2.3.0.66/Redist"


class FPFH_Publisher(Node):
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('fpfh_pub')

    self.fpfh_publisher = self.create_publisher(
            Float64MultiArray, 'fpf_histogram', 1)

    #offline test mode, if true will sample frames from numpy data file
    #instead of 3D camera, set to false to use live frames
    self.offline_test = True

    

    
    

    if self.offline_test:
        
        self.labels = ["Thumbs Up","Thumbs Down","Swipe Left","Swipe Right","One Snap","Two Snap"]
        self.fpfh_test_data = np.load(os.getcwd() + "/fpfh_test_data/test_feed_fpfh.npy")
        timer_period = 0.12 #seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cls_counter = 0
        self.sample_counter = 0
        self.frame_counter = 0

    else: 
      # Open and register the initialized Device
      self.device = openni2.Device.open_any()
      # Register Message
      if self.device:
        print("Device has been registered")
      else:
        print("Error! Device Registration failed!")

      # Create the depth stream
      self.depth_stream = self.device.create_depth_stream()
      # Display mode and resolution config from the OpenNI
      self.depth_stream.set_video_mode(c_api.OniVideoMode(pixelFormat=c_api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_100_UM, resolutionX=640, resolutionY=480,
                            fps=30))
      # Start the output stream
      self.depth_stream.start()
      # Create the timer
      timer_period = 0.001  # seconds
      self.timer = self.create_timer(timer_period, self.timer_callback)
      
      

      #visualizer instance
      self.vis = o3d.visualization.Visualizer()
      self.vis.create_window()

    # variable to set whether point clouds are saved 
    self.store_live_frames = False

    if self.store_live_frames:
      self.sample_counter = 0
      self.sample_list = []
      if not os.path.exists("output_frames"):
          os.mkdir("output_frames")
      
      if not os.path.exists("output_frames_png"):
          os.mkdir("output_frames_png")
      self.output_dir = os.getcwd() + "/output_frames"
      self.frame_png_counter = 0



  def timer_callback(self):
    
 
    if self.offline_test:
      
      
      fpfh_to_publish = self.fpfh_test_data[self.cls_counter,self.sample_counter,self.frame_counter,:]

      self.frame_counter += 1

      #control the access of test frames
      if self.frame_counter == 14:
          self.frame_counter = 0
          self.sample_counter += 1
          if self.sample_counter == 10:
              self.sample_counter = 0
              self.cls_counter += 1
              if self.cls_counter == 6:
                  self.cls_counter = 0

      #no need for processing since the frames are already processed

      msg_to_publish = Float64MultiArray()

      msg_to_publish.data = fpfh_to_publish.tolist()

      print("[INFO] Finished processing frame, publishing FPFH, label is " + self.labels[self.cls_counter])

      self.fpfh_publisher.publish(msg_to_publish)
       



    
    else:
      
      self.pcd = self.get_points()

      self.process_pcd(self.pcd)

    


  def get_depth_image(self):
    distanceMap = np.frombuffer(self.depth_stream.read_frame(
    ).get_buffer_as_uint16(), dtype=np.uint16).reshape(480, 640)

    # Correct the range, Depth images are 12bits
    depthImage = np.uint8(distanceMap.astype(float) * 255 / 2 ** 12 - 1)

    # Fill with blacks
    depthImage = 255 - depthImage
    return depthImage


  def get_points(self):
      depth_o3d = o3d.geometry.Image((self.get_depth_image()).astype(np.uint16))

      pcd = o3d.geometry.PointCloud.create_from_depth_image(depth_o3d,
                                                              o3d.camera.PinholeCameraIntrinsic(
                                                              o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault),
                                                              np.identity(4), depth_scale=100.0, depth_trunc=1000.0)

      pcd = pcd.remove_non_finite_points()

      return np.asarray(pcd.points)


  def process_pcd(self, pcd_as_np_array):

        c1 = time.perf_counter()

        if pcd_as_np_array.size != 0:

            
        
            self.o3d_pcd = o3d.geometry.PointCloud(
                            o3d.utility.Vector3dVector(pcd_as_np_array))


            cropping_points = np.array([
            #points for upper polygon
            [-1.0, 1.0, 1.5],
            [-1.0, 1.0, 2.5],
            [1.0, 1.0, 1.5],
            [1.0, 1.0, 2.5],

            #points for lower polygon
            [-1.0, -1.0, 1.5],
            [-1.0, -1.0, 2.5],
            [1.0, -1.0, 1.5],
            [1.0, -1.0, 2.5]
            ]).astype(np.float64)

            #create bounding box from specified points
            cuboid_points = o3d.utility.Vector3dVector(cropping_points)
            obb = o3d.geometry.OrientedBoundingBox.create_from_points(cuboid_points)

            #crop the input pcd
            voi_pcd = self.o3d_pcd.crop(obb)

            #downsampling
            voi_pcd = voi_pcd.voxel_down_sample(voxel_size=0.015)

            #scaling for visualizer
            voi_pcd = voi_pcd.scale(0.4, center = voi_pcd.get_center())

            #remove outlier points
            voi_pcd, ind = voi_pcd.remove_statistical_outlier(nb_neighbors = 20, std_ratio = 0.5)



            if len(voi_pcd.points) < 1500:
                  print("[INFO] Not enough points within box: " + str(len(voi_pcd.points)) + "/1500")
                

            else:
              
            
                fpfh_to_publish = self.produce_fpfh(voi_pcd)

                msg_to_publish = Float64MultiArray()

                c2 = time.perf_counter()

                time_to_process_fpfh = c2 - c1

                #delay execution if processing was too fast
                if time_to_process_fpfh < 0.12:
                    time.sleep(0.12 - time_to_process_fpfh)

                msg_to_publish.data = fpfh_to_publish.tolist()

                print("[INFO] Finished processing frame, publishing FPFH")

                self.fpfh_publisher.publish(msg_to_publish)

                #creating a new pcd to flip and display on the visualizer window
                pcd_to_vis = o3d.geometry.PointCloud(
                            o3d.utility.Vector3dVector(np.asarray(voi_pcd.points)))

                rotation_matrix = pcd_to_vis.get_rotation_matrix_from_xyz((0, 0, np.pi))
                pcd_to_vis = pcd_to_vis.rotate(rotation_matrix, center = (0,0,0))

                self.vis.add_geometry(pcd_to_vis)       
                self.vis.update_geometry(pcd_to_vis)
                self.vis.update_renderer()
                self.vis.poll_events()
                
                
                #store frames as both .png images and pcd files (each sample is 14 frames)
                if self.store_live_frames:
                    self.sample_list.append(voi_pcd)

                    self.vis.capture_screen_image(self.output_dir + "_png" + "/" + str(self.sample_counter) + 
                        "_" + str(self.frame_png_counter)+ ".png")
                
                    self.frame_png_counter += 1
                    if self.frame_png_counter == 14:
                        self.frame_png_counter = 0

                    if len(self.sample_list) == 14:
                        for index, frame in enumerate(self.sample_list):
                            o3d.io.write_point_cloud(self.output_dir + "/" + str(self.sample_counter) + "_" + str(index)
                            + ".pcd", frame)

                        self.sample_list = []
                        self.sample_counter += 1

                self.vis.clear_geometries()

                c3 = time.perf_counter()
                time_to_process = c3 - c1
                

                print("[INFO] Time taken to process frame: " + str(time_to_process))

            

        else:

            print("[INFO] Empty Received Point Cloud")


  #function to calculate fpfh from input point cloud frame
  def produce_fpfh(self, input_pcd):

        
        input_pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius= 0.03, max_nn=30))

        pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(input_pcd,
                    o3d.geometry.KDTreeSearchParamHybrid(radius= 0.05, max_nn=100))

        pcd_fpfh_data_norm = sum_and_normalize_fpfh_frame(pcd_fpfh.data.T)

        return pcd_fpfh_data_norm



    




def sum_and_normalize_fpfh_frame(feature_array):
    
    feature_nparr = np.array(feature_array)
    
    #summing columns over all points, ex: i/p shape: (2000,33) -> o/p shape: (33)
    feature_sum = feature_nparr.sum(axis = 0)

    #normalizing output vector
    feature_norm = feature_sum / feature_sum.sum()
    return feature_norm






def main(args=None):
  # Initialize OpenNI library
  openni2.initialize(dist)
  if openni2.is_initialized():
    print("openni2 driver has been initialized!")
  else:
    print("Error! openni2 driver has not been initialized!")

  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  fpfh_pub = FPFH_Publisher()
  
  # Spin the node so the callback function is called.
  rclpy.spin(fpfh_pub)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  fpfh_pub.destroy_node()
  openni2.unload()

  # Shutdown the ROS client library for Python
  rclpy.shutdown()
if __name__ == '__main__':
  main()

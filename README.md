# Hardwave_lidar_Yan_Han
Hardware lidar code (including all development versions)


The project is organized into two folders:

- **Lidar code in development**  
  Contains the in‐progress hardware LiDAR code; some scripts here may not run reliably.

- **lidarcode**  
  Contains the stable, fully tested hardware LiDAR code—the final version used for all subsequent data processing.

---

### 3D LiDAR (Velodyne VLP-16)

1. After setting up the LiDAR’s ROS 2 environment, start publishing its data with:  
   ```
   ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py
   ```
2. Once the Velodyne driver is running, open MATLAB and run the 3D LiDAR scripts from the **lidarcode** folder.
3. To visualize the point cloud in RViz, use:  
   ```
   ros2 run rviz2 rviz2 -f velodyne
   ```

---

### 2D LiDAR (Hokuyo UST-10LX)

1. After configuring the Hokuyo driver, launch it with:  
   ```
   ros2 launch urg_node2 urg_node2.launch.py
   ```
2. Once the node is active, open MATLAB and run the 2D LiDAR scripts from the **lidarcode** folder.
3. To view the laser scan in RViz, run:  
   ```
   ros2 run rviz2 rviz2
   ```
   Then in RViz:
   - Click **Add** → **By Topic**  
   - Select the **/scan** entry  
   - In the Displays panel, change the **Fixed Frame** from **map** to **laser**  
   This will render the 2D LiDAR data in RViz.

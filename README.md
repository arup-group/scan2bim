# Implementation processes

Step 1 - download point cloud data. Use matlab lidar processing toolkit to segment pcd.

Step 2 - use open3d_.py for downsampling, clustering, and removing noise. 

Step 3 - use Scan2Bim_ver6.m to further removing noise data. 

Step 4 - use the dynamo script in the modelling folder to create revit models. 

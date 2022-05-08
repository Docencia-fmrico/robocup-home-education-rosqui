<p align="center"><a href="https://www.youtube.com/channel/UC4Loc3tyy1vvGsMoBC5KCSw" target="_blank">
    <img src="https://github.com/Docencia-fmrico/visual-behavior-rosqui/blob/main/images/logo.jpg">
</a></p>

<img src="https://github.com/Docencia-fmrico/visual-behavior-rosqui/blob/main/images/kobuki.jpg" align="right"
     alt="Kobuki Image" width="250" height="200">
    
<h1>ROBOCUP HOME EDUCATION</h1>
 
 <p> We were asked to complete two of the three different tasks for the robocup home edition which its final is going to be held in the 1st os June in Guimarães, Portugal. These 3 tasks are:
 
 <b>Carry my Luggage</b>:
  
  Its main goal is that the robot helps the operator to carry a bag to the car parked outside.
  This is the map in which the task is going to be done:
  
 <img src="" align="center"
alt="Carry my luggage map" width="700" height="400">
  
  The route is:
  
    1º: The robot starts in the start point and it should get inside of the arena.
  
    2º: Inside of the arena
 
 <b>Find my mates</b>:
 
 <b>The Receptionist</b>:

<h2>CARRY MY LUGGAGE</h2>
First of all we have done a behaviour tree called <b>follow_person.xml</b>. Using Darknet_ros with a 3D camera we can obtain all x, y and z coordinates of a person. 

<details><summary><b>Instalation</b></summary>
For this task we had to install the following packages:
    
    $ sudo apt-get install ros-noetic-behaviortree-cpp-v3
    $ sudo apt-get install ros-noetic-move-base-msgs
    $ sudo apt-get install ros-noetic-kobuki-*
    $ sudo apt-get install ros-noetic-cameras-*
    
If you want to try it by your own personal camera, you can download the following package:
   
    $ sudo apt-get install ros-noetic-usb-cam-*
   
By the way, if you get in trouble with any package you may need to download a github repository related to the package. 

</details>

<details><summary><b>Commands Used</b></summary>
    
- <b>See Darknet Ros using 3D camera</b>
    
    You should follow the followings steps:
    
        $ roslaunch openni2_launch openni2.launch
        $ roslaunch darknet_ros darknet_ros.launch image:=/camera/rgb/image_raw/
    
- <h4>See Darknet Ros using usb camera</h3>
    
    You should follow the followings steps:
    
        $ roscore
        $ rosrun usb_cam usb_cam_node
        $ rosrun cameras_cpp nodo_camera
        $ roslaunch darknet_ros darknet_ros.launch iamge:=/usb_cam/image_raw/
      
</details>

<details><summary><b>Behaviour tree</b></summary>
    
This is the tree we have decided to use:
 
<img src="https://github.com/Docencia-fmrico/visual-behavior-rosqui/blob/julia/images/bt_fp.gif" align="center"
alt="Follow person bt" width="600" height="600">

And this is how it looks like in <a href="https://github.com/BehaviorTree/Groot">Groot</a>:

<img src="https://github.com/Docencia-fmrico/visual-behavior-rosqui/blob/julia/images/follow_person_groot.png" align="center"
alt="Follow person bt groot" width="600" height="600">
        
    
</details>

<details><summary><b>Filtered Darknet Ros</b></summary>
We would like to point out that when using <a href="https://github.com/leggedrobotics/darknet_ros">Darknet_ros</a> only for people, we had to edit all .yaml files. We just left <b>person</b> in <b>detection clases names</b>
    
Here you can see a picture of it:
    
<img src="https://github.com/Docencia-fmrico/visual-behavior-rosqui/blob/main/images/darknet_filtered.jpg" align="center"
alt="Darknet Filtered" width="700" height="400">
    
    
</details>
 
<h2>FIND MY MATES</h2>
First of all we have done a behaviour tree called <b>follow_ball.xml</b>. Using <b>Funcion Transformations</b>, a 3D camera and a color filter we can detect and follow a colored ball. 
    
<details><summary><b>Instalation</b></summary>
For this task we had to install the following packages:

    $ sudo apt-get install openni2-*
    $ sudo apt-get install ros-noetic-rgbd-launch 
    $ sudo apt-get install --fix-missing ros-noetic-rgbd-launch
 
</details>
    
<details><summary><b>Commands Used</b></summary>
        
- <b>Filter Ball using 3D camera</b>
    
    You should follow the followings steps:
    
        $ roslaunch openni2_launch openni2.launch
        $ rosrun cameras_cpp nodo_camera (filter image)
        $ rviz 
    
    In rviz add image and its topic is /hsv/image_filtered/
    And these are the values used for filtering the ball:
    
    <img src="https://github.com/Docencia-fmrico/visual-behavior-rosqui/blob/main/images/ball_filtered.jpg" align="center"
    alt="Ball filtered" width="600" height="600">
    
    
 - <b>Filter Ball using rviz</b>
    
    You should follow the followings steps:
    
        $ roslaunch robots sim.launch
        $ rosrun cameras_cpp nodo_camera (filter image)
        $ rosrun cameras_cpp nodo_rgbd_filtered (publish in the image filtered topic)
        $ roslaunch robots kobuki_xtion.launch (makes the transform)
        $ rviz (and choose 0 channel)
        $ roslaunch kobuki_keyop keyop.launch (for moving in the simulation and the image)
      
</details>
    
<details><summary><b>Behaviour tree</b></summary>

This is the tree we have decided to use:
 
<img src="https://github.com/Docencia-fmrico/visual-behavior-rosqui/blob/main/images/bt_fb.gif" align="center"
alt="Follow ball bt" width="600" height="600">

And this is how it looks like in <a href="https://github.com/BehaviorTree/Groot">Groot</a>:

<img src="https://github.com/Docencia-fmrico/visual-behavior-rosqui/blob/julia/images/follow_ball_groot.png" align="center"
alt="Follow ball bt groot" width="600" height="600">

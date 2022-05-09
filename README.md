<p align="center"><a href="https://www.youtube.com/channel/UC4Loc3tyy1vvGsMoBC5KCSw" target="_blank">
    <img src="https://github.com/Docencia-fmrico/visual-behavior-rosqui/blob/main/images/logo.jpg">
</a></p>

<img src="https://github.com/Docencia-fmrico/visual-behavior-rosqui/blob/main/images/kobuki.jpg" align="right"
     alt="Kobuki Image" width="230" height="200">
    
<h1>ROBOCUP HOME EDUCATION</h1>
 
 <p> We were asked to complete two of the three different tasks for the Robocup Home Edition which is going to be held on first of June in Guimarães, Portugal. These 3 tasks are:
 
 <b>Carry my Luggage</b>:
  
  Its main goal is that the robot helps the operator to carry a bag to the car parked outside.
  This is the map in which the task is going to be done:
  
 <img src="https://github.com/Docencia-fmrico/robocup-home-education-rosqui/blob/main/images/luggage_map.jpeg" align="center"
alt="Carry my luggage map" width="500" height="500">
  
  The route is:
  
    1º: The robot starts in the start point and it should get inside of the arena.
    2º: Inside of the arena, reach to the referee's position.
    3º: The referee is going to indicate the bag and the robot will move closer to it.
    4º: The robot should indicate when it should start following the referee.
    5º: The robot should follow the referee inside of the arena avoiding obstacles.
    6º: The robot should follow the referee out of the arena until he/she indicates it.
    7º: The robot should come back to the arena.
 
 <b>Find my mates</b>:
    
  Its main goal is that the robot will get int the arena to find mates and tell the referee their names and extra information.
  This is the map in which the task is going to be done:
  
 <img src="https://github.com/Docencia-fmrico/robocup-home-education-rosqui/blob/main/images/find_mates_map.jpeg" align="center"
alt="Find My Mates map" width="500" height="500">
  
  The route is:
  
    1º: The robot starts in the start point and it should get inside of the arena.
    2º: Inside of the arena, the robot must find the first person.
    3º: After that, the robot will move closer to the person.
    4º: The robot should  ask the person's name and obtain the T-shirt color and the object.
    5º: After that, the robot will report the information to the referee or it will keep finding people and report at the end. 
  
 <b>The Receptionist</b>:
 
  Its main goal is that the robot will welcome people, ask their names and their favourite drink and give seat. After that, the robot will report Jhon about people, indicating where they are seated and the information obtained.
  This is the map in which the task is going to be done:
  
 <img src="https://github.com/Docencia-fmrico/robocup-home-education-rosqui/blob/main/images/recepcionist_map.jpeg" align="center"
alt="The Recepcionist map" width="500" height="500">
  
  The route is:
  
    1º: The robot starts in the start point and it should get inside of the arena.
    2º: Inside of the arena, the robot should welcome the first person, obtaining information about him/her.
    3º: After that, the robot should guide the person to the seats and offer one.
    4º: After that, the robot will report the information to the referee or it will keep finding people and report at the end.
 

We decided to deal <b>carry my luggage</b> and <b>find my mates</b>: 
<h2>CARRY MY LUGGAGE</h2>

This is the <b>behaviour tree</b> we have decided to implement:

<img src="https://github.com/Docencia-fmrico/robocup-home-education-rosqui/blob/main/images/luggage_bt.png" align="center"
alt="The Recepcionist map" width="600" height="600">


<details><summary><b>Start</b></summary>
 
 Start, basically is a node used for <b>initializing the program without clicking a button.</b>
    
 Here you can see the tick of <b>Start.cpp</b>:
    
     Start::tick()
     { 
            ROS_INFO("Start");
            if (forwarder_.get_first() == 1)
            {
                forwarder_.listen();
            }

            if (forwarder_.get_start() == 0)
                return BT::NodeStatus::SUCCESS;

            return BT::NodeStatus::RUNNING;
     }
    
    
</details>

<details><summary><b>Go to Ref</b></summary>
    
    
 Go to Ref, basically is a node used for <b>going to the referee's position.</b>
    
 In <b>GotoRef.h</b> you can find the vector with the <b>specific position of the referee.</b>
    
     std::vector<float> coords_ = {0.88, 4.48, 0.0, 0.0, 0.0, -0.96, 0.262};
    
 Here you can see the tick of <b>GotoRef.cpp</b>:    
    
    GoToRef::tick()
    {
	    if(first_){
		    Navigation my_node_;
		    my_node_.doWork(200, coords_);
		    first_ = false;
	    }
	
	    if (result_ != 0)
		    ROS_INFO("Result: %d", result_);

	    if (result_ == 3)
	    {
		    ROS_INFO("LEAVING");
		    return BT::NodeStatus::SUCCESS;
	    }

  	    return BT::NodeStatus::RUNNING;
    }
    
    
</details>

<details><summary><b>Detect Luggage</b></summary>
	
This node waits for the referee to receive either <b>left</b> or <b>right</b>.
    
</details>

<details><summary><b>Go to bag</b></summary>
       
 Go to bag, basically is a node used for <b>turning to the direction given in Detect Luggage.</b>
      
 Here you can see the tick of <b>GotoBag.cpp</b>:
	
 ```
 GotoBag::tick()
 {
     if (first_)
     {
 	detected_ts_ = ros::Time::now();
    	bag_pos_ = getInput<std::string>("bag_pos").value();
    	first_ = false;
     }
	
     geometry_msgs::Twist cmd;
     double current_ts_ = (ros::Time::now() - detected_ts_).toSec();
     ROS_INFO("TIME: %f", current_ts_);
	
     if ((current_ts_ < ACTION_TIME_))
     {
 	cmd.linear.x = 0;

   	if (bag_pos_ == "left")
            cmd.angular.z = TURNING_VEL_;
      	else
            cmd.angular.z = -TURNING_VEL_;
        ROS_INFO("TIME: %f %f", current_ts_, TURNING_VEL_);
				  
     }else if (current_ts_ >= 5*ACTION_TIME_)
     {
        ROS_INFO("BAG REACHED");
        return BT::NodeStatus::SUCCESS;
     }
	
     pub_vel_.publish(cmd);
     return BT::NodeStatus::RUNNING;
 }
 ```
   
</details>

<details><summary><b>Follow Person</b></summary>
	
 This node is reused from the task <b>visual behaviour</b>.
	
 You can <a href="https://github.com/Docencia-fmrico/visual-behavior-rosqui">have a look</a> if you want to.
    
    
</details>

<details><summary><b>Lost</b></summary>
This node is used when the robot cannot see the referee and start turning and saying "I am lost referee", so the referee can realize that the robot is not seeing him/her.
Here you can see the tick of <b>Lost.cpp</b>:    
	
```
 Lost::tick()
 {
   ROS_INFO("Lost tick");
   geometry_msgs::Twist cmd;
   luggage::Dialog forwarder_;

   if ((ros::Time::now() - detected_ts_).toSec() > time_)
   {
     detected_ts_ = ros::Time::now();
     forwarder_.speak("I am lost referee");
     time_ ++;
   }

   cmd.linear.x = 0;
   cmd.angular.z = TURN_VEL;

   pub_vel_.publish(cmd);
   return BT::NodeStatus::FAILURE;
 }
	
```
   
</details>

<details><summary><b>Percieve Person</b></summary>
	
 This node is reused from the task <b>visual behaviour</b>.
	
 You can <a href="https://github.com/Docencia-fmrico/visual-behavior-rosqui">have a look</a> if you want to.
    
    
</details>

<details><summary><b>Go to origin</b></summary>
	
Go to Origin, basically is a node used for <b>going to the starting position.</b>
    
 In <b>GoToOrigin.h</b> you can find the vector with the <b>specific position of the origin.</b>
    
     std::vector<float> coords_ = {0.37, 3.63, 0.0, 0.0, 0.0, -0.91, 0.41};
    
 Here you can see the tick of <b>GotoOrigin.cpp</b>:    

```
 GoToOrigin::tick()
{	
    if(first_){
	Navigation my_node_;
	my_node_.doWork(200, coords_);
	first_ = false;
    }

    if (result_ != 0)
	ROS_INFO("Result: %d", result_);

    if (result_ == 3)
    {
	ROS_INFO("LEAVING");
	return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

```    
</details>

<h2>FIND MY MATES</h2>

This is the <b>behaviour tree</b> we have decided to implement:

<img src="https://github.com/Docencia-fmrico/robocup-home-education-rosqui/blob/main/images/find_mates_bt.png" align="center"
alt="The Recepcionist map" width="600" height="600">

<details><summary><b>Go to arena</b></summary>
	
Go to arena, basically is a node used for <b>going to the middle of the arena.</b>
    
 In <b>GotoArena.h</b> you can find the vector with the <b>specific position of the middle of the arena.</b>
    
     std::vector<float> coords_ = {0.37, 3.63, 0.0, 0.0, 0.0, -0.91, 0.41};
    
 Here you can see the tick of <b>GotoArena.cpp</b>:    
 
 ```
 GoToArena::tick()
 {	
	if(first_){
		Navigation my_node_;
		my_node_.doWork(200, coords_);
		first_ = false;
	}

	if (result_ != 0)
		ROS_INFO("Result: %d", result_);

	if (result_ == 3)
	{
		ROS_INFO("LEAVING");
		return BT::NodeStatus::SUCCESS;
	}

  	return BT::NodeStatus::RUNNING;
}
```    
    
</details>

<details><summary><b>Go to person</b></summary>
	
Go to person, basically is a node used for <b>going for each person's position.</b>
    
 In <b>GotoPerson.h</b> you can find 6 vectors for <b>each person's position</b>
    
    std::vector<float> coords1 = {1.24, 6.43, 0.0, 0.0, 0.0, 0.73, 0.67};
    std::vector<float> coords2 = {-0.7, 6.15, 0.0, 0.0, 0.0, 0.84, 0.52};
    std::vector<float> coords3 = {-0.77, 5.82, 0.0, 0.0, 0.0, 0.99, 0.02};
    std::vector<float> coords4 = {-0.15, 4.28, 0.0, 0.0, 0.0, -0.94, 0.32};
    std::vector<float> coords5 = {0.53, 3.44, 0.0, 0.0, 0.0, -0.76, 0.64};
    std::vector<float> coords6 = {2.21, 3.21, 0.0, 0.0, 0.0, -0.69, 0.71};
    
 Here you can see the tick of <b>GotoPerson.cpp</b>:    
 
 ```
 GoToPerson::tick()
 {	
	if(first_){
		Navigation my_node_;
		my_node_.doWork(200, all_coords[current_pos_]);
		first_ = false;
	}

	if (result_ != 0)
		ROS_INFO("Result: %d", result_);

	if (result_ == 3)
	{
		ROS_INFO("LEAVING");
		return BT::NodeStatus::SUCCESS;
		current_pos_++;
		first_ = true;
		result_ = 0;
	}

  	return BT::NodeStatus::RUNNING;
 }
``` 
</details>

<details><summary><b>Analyze person</b></summary>

In this node we tried to get the person's color T-shirt but we found many problems so we finally decided to make a node which <b>tells that there is a person.</b>
	
Here you can see the tick of <b>AnalyzePerson.cpp:</b>
```
AnalyzePerson::tick()
{ 
    if (detected_)
    {
      setOutput("occupied_pos",occupied_pos_);
      detected_ = false;
      return BT::NodeStatus::SUCCESS; 
    }
    else {
      occupied_pos_++;
      return BT::NodeStatus::FAILURE; 
    }
}

```

</details>

<details><summary><b>Say description</b></summary>

This node gets the <b>person's position</b> from the blackboard and tells the referee.
	
Here you can see the tick of <b>SayDescription.cpp:</b>
```	
SayDescription::tick()
{	

  if(first_){
    detected_ts_ = ros::Time::now();
    pos_ = getInput<int>("occupied_pos").value();
    first_ = false;
  }

  double current_ts_ = (ros::Time::now() - detected_ts_).toSec();

  if(current_ts_< TIME_TO_SPEAK){
    forwarder_.speak("There is a person in position" + std::to_string(pos_));
    return BT::NodeStatus::RUNNING;
  }
  else {
    first_ = true;
    return BT::NodeStatus::SUCCESS;
  }
}
```    
    
</details>


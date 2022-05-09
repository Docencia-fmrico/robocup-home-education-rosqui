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
    
 Here you can see the tick in <b>start.cpp</b>:
    
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
    
    
</details>

<details><summary><b>Detect Luggage</b></summary>
    
    
</details>

<details><summary><b>Go to bag</b></summary>
    
    
</details>

<details><summary><b>Follow Person</b></summary>
    
    
</details>

<details><summary><b>Lost</b></summary>
    
    
</details>

<details><summary><b>Percieve Person</b></summary>
    
    
</details>

<details><summary><b>Go to origin</b></summary>
    
    
</details>


<h2>FIND MY MATES</h2>

This is the <b>behaviour tree</b> we have decided to implement:

<img src="https://github.com/Docencia-fmrico/robocup-home-education-rosqui/blob/main/images/find_mates_bt.png" align="center"
alt="The Recepcionist map" width="600" height="600">

<details><summary><b>Go to arena</b></summary>
    
    
</details>

<details><summary><b>Go to person</b></summary>
    
    
</details>

<details><summary><b>Analyze person</b></summary>


 <b>Filtering problems</b>   
    
</details>

<details><summary><b>Say description</b></summary>
    
    
</details>


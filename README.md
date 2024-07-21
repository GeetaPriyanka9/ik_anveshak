# ik_anveshak
Code_documentation:

# TOPICS:

| **Topic**              | **Function**                                                                                                                                                                 |
|------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `/auto_arm_signals`    | Publishes the frequency values for PWM to the motor drivers.                                                                                                                 |
| `/enc_drive`           | Retrieves the current base, elbow, and shoulder angles via the `enc_callback` function. Based on the current position and the goal position, joints can be moved accordingly. |
| `/arm_goal`            | Receives the goal position. The `arm_goal_sub_clbk` function checks if the received values are valid numerical values and assigns them to `self.goal`.                         |
| `/arm_goal_bool`       | Specifies whether the goal position needs to be entered manually. The `arm_goal_given_clbk` function sets the `self.arm_goal_coming_from_tanish` attribute to false if manual input is required. |
| `/ik_over_ah`          | Publishes boolean values indicating whether the inverse kinematics mission is completed or not.                                                                              |




# FLOW CHART :














## Variables Intuition:

1. ```self.enc_data```: It is an array of the current encoder readings of base joint,shoulder and elbow joint.
2. ```self.goal```: we get these goal coordinates from the ```arm_goal_sub_clbk``` callback function if the boolean ```self.arm_goal_coming_from_tanish``` is true and if that’s false we specify goal coordinates manually using ```self.x```, ```self.y```, ```self.z```
3. ```self.start_time```: checks keep track of time.
4. ```self.tanish_count```: It is defined in order for us to decide if we want to just take goal coordinates at the start or if we want to keep getting goal coordinates till we reach the goal( they kind of slightly change as our rover moves and detects the object’s center coordinates which in this case are our goal coordinates).
5. ```self.goal_counter```: For the task to complete we need to reach the goal and hold the object there for sometime and then reach the zero position, so if this becomes 2 we publish ```self.pranav_bool_pub``` as true.
6. ```theta1```,```theta2```,```theta3```: encoder reading for base,shoulder,elbow joints respectively.
7. ```self.input```: it is just an input value of 1 or -1 for picking up or dropping respectively.
8. ```self.pranav_bool_pub```: after the task is completed and the arm returns to zero position it becomes true.
9. ```listen_to_tanish```: once we have reached the goal then this becomes false and we specify the further actions.
10. ```self.pwm_limit```: It is an array specifying the maximum frequency of pwm signals for base,shoulder,elbow.













## Methods:
   def angle_value(self):
a1 and a2 are the lengths of the shoulder and elbow links respectively.
```python
    
        x = self.x
        y = self.y
        z = self.z
        print(
            f"Going to coordinates :{x},{y},{z}, {abs((x**2+y**2+z**2-a1**2-a2**2)/(2*a1*a2))}")
        if (abs((x**2+y**2+z**2-a1**2-a2**2)/(2*a1*a2)) < 1):
            print("Angles being set for coordinates")
            self.q3 = math.pi/2
            self.q2 = -math.acos((x**2+y**2+z**2-a1**2-a2**2)/(2*a1*a2))
            self.q1 = math.atan(z/math.sqrt(x**2+y**2)) + \
                math.atan((-a2*math.sin(self.q2))/(a1+a2*math.cos(self.q2)))
        if (y != 0):
            self.q3 = math.atan(x/y)

        theta1 = self.q3
        theta2 = math.pi/2 - self.q1
        theta3 = math.pi/2 + self.q2

       # print(np.arccos(self.pz/(2*k)))
        base = (180*theta1)/math.pi
        shoulder = (180*theta2)/math.pi
        elbow = (180*theta3)/math.pi
        print(f"Goal: Base = {base}, Shoulder = {shoulder}, Elbow = {elbow}")

        print(
            f"Current: Base = {self.enc_data[0]}, Shoulder = {self.enc_data[1]}, Elbow = {self.enc_data[2]}")
```
Here we are calculating the joint angles (q1, q2, q3) needed to reach a specified goal position (x, y, z) in 3D space, using inverse kinematics. It then converts these angles to degrees.
By checking abs((x**2+y**2+z**2-a1**2-a2**2)/(2*a1*a2)) < 1 we are seeing whether the goal is reachable because cosine lies between [-1,1].
Then it creates a msg of the type Int32MultiArray() and publishes '/auto_arm_signals' topic.
 Compares calculated desired angles with the current encoder readings to determine the motor commands using ```self.pwm_limit```.



# Setting up NTP server from laptop

We use chrony for this:

```
sudo apt install chrony
```

Open the config file:

```
sudo nano /etc/chrony/chrony.conf
```

Add the following:

```
# If pc has no connection set local time as server
local stratum 10

# Allow waterlinked 3d sonar to make a connection
allow 192.168.194.96
```

Then restart the terminal:
```
sudo systemctl restart chronyd
```

In the Sonar GUI go to config and change NTP server from automatic to the ip address of your pc. 

# Static transformer between sonar and base frame

So if you run the launch file you run also a static transformer node that publishes the transform from sonar to base frame.
The sonar frame has x forward, y right and z down. The base frame has x forwards, y left and z up.

# Launch file

Install the Water Linked sonar Python SDK before launching:

```
python3 -m pip install --user wlsonar==0.4.0
```

Launch file contains the base_link that flips the point cloud 180 degrees around the x-axis.
Because frame sonar is z down and y to the right, and in rviz and open3D z is up and y to left, so visualization purposes to 
not see the point cloud upside down in the viewers.

```
ros2 launch sonar_3d sonar3d_drive.launch.py 
```

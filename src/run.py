#!/usr/bin/env python

import rospy
import subprocess
import signal

  
child = subprocess.Popen(["roslaunch","cola2_stonefish","sparus2_haifa_deepersense_simulation.launch"], stdout=subprocess.PIPE)
for c in iter(lambda: child.stdout.readline(), b''): 
    print('line', c)
    if c.startswith('[INFO]'):
        break
# child.wait() #You can use this line to block the parent process untill the child process finished.
print("parent process")
print(child.poll())

rospy.loginfo('The PID of child: %d', child.pid)
print ("The PID of child:", child.pid)

rospy.sleep(15)

child.send_signal(signal.SIGINT) #You may also use .terminate() method
#child.terminate()

#for more: https://docs.python.org/2/library/subprocess.html
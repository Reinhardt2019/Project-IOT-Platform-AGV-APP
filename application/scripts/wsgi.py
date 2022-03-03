#!/usr/bin/env python3
import os
import rospy
import threading
from AGV_APP import create_app
from std_msgs.msg import String

#threading.Thread(target=lambda: rospy.init_node('test_node', disable_signals=True)).start()
#pub = rospy.Publisher('/delivery_server/delivery', String, queue_size=1)

app = create_app()

if __name__ == "__main__":
    app.run(host='0.0.0.0')


import rospy, cv2, cv_bridge, numpy, math
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

class Follower:
  def __init__(self):
      self.bridge = cv_bridge.CvBridge()
      cv2.namedWindow("window", 1)
      self.image_sub = rospy.Subscriber('image_raw', Image, self.image_callback)

      self.pub = rospy.Publisher('track_point', Float64, queue_size=10)



  def image_callback(self, msg):
      image = self.bridge.imgmsg_to_cv2(msg)
      hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
      lower_yellow = numpy.array([ 50, 50, 140])
      upper_yellow = numpy.array([255, 255, 190])
      #original limits below
      #lower_yellow = numpy.array([ 50, 50, 170])
      #upper_yellow = numpy.array([255, 255, 190])
      mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
      h, w, d = image.shape
      search = int(h/2.0*(math.asin(0.5/3.0) / .875) + h/2.0)
      search_top = search - 10
      search_bot = search_top + 10
      mask[0:search_top, 0:w] = 0
      mask[search_bot:h, 0:w] = 0
      M = cv2.moments(mask)
      if M['m00'] > 0:
          cx = int(M['m10']/M['m00'])
          cy = int(M['m01']/M['m00'])
          cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

      cv2.imshow("window", image)
      cv2.waitKey(3)

      #hello_str = "hello world %s" % rospy.get_time()
      #rospy.loginfo(hello_str)
      y = (w/2 - cx)/float(w/2)
      self.pub.publish(y)


rospy.init_node('follower')
follower = Follower()
rospy.spin()

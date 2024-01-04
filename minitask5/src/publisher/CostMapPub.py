import time
from Logger import logger
import threading
#from common.OccupiedGrid import OccupiedGrid
from nav_msgs.msg import OccupancyGrid

class CostMapPub(threading.Thread):
    def __init__(self, rospy, occupancyGrid):
        threading.Thread.__init__(self)
        self.rospy = rospy
        self.occuGrid = occupancyGrid
        self.isStop = False
        self.mapPub = rospy.Publisher('map_msgs', OccupancyGrid, queue_size=10)

    def stop(self):
        self.isStop = True

    def run(self):
        while not self.isStop:
            self.occuGrid.drawMap()
            self.mapPub.publish(self.occuGrid.grid)
            time.sleep(0.2) # s

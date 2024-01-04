from Logger import logger
from common.DataStruct import TopicMsg
import time
import threading
import math

class MoveToChecking(threading.Thread):
    def __init__(self, topicMsg):
        threading.Thread.__init__(self)
        self.topicMsg = topicMsg
        self.isStop = False
    
    def stop(self):
        self.isStop = True

    def run(self):
        while not self.isStop:
            if self.topicMsg.currPointToMove == None:
                pass
            else:
                pose = self.topicMsg.currPointToMove
                dis = math.sqrt((self.topicMsg.currentX - pose.position.x)**2 + (self.topicMsg.currentY - pose.position.y)**2)
                if dis <= 0.7:
                    if self.topicMsg.move_base != None:
                        self.topicMsg.currPointToMove = None
                        self.topicMsg.move_base.cancel_goal()

            time.sleep(0.1) # s


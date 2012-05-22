#!/usr/bin/python
import roslib
roslib.load_manifest('raw_generic_states')
import rospy
import smach
import smach_ros
import raw_srvs.srv
import std_srvs.srv


class enable_object_finder(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed'])
        
        self.object_finder_srv_start = rospy.ServiceProxy('/raw_perception/object_segmentation/start', std_srvs.srv.Empty)

    def execute(self, userdata): 
        #get object pose list
        rospy.wait_for_service('/raw_perception/object_segmentation/start', 30)
        
        try:
            resp = self.object_finder_srv_start()
        except rospy.ServiceException, e:
            error_message = "%s"%e
            rospy.logerr("calling <</raw_perception/object_segmentation/start>> service not successfull, error: %s", error_message)
            return 'failed'
        
        return 'succeeded'

class detect_object(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed'],
            output_keys=['object_list'])
        
        self.object_finder_srv = rospy.ServiceProxy('/raw_perception/object_segmentation/get_segmented_objects', raw_srvs.srv.GetObjects)
        self.object_finder_srv_stop = rospy.ServiceProxy('/raw_perception/object_segmentation/stop', std_srvs.srv.Empty)

    def execute(self, userdata):     
        #get object pose list
        rospy.wait_for_service('/raw_perception/object_segmentation/get_segmented_objects', 30)
        rospy.sleep(3)
        for i in range(20): 
            print "find object try: ", i
            resp = self.object_finder_srv()
              
            if (len(resp.pointCloudCentroids) <= 0):
                rospy.loginfo('found no objects')
                rospy.sleep(1);
            else:    
                rospy.loginfo('found {0} objects'.format(len(resp.pointCloudCentroids)))
                break
            
        #stop perception component
        rospy.wait_for_service('/raw_perception/object_segmentation/stop', 5)
        try:
            resp_stop = self.object_finder_srv_stop()
        except rospy.ServiceException, e:
            error_message = "%s"%e
            rospy.logerr("calling <</raw_perception/object_segmentation/stop>> service not successfull, error: %s", error_message)
            return 'failed'       
    
        if (len(resp.pointCloudCentroids) <= 0):
            rospy.logerr("no graspable objects found");
            userdata.object_list = []            
            return 'failed'
        
        else:
            userdata.object_list = resp.pointCloudCentroids
            return 'succeeded'

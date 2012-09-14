#!/usr/bin/python
import roslib
roslib.load_manifest('raw_generic_states')
import rospy
import smach
import smach_ros
import raw_srvs.srv
import std_srvs.srv
import tf 



class find_drawer(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed'], output_keys=["drawer_pose"])
        
    def execute(self, userdata): 
               
        # find drawer front edge position with sergeys perception component
               
        self.userdata.drawer_pose = ""
        
        return 'succeeded'
 

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

        for i in range(40): 
            print "find object try: ", i
            resp = self.object_finder_srv()
              
            if (len(resp.objects) <= 0):
                rospy.loginfo('found no objects')
                rospy.sleep(0.5);
            else:    
                rospy.loginfo('found {0} objects'.format(len(resp.objects)))
                break
            
        #stop perception component
        rospy.wait_for_service('/raw_perception/object_segmentation/stop', 5)
        try:
            resp_stop = self.object_finder_srv_stop()
        except rospy.ServiceException, e:
            error_message = "%s"%e
            rospy.logerr("calling <</raw_perception/object_segmentation/stop>> service not successfull, error: %s", error_message)
            return 'failed'       
    
        if (len(resp.objects) <= 0):
            rospy.logerr("no graspable objects found");
            userdata.object_list = []            
            return 'failed'
        
        else:
            userdata.object_list = resp.objects
            return 'succeeded'


class recognize_objects(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed'],
            input_keys=['recognized_objects'],
            output_keys=['recognized_objects'])
        
        self.object_finder_srv = rospy.ServiceProxy('/raw_object_perception/find_objects', raw_srvs.srv.GetObjects)

    def execute(self, userdata):     
        #get object pose list
        rospy.wait_for_service('/raw_object_perception/find_objects', 30)

        for i in range(10): 
            print "find object try: ", i
            resp = self.object_finder_srv()
              
            if (len(resp.objects) <= 0):
                rospy.loginfo('found no objects')
            else:    
                rospy.loginfo('found {0} objects'.format(len(resp.objects)))
                break

        if len(resp.objects) == 0:
            rospy.loginfo('NO objects in FOV')
            return 'failed'

        tf_listener = tf.TransformListener()

        tf_wait_worked = False
        while not tf_wait_worked:
            try:
                print "frame_id:",resp.objects[0].pose.header.frame_id
                print "cluster_id:",resp.objects[0].cluster.header.frame_id
                tf_listener.waitForTransform(resp.objects[0].pose.header.frame_id, '/odom', resp.objects[0].pose.header.stamp, rospy.Duration(2))
                tf_wait_worked = True
            except Exception, e:
                print "tf exception in recognize person: wait for transform: ", e
                tf_wait_worked = False
                rospy.sleep(0.5)
                   
        transformed_poses = []
        obj_count = 1
        for obj in resp.objects:
            tf_worked = False
            
            if obj_count >= 4:
                break

            obj_count = obj_count + 1

            while not tf_worked:
                try:
                    obj.pose = tf_listener.transformPose('/odom', obj.pose)
                    transformed_poses.append(obj.pose)
                    tf_worked = True
                except Exception, e:
                    print "tf exception in recognize person: ", e
                    tf_worked = False

        userdata.recognized_objects = transformed_poses

        print "################ OBJECTS TAKEN: ", len(userdata.recognized_objects)

        return 'succeeded'

#!/usr/bin/python
import roslib
roslib.load_manifest('raw_generic_states')
import rospy
import smach
import smach_ros
import hbrs_srvs.srv
import std_srvs.srv
import tf 
import geometry_msgs.msg



class find_drawer(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed'], output_keys=["drawer_pose"])
        
    def execute(self, userdata): 
               
        # find drawer front edge position with sergeys perception component
               
        userdata.drawer_pose = geometry_msgs.msg.PoseStamped()
        
        return 'succeeded'
 


class detect_object(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed'],
            output_keys=['object_list'])
        
        self.object_finder_srv = rospy.ServiceProxy('/raw_perception/object_segmentation/get_segmented_objects', hbrs_srvs.srv.GetObjects)

    def execute(self, userdata):     
        #get object pose list
        rospy.wait_for_service('/raw_perception/object_segmentation/get_segmented_objects', 30)

        for i in range(10): 
            print "find object try: ", i
            resp = self.object_finder_srv()
              
            if (len(resp.objects) <= 0):
                rospy.loginfo('found no objects')
                rospy.sleep(0.1);
            else:    
                rospy.loginfo('found {0} objects'.format(len(resp.objects)))
                break
            
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
        
        self.object_finder_srv_name = '/detect_objects'
        self.object_finder_srv = rospy.ServiceProxy(self.object_finder_srv_name, hbrs_srvs.srv.GetObjects)

    def execute(self, userdata):     
 

        for i in range(10): 
            print "find object try: ", i
            
            try:
                rospy.wait_for_service(self.object_finder_srv_name, 15)
                resp = self.object_finder_srv()
            except Exception, e:  
                rospy.logerr("service call %s failed", self.object_finder_srv_name)         
        

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
                # TODO: fix in perception and remove the lines below then
                resp.objects[0].pose.header.frame_id = '/openni_rgb_optical_frame'


                print "frame_id:",resp.objects[0].pose.header.frame_id
                print "cluster_id:",resp.objects[0].cluster.header.frame_id
                tf_listener.waitForTransform(resp.objects[0].pose.header.frame_id, '/odom', resp.objects[0].pose.header.stamp, rospy.Duration(2))
                tf_wait_worked = True
            except Exception, e:
                print "tf exception in recognize_objects: wait for transform: ", e
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
                    # TODO: fix in perception and remove the lines below then
                    obj.pose.header.frame_id = '/openni_rgb_optical_frame'


                    obj.pose = tf_listener.transformPose('/odom', obj.pose)
                    transformed_poses.append(obj.pose)
                    tf_worked = True
                except Exception, e:
                    print "tf exception in recognize_objects: transform pose: ", e
                    tf_worked = False

        userdata.recognized_objects = transformed_poses

        print "################ OBJECTS TAKEN: ", len(userdata.recognized_objects)

        return 'succeeded'

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
import hbrs_msgs.msg

from hbrs_srvs.srv import GetObjects


class find_drawer(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['found_drawer', 'no_drawer_found', 'srv_call_failed'], output_keys=["drawer_pose_list"])
        
        self.drawer_finder_srv_name = '/hbrs_perception/detect_marker'
        self.drawer_finder_srv = rospy.ServiceProxy(self.drawer_finder_srv_name, hbrs_srvs.srv.GetObjects)
        
    def execute(self, userdata): 
        
        try:
            rospy.wait_for_service(self.drawer_finder_srv_name, 15)
            resp = self.drawer_finder_srv()
        except Exception, e:
            rospy.logerr("could not execute service <<%s>>: %e", self.drawer_finder_srv_name, e)
            return 'srv_call_failed'
            
        if (len(resp.objects) <= 0):
            rospy.logerr('found no drawer')
            return 'no_drawer_found'
        
        rospy.loginfo('found {0} drawers'.format(len(resp.objects)))
        
        userdata.drawer_pose_list = resp.objects
        
        return 'found_drawer'


class detect_object(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed'],
            output_keys=['object_list'])
        
        self.object_finder_srv = rospy.ServiceProxy('/hbrs_object_finder/get_segmented_objects', hbrs_srvs.srv.GetObjects)

    def execute(self, userdata):     
        #get object pose list
        rospy.wait_for_service('/hbrs_object_finder/get_segmented_objects', 30)

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

    #DETECT_SERVER = '/detect_objects'
    DETECT_SERVER = '/hbrs_object_finder/get_segmented_objects'

    def __init__(self, retries=5):
        smach.State.__init__(self,
                             outcomes=['objects_found',
                                       'no_objects_found',
                                       'srv_call_failed'],
                             input_keys=['recognized_objects'],
                             output_keys=['recognized_objects'])
        self.detect_objects = rospy.ServiceProxy(self.DETECT_SERVER, GetObjects)
        self.tf_listener = tf.TransformListener()
        self.retries = retries

    def execute(self, userdata):

        for i in range(self.retries):
            rospy.loginfo('Looking for objects (attempt %i/%i)' % (i + 1, self.retries))
            try:
                rospy.wait_for_service(self.DETECT_SERVER, 15)
                resp = self.detect_objects()
            except Exception as e:
                rospy.logerr("Service call to <<%s>> failed", self.DETECT_SERVER)
                return 'srv_call_failed'
            if not resp.objects:
                rospy.loginfo('Found no objects')
            else:
                rospy.loginfo('Found %i objects' % len(resp.objects))
                break

        if not resp.objects:
            rospy.loginfo('No objects in the field of view')
            return 'no_objects_found'

        #transform to odom
        for obj in resp.objects:
            tf_worked = False
            while not tf_worked:
                try:
                    obj.pose.header.stamp = rospy.Time.now()
                    self.tf_listener.waitForTransform('/odom', obj.pose.header.frame_id, rospy.Time.now(), rospy.Duration(5))
                    obj.pose = self.tf_listener.transformPose('/odom', obj.pose)
                    tf_worked = True
                except Exception, e:
                    rospy.logerr("Tf exception in recognize objects: %s", e)
                    rospy.sleep(0.2)

        userdata.recognized_objects = resp.objects

        return 'objects_found'

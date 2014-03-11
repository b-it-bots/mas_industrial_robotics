#!/usr/bin/python

import rospy
import smach
import smach_ros
##import hbrs_srvs.srv
import std_srvs.srv
import tf
import geometry_msgs.msg
##import hbrs_msgs.msg

##from hbrs_srvs.srv import GetObjects, ReturnBool
##from raw_srvs.srv import FindHoles
##from raw_msgs.msg import Hole
##from raw_srvs.srv import DoVisualServoing
##from raw_msgs.msg import VisualServoing

from mcr_perception_msgs.srv import GetObjectList
from mir_controller_msgs.srv import StartVisualServoing

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

#FIXME: is this still used? not in BTT
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

#FIXME: What is the difference between detect_objects and find_objects
class find_objects(smach.State):

    DETECT_SERVER = '/mcr_perception/detect_objects'

    def __init__(self, retries=5, frame_id=None):
        smach.State.__init__(self,
                             outcomes=['objects_found',
                                       'no_objects_found',
                                       'srv_call_failed'],
                             input_keys=['found_objects'],
                             output_keys=['found_objects'])
        self.detect_objects = rospy.ServiceProxy(self.DETECT_SERVER, GetObjectList)
        self.tf_listener = tf.TransformListener()
        self.retries = retries
        self.frame_id = frame_id

    def execute(self, userdata):
        userdata.found_objects = None
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
                n = str([obj.name for obj in resp.objects])
                rospy.loginfo('Found %i objects: %s' % (len(resp.objects), n))
                break

        if not resp.objects:
            rospy.loginfo('No objects in the field of view')
            return 'no_objects_found'

        if self.frame_id:
            for obj in resp.objects:
                try:
                    obj.pose = self.tf_listener.transformPose(self.frame_id, obj.pose)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.logerr('Unable to transform %s -> %s' % (obj.pose.header.frame_id, self.frame_id))

        userdata.found_objects = resp.objects
        return 'objects_found'


class find_holes(smach.State):

    SERVER = '/find_holes'

    def __init__(self, retries=3, frame_id=None):
        smach.State.__init__(self,
                             outcomes=['holes_found',
                                       'no_holes_found',
                                       'srv_call_failed'],
                             input_keys=['found_holes', 'simulation'],
                             output_keys=['found_holes'])
        self.find_holes = rospy.ServiceProxy(self.SERVER, FindHoles)
        self.tf_listener = tf.TransformListener()
        self.retries = retries
        self.frame_id = frame_id

    def execute(self, userdata):
        if userdata.simulation:
            return self.simulated_holes()
        userdata.found_holes = None
        for i in range(self.retries):
            rospy.loginfo('Looking for holes (attempt %i/%i)' % (i + 1, self.retries))
            try:
                rospy.wait_for_service(self.SERVER, 15)
                resp = self.find_holes()
            except Exception as e:
                rospy.logerr("Service call to <<%s>> failed", self.SERVER)
                return 'srv_call_failed'
            if not resp.holes:
                rospy.loginfo('Found no holes')
            else:
                rospy.loginfo('Found %i holes' % len(resp.holes))
                break

        if not resp.holes:
            rospy.loginfo('No holes in the field of view')
            return 'no_holes_found'

        if self.frame_id:
            for hole in resp.holes:
                try:
                    hole.position = self.tf_listener.transformPoint(self.frame_id, hole.position)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.logerr('Unable to transform %s -> %s' % (hole.position.header.frame_id, self.frame_id))

        userdata.found_holes = resp.holes
        return 'holes_found'

    def simulated_holes(self):
        holes = list()
        for p in [(0.0226904768497, 0.151216566563, 1.08289813995),
                  (0.13507245481, 0.125635072589, 1.12861084938),
                  (0.208152621984, 0.263396263123, 0.849790096283),
                  (0.0932492688298, 0.279482185841, 0.823174893856)]:
            h = Hole()
            pp = h.position.position
            pp.x, pp.y, pp.z = p
            h.header.stamp = rospy.Time.now()
            h.header.frame_id = '/tower_cam3d_rgb_optical_frame'
            holes.append(h)
        return holes


class do_visual_servoing(smach.State):

    SERVER = '/mir_controllers/visual_servoing/do_visual_servoing'

    def __init__( self ):
        smach.State.__init__( self,
                              outcomes=[ 'succeeded', 'failed', 'timeout', 'lost_object' ],
                              input_keys=['simulation'] )
        self.do_vs = rospy.ServiceProxy( self.SERVER, StartVisualServoing )

    def execute( self, userdata ):
        #if( userdata.simulation = True ):
        #   return 'succeeded'
        try:
            rospy.loginfo( "Calling service <<%s>>" % self.SERVER )
            response = self.do_vs()
        except rospy.ServiceException as e:
            rospy.logerr( "Exception when calling service <<%s>>: %s" % ( self.SERVER, str( e ) ) )
            return 'failed'
        if( response.return_value.error_code == 0 ):
            return 'succeeded'
        elif( response.return_value.error_code == -1 ):
            return 'failed'
        elif( response.return_value.error_code == -2 ):
            return 'timeout' 
        elif( response.return_value.error_code == -3 ):
            return 'lost_object' 

#!/usr/bin/python

import rospy
import smach
import std_msgs.msg
import tf

import mcr_perception_msgs.msg
import mcr_perception_msgs.srv
import mir_controller_msgs.srv


class find_objects(smach.State):

    OBJECT_LIST_TOPIC = '/mcr_perception/object_detector/object_list'
    EVENT_IN_TOPIC = '/mcr_perception/object_detector/event_in'
    EVENT_OUT_TOPIC = '/mcr_perception/object_detector/event_out'

    def __init__(self, retries=5, frame_id=None):
        smach.State.__init__(self,
                             outcomes=['objects_found',
                                       'no_objects_found'],
                             input_keys=['found_objects'],
                             output_keys=['found_objects'])
        self.object_list_sub = rospy.Subscriber(self.OBJECT_LIST_TOPIC, mcr_perception_msgs.msg.ObjectList, self.object_list_cb)
        self.event_out_sub = rospy.Subscriber(self.EVENT_OUT_TOPIC, std_msgs.msg.String, self.event_out_cb)
        self.event_in_pub = rospy.Publisher(self.EVENT_IN_TOPIC, std_msgs.msg.String)
        self.tf_listener = tf.TransformListener()
        self.retries = retries
        self.frame_id = frame_id

    def object_list_cb(self, event):
        self.object_list = event

    def event_out_cb(self, event):
        self.event_msg = event.data

    def execute(self, userdata):
        userdata.found_objects = None
        for i in range(self.retries):
            self.object_list = None
            self.event_msg = ""

            rospy.loginfo('Looking for objects (attempt %i/%i)' % (i + 1, self.retries))
            self.event_in_pub.publish("e_trigger")

            timeout = rospy.Duration.from_sec(10.0) # wait max of 10.0 seconds
            start_time = rospy.Time.now()

            while(True):
                if self.event_msg == "e_done" and self.object_list is not None:
                    break
                elif self.event_msg == "e_failed":
                    rospy.loginfo('Found no objects')
                    break
                elif (rospy.Time.now() - start_time) > timeout:
                    rospy.logerr('Timeout of %f seconds exceeded waiting for object_detector' % float(timeout.to_sec()))
                    break
                rospy.sleep(0.1)

            if not self.object_list or len(self.object_list.objects) <= 0:
                rospy.loginfo('Found no objects')
            else:
                n = str([obj.name for obj in self.object_list.objects])
                rospy.loginfo('Found %i objects: %s' % (len(self.object_list.objects), n))
                break

        if not self.object_list  or len(self.object_list.objects) <= 0:
            rospy.loginfo('No objects in the field of view')
            return 'no_objects_found'

        if self.frame_id:
            for obj in self.object_list.objects:
                try:
                    obj.pose = self.tf_listener.transformPose(self.frame_id, obj.pose)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.logerr('Unable to transform %s -> %s' % (obj.pose.header.frame_id, self.frame_id))

        userdata.found_objects = self.object_list.objects
        return 'objects_found'


class do_visual_servoing(smach.State):

    SERVER = '/mir_controllers/visual_servoing/do_visual_servoing'

    def __init__( self ):
        smach.State.__init__( self,
                              outcomes=[ 'succeeded', 'failed', 'timeout', 'lost_object' ],
                              input_keys=['vscount'],
                              output_keys=['vscount'])
        self.do_vs = rospy.ServiceProxy( self.SERVER, mir_controller_msgs.srv.StartVisualServoing )

    def execute( self, userdata ):
        try:
            rospy.loginfo( "Calling service <<%s>>" % self.SERVER )
            response = self.do_vs()
        except rospy.ServiceException as e:
            userdata.vscount = 0
            rospy.logerr( "Exception when calling service <<%s>>: %s" % ( self.SERVER, str( e ) ) )
            return 'failed'
        if( response.return_value.error_code == 0 ):
            userdata.vscount = 0
            return 'succeeded'
        elif( response.return_value.error_code == -1 ):
            return 'failed'
        elif( response.return_value.error_code == -2 ):
            return 'timeout'
        elif( response.return_value.error_code == -3 ):
            return 'lost_object'


class find_holes(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes=['found_holes','found_no_holes','timeout'],
            input_keys=['all_found_holes'],
            output_keys=['all_found_holes'])

        self.pub_find_holes_event = rospy.Publisher('/mcr_perception/hole_detection/event_in', std_msgs.msg.String, latch=True)
        self.sub_find_holes_event = rospy.Subscriber('/mcr_perception/hole_detection/event_out', std_msgs.msg.String, self.hole_detection_event_cb)
        self.sub_found_holes = rospy.Subscriber('/mcr_perception/hole_detection/detected_holes', mcr_perception_msgs.msg.ObjectList, self.found_hole_detection_cb)

        self.hole_detection_event = None
        self.all_found_holes = None

    def hole_detection_event_cb(self, event):
        self.hole_detection_event = event.data

    def found_hole_detection_cb(self, msg):
        self.all_found_holes = msg.objects

    def execute(self, userdata):

        self.hole_detection_event = None
        self.all_found_holes = None

        # publish event to start the movement
        self.pub_find_holes_event.publish(std_msgs.msg.String('e_trigger'))

        timeout = rospy.Duration.from_sec(5.0)  #wait for the done event max. 5 seconds
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():

            if self.hole_detection_event and self.hole_detection_event == 'e_done':
                rospy.loginfo('got done event and detections from hole detection')
                userdata.all_found_holes = self.all_found_holes

                if (len(self.all_found_holes) > 0):
                    return 'found_holes'
                else:
                    return 'found_no_holes'

            if (rospy.Time.now() - start_time) > timeout:
                rospy.logerr('timeout of %f seconds exceeded for relative base movement' % float(timeout.to_sec()))
                return 'timeout'

            rospy.sleep(0.1)

        return 'found_no_holes'

class find_cavities(smach.State):
    def __init__(self, frame_id=None):
        smach.State.__init__(self,
            outcomes=['succeeded','not_all_cavities_found', 'timeout'],
            input_keys=['selected_objects', 'found_cavities'],
            output_keys=['found_cavities'])

        self.sub_cavity = rospy.Subscriber('/mcr_perception/cavity_message_builder/output/cavity', mcr_perception_msgs.msg.Cavity, self.cavity_cb)
        self.pub_contour_finder_event = rospy.Publisher('/mcr_perception/contour_finder/input/event_in', std_msgs.msg.String)
        self.pub_object_category = rospy.Publisher('/mcr_perception/cavity_template_publisher/input/object_name', std_msgs.msg.String)
        self.tf_listener = tf.TransformListener()
        self.cavity = None
        self.frame_id = frame_id

    def cavity_cb(self, cavity):
        self.cavity = cavity

    def execute(self, userdata):
        userdata.found_cavities = []
        for obj in userdata.selected_objects:
            self.cavity = None
            self.pub_object_category.publish(obj.name)
            self.pub_contour_finder_event.publish("e_trigger")

            timeout = rospy.Duration.from_sec(5.0)  #wait for the done event max. 5 seconds
            start_time = rospy.Time.now()
            while not rospy.is_shutdown():
                if self.cavity:
                    rospy.loginfo('Received Cavity message for %s', obj.name)
                    self.cavity.object_name = obj.name
                    userdata.found_cavities.append(self.cavity)
                    break

                if (rospy.Time.now() - start_time) > timeout:
                    rospy.logerr('timeout of %f seconds exceeded for finding cavity' % float(timeout.to_sec()))
                    return 'timeout'

                rospy.sleep(0.1)

        if self.frame_id:
            for cavity in userdata.found_cavities:
                try:
                    cavity.pose = self.tf_listener.transformPose(self.frame_id, cavity.pose)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.logerr('Unable to transform %s -> %s' % (cavity.pose.header.frame_id, self.frame_id))

        if len(userdata.found_cavities) == len(userdata.selected_objects):
            return 'succeeded'
        else:
            return 'not_all_cavities_found'

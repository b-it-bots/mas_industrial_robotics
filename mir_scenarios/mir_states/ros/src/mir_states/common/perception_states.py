#!/usr/bin/python

import rospy
import smach
import std_msgs.msg
import tf

import mas_perception_msgs.msg
import mir_controller_msgs.srv


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



class find_cavities(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes=['succeeded', 'failed'],
            input_keys=['selected_objects', 'found_cavities'],
            output_keys=['found_cavities'])

        self.sub_cavity = rospy.Subscriber('/mcr_perception/cavity_message_builder/output/cavity', mas_perception_msgs.msg.Cavity, self.cavity_cb)
        self.pub_contour_finder_event = rospy.Publisher('/mcr_perception/contour_finder/input/event_in', std_msgs.msg.String)
        self.pub_object_category = rospy.Publisher('/mcr_perception/cavity_template_publisher/input/object_name', std_msgs.msg.String)
        self.tf_listener = tf.TransformListener()
        self.cavity = None

    def cavity_cb(self, cavity):
        self.cavity = cavity

    def execute(self, userdata):
        local_found_cavities = []
        for obj in userdata.selected_objects:
            self.cavity = None
            self.pub_object_category.publish(obj.name)
            self.pub_contour_finder_event.publish("e_trigger")

            timeout = rospy.Duration.from_sec(5.0)  #wait for the done event max. 5 seconds
            start_time = rospy.Time.now()
            while not rospy.is_shutdown():
                if self.cavity:
                    rospy.loginfo('Received Cavity message for %s, matching error: %.5f', obj.name, self.cavity.template_matching_error.matching_error)
                    self.cavity.object_name = obj.name
                    local_found_cavities.append(self.cavity)
                    break

                if (rospy.Time.now() - start_time) > timeout:
                    rospy.logwarn('timeout of %f seconds exceeded for finding cavity' % float(timeout.to_sec()))
                    break

                rospy.sleep(0.01)

        if len(local_found_cavities) == 0:
            return 'failed'

        userdata.found_cavities = local_found_cavities

        return 'succeeded'

class check_found_cavities(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes=['cavities_found','no_cavities_found'],
            input_keys=['best_matched_cavities'])

    def execute(self, userdata):

        if len(userdata.best_matched_cavities) == 0:
            return 'no_cavities_found'
        else:
            return 'cavities_found'

class find_best_matched_cavities(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes=['succeeded', 'complete'],
            input_keys=['best_matched_cavities', 'found_cavities'],
            output_keys=['best_matched_cavities'])

        self.matching_threshold = 0.1
        self.loop_count = 0

    def execute(self, userdata):

        for cavity in userdata.found_cavities:
            exists = False
            for idx,c in enumerate(userdata.best_matched_cavities):
                if cavity.object_name == c.object_name:
                    if cavity.template_matching_error.matching_error < c.template_matching_error.matching_error:
                        userdata.best_matched_cavities[idx] = cavity
                        print "Found better cavity for ", cavity.object_name, ". Old: ", c.template_matching_error.matching_error, " New: ", cavity.template_matching_error.matching_error
                    exists = True

            if exists == False:
                if cavity.template_matching_error.matching_error < self.matching_threshold:
                    userdata.best_matched_cavities.append(cavity)

        if self.loop_count >= 2:
            self.loop_count = 0
            return 'complete'
        self.loop_count += 1
        return 'succeeded'


#!/usr/bin/python

import rospy
import smach
import std_msgs.msg
import tf

import mcr_perception_msgs.msg
import mcr_perception_msgs.srv
import mir_controller_msgs.srv

import random


class find_objects(smach.State):

    OBJECT_LIST_TOPIC = '/mcr_perception/object_detector/object_list'
    EVENT_IN_TOPIC = '/mcr_perception/object_detector/event_in'
    EVENT_OUT_TOPIC = '/mcr_perception/object_detector/event_out'

    def __init__(self, retries=5):
        smach.State.__init__(self,
                             outcomes=['objects_found',
                                       'no_objects_found'],
                             input_keys=['found_objects'],
                             output_keys=['found_objects'])
        self.object_list_sub = rospy.Subscriber(self.OBJECT_LIST_TOPIC, mcr_perception_msgs.msg.ObjectList, self.object_list_cb)
        self.event_out_sub = rospy.Subscriber(self.EVENT_OUT_TOPIC, std_msgs.msg.String, self.event_out_cb)
        self.event_in_pub = rospy.Publisher(self.EVENT_IN_TOPIC, std_msgs.msg.String)
        self.retries = retries

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



class find_cavities(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes=['succeeded', 'failed'],
            input_keys=['selected_objects', 'found_cavities'],
            output_keys=['found_cavities'])

        self.sub_cavity = rospy.Subscriber('/mcr_perception/cavity_message_builder/output/cavity', mcr_perception_msgs.msg.Cavity, self.cavity_cb)
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

                rospy.sleep(0.1)

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




class accumulate_recognized_objects_list(smach.State):
    def __init__(self, loop=3):
        '''
        Youbot views the workbench from different angles and finds the objects.
        The function then accumulates the list together to remove any duplicates.
        '''
        smach.State.__init__( self,
                outcomes=[ 'complete', 'merged'],
                input_keys=['recognized_objects'],
                output_keys=['recognized_objects'])

        self.loop = loop        #Number of times the accumulate is called
        self.loop_counter = 0

        #List containing the union of all the recognized objects from different views
        self.overall_recognized_objects = list()

    def execute(self, userdata):
        self.similarity_distance_threshold = 0.02
        if not self.overall_recognized_objects and userdata.recognized_objects:
            self.overall_recognized_objects = list(userdata.recognized_objects)
        elif self.overall_recognized_objects and userdata.recognized_objects:
            new_objects = list()
            old_objects = list()
            for recognized_object in userdata.recognized_objects:
                duplicate_found  = False

                for overall_object in self.overall_recognized_objects[:]:
                    diff_x = abs(overall_object.pose.pose.position.x -
                            recognized_object.pose.pose.position.x)
                    diff_y = abs(overall_object.pose.pose.position.y - 
                            recognized_object.pose.pose.position.y)
                    diff_z = abs(overall_object.pose.pose.position.z - 
                            recognized_object.pose.pose.position.z)
                    if diff_x < self.similarity_distance_threshold and  \
                            diff_y < self.similarity_distance_threshold and  \
                            diff_z < self.similarity_distance_threshold:
                                #Check for probability and add which duplicate to append
                        if overall_object.probability < recognized_object.probability:
                            # the new object has higher probability then
                            duplicate_found  = False
                            self.overall_recognized_objects.remove(overall_object)
                        else :
                            duplicate_found  = True

                if duplicate_found == False:
                    print "Adding object : ",recognized_object.name, " to list "
                    new_objects.append(recognized_object)
                    duplicate_found = True

            self.overall_recognized_objects.extend(new_objects) 
        userdata.recognized_objects = []



        self.loop_counter = self.loop_counter + 1
        if (self.loop_counter == self.loop):
            #Accumulate has been called for n times.
            # copying the local union list to userdata
            userdata.recognized_objects = list(self.overall_recognized_objects)

            #Reset counter and list
            self.loop_counter = 0
            self.overall_recognized_objects = list()

            rospy.loginfo("(before shuffle) MERGED OBJECT LIST : %s" 
                    % ([obj.name for obj in userdata.recognized_objects]) )
            if userdata.recognized_objects:
                random.shuffle(userdata.recognized_objects)
            print "FINAL LIST : ", [obj.name for obj in userdata.recognized_objects]
            rospy.loginfo("FINAL MERGED OBJECT LIST : %s" 
                    % ([obj.name for obj in userdata.recognized_objects]) )
            return 'complete'
        else :
            return 'merged'



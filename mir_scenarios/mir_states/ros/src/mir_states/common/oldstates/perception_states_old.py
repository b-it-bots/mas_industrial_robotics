#!/usr/bin/python

import rospy
import smach
import smach_ros
import hbrs_srvs.srv
import std_srvs.srv
import tf 
import geometry_msgs.msg
import hbrs_msgs.msg



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

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['found_objects', 'no_objects_found', 'srv_call_failed'],
            input_keys=['recognized_objects'],
            output_keys=['recognized_objects'])
        
        self.object_finder_srv_name = '/detect_objects'
        self.object_finder_srv = rospy.ServiceProxy(self.object_finder_srv_name, hbrs_srvs.srv.GetObjects)
        self.tf_listener = tf.TransformListener()

    def execute(self, userdata):     
         

        for i in range(5): 
            print "find object try: ", i
            
            try:
                rospy.wait_for_service(self.object_finder_srv_name, 15)
                resp = self.object_finder_srv()
            except Exception, e:  
                rospy.logerr("service call %s failed", self.object_finder_srv_name)     
                return 'srv_call_failed'    
        

            if (len(resp.objects) <= 0):
                rospy.loginfo('found no objects')
            else:    
                rospy.loginfo('found {0} objects'.format(len(resp.objects)))
                break

        if len(resp.objects) == 0:
            rospy.loginfo('NO objects in FOV')
            return 'no_objects_found'

        #print resp.objects

        #transform to odom
        for i in range(len(resp.objects)):
            tf_worked = False
            while not tf_worked:
                try:
                    print "frame_id: ", resp.objects[i].pose.header.frame_id
                    resp.objects[i].pose.header.stamp = rospy.Time.now()
                    self.tf_listener.waitForTransform('/odom', resp.objects[i].pose.header.frame_id, rospy.Time.now(), rospy.Duration(5))
                    obj_pose_transformed = self.tf_listener.transformPose('/odom', resp.objects[i].pose)
                    resp.objects[i].pose = obj_pose_transformed
                    print resp.objects[i].name
                    print resp.objects[i].pose
                    tf_worked = True
                except Exception, e:
                    rospy.logerr("tf exception in recognize objects: transform: %s", e)
                    rospy.sleep(0.2)
                    tf_worked = False

        userdata.recognized_objects = resp.objects

        print "################ OBJECTS TAKEN: ", len(userdata.recognized_objects)

        '''
        #obj_names = ['screw1', 'nut1']
        for i in range(2):
            obj_name = raw_input("object_name: ")
            
            obj = hbrs_msgs.msg.Object()
            obj.name = obj_name
            
            userdata.recognized_objects.append(obj)
        

        '''
        
        return 'found_objects'

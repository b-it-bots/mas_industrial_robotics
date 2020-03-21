#from Tkinter import *
#!/usr/bin/env python

import os
import rospy
import rospkg
import sys
import Tkinter as tk
import re

from rosplan_dispatch_msgs.msg import *
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *

class VisualizeWorldModel():
    def __init__(self):

        #from the export image of inkscape
        self.canvas_width = 1000
        self.canvas_height = 700

        self.master = tk.Tk()

        self.canvas = tk.Canvas(self.master, 
                width=self.canvas_width, 
                height=self.canvas_height)
        self.canvas.pack()

        rect = tk.PhotoImage(file="rect.gif")
        conveyor = tk.PhotoImage(file="conveyor.gif")
        youbot = tk.PhotoImage(file="youbot.gif")
        entry = tk.PhotoImage(file="entry.gif")

        self.locations = {}
        self.objects_on_location = {}
        self.goals_from_knowledge_base = {}

        ##########################################################
        # Workspace details
        # @todo : move the below files as config file to load
        ##########################################################

        self.locations['SH-01'] = [100, 100, "Shelf 01"]
        self.locations['SH-02'] = [320, 100, "Shelf 02"]
        self.locations['SH-03'] = [530, 100, "Shelf 03"]
        self.locations['SH-04'] = [830, 100, "Shelf 03"]
        self.locations['WS-01'] = [820, 250, "Drilling Workstation"]
        self.locations['WS-03'] = [750, 100, "Force Fitting Workstation"]
        self.locations['CB-01'] = [100, 250, "Conveyor Belt"]
        self.locations['WS-05'] = [200, 400, "Work Station "]
        self.locations['WS-06'] = [500, 500, "Work Station "]
        self.locations['WS-07'] = [800, 400, "Work Station "]
        #self.locations['entry'] = [800, 250, "Entry"]
        self.locations['yb'] = [500, 250, "Youbot"]

        #creating empty list of objects
        for key,loc in self.locations.iteritems():
            self.objects_on_location[key] = [""]

        for key,loc in self.locations.iteritems():
            #Adding location for putting text
            self.locations[key].append(loc[0] + 40)
            self.locations[key].append(loc[1] + 20)

        self.canvas.create_image(self.locations['SH-01'][0],self.locations['SH-01'][1], anchor=tk.NW, image=rect)
        self.canvas.create_image(self.locations['SH-02'][0],self.locations['SH-02'][1], anchor=tk.NW, image=rect)
        self.canvas.create_image(self.locations['SH-03'][0],self.locations['SH-03'][1], anchor=tk.NW, image=rect)
        self.canvas.create_image(self.locations['SH-04'][0],self.locations['SH-04'][1], anchor=tk.NW, image=rect)
        self.canvas.create_image(self.locations['WS-01'][0],self.locations['WS-01'][1], anchor=tk.NW, image=rect)
        self.canvas.create_image(self.locations['WS-03'][0],self.locations['WS-03'][1], anchor=tk.NW, image=rect)
        self.canvas.create_image(self.locations['WS-05'][0],self.locations['WS-05'][1], anchor=tk.NW, image=rect)
        self.canvas.create_image(self.locations['WS-06'][0],self.locations['WS-06'][1], anchor=tk.NW, image=rect)
        self.canvas.create_image(self.locations['WS-07'][0],self.locations['WS-07'][1], anchor=tk.NW, image=rect)
        self.canvas.create_image(self.locations['CB-01'][0],self.locations['CB-01'][1], anchor=tk.NW, image=conveyor)
        #self.canvas.create_image(self.locations['entry'][0],self.locations['entry'][1], anchor=tk.NW, image=entry)
        self.canvas.create_image(self.locations['yb'][0],self.locations['yb'][1], anchor=tk.NW, image=youbot)

        #Writing names to the images
        for key,loc in self.locations.iteritems():
            self.canvas.create_text(loc[0], loc[1], text=key+" "+loc[2], anchor=tk.NW, fill="blue")
            text_id = self.canvas.create_text(loc[3], loc[4], text="", anchor=tk.NW)
            self.locations[key].append(text_id)
            text_id = self.canvas.create_text(loc[3], loc[4], text="", anchor=tk.NW)
            self.locations[key].append(text_id)

        self.master.after(50, self.update_text)
        self.master.mainloop()

    def query_database_facts(self):
        #empty list of objects
        for key,loc in self.locations.iteritems():
            self.objects_on_location[key] = [""]
        try:
            model_client = rospy.ServiceProxy('/kcl_rosplan/get_current_knowledge', GetAttributeService)
            resp = model_client('')
            for attribute in resp.attributes:
                if 'on' == attribute.attribute_name:
                    location = attribute.values[1].value
                    object = attribute.values[0].value
                    print "location : ", location
                    regex_sh_01 = re.compile("SH-0[1-6]")
                    regex_sh_02 = re.compile("SH-0[7-9]|SH-1[0-2]")
                    regex_sh_03 = re.compile("SH-1[3-8]")
                    regex_sh_04 = re.compile("SH-19]|SH-2[0-4]")
                    if(regex_sh_01.match(location)):
                        print 'regex match 01'
                        location = "SH-01"
                    elif(regex_sh_02.match(location)):
                        print 'regex match 02'
                        location = "SH-02"
                    elif(regex_sh_03.match(location)):
                        print 'regex match 03'
                        location = "SH-03"
                    elif(regex_sh_04.match(location)):
                        print 'regex match 04'
                        location = "SH-04"
                    self.objects_on_location[location].append(object)
            # First for loop stores all objects location
            # Second for loop for storing objects in other locations
            for attribute in resp.attributes:
                if 'in' == attribute.attribute_name:
                    container = attribute.values[1].value
                    object = attribute.values[0].value
                    #Finding the location of the container
                    for location, objects in self.objects_on_location.iteritems():
                        if container in objects:
                            self.objects_on_location[location].append(object)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def query_database_goals(self):
        #empty list of objects
        for key,loc in self.locations.iteritems():
            self.goals_from_knowledge_base[key] = [""]
        try:
            model_client = rospy.ServiceProxy('/kcl_rosplan/get_current_goals', GetAttributeService)
            resp = model_client('')
            for attribute in resp.attributes:
                if 'on' == attribute.attribute_name:
                    location = attribute.values[1].value
                    object = attribute.values[0].value
                    print "location : ", location
                    regex_sh_01 = re.compile("SH-0[1-6]")
                    regex_sh_02 = re.compile("SH-0[7-9]|SH-1[0-2]")
                    regex_sh_03 = re.compile("SH-1[3-8]")
                    regex_sh_04 = re.compile("SH-19]|SH-2[0-4]")
                    if(regex_sh_01.match(location)):
                        print 'regex match 01'
                        location = "SH-01"
                    elif(regex_sh_02.match(location)):
                        print 'regex match 02'
                        location = "SH-02"
                    elif(regex_sh_03.match(location)):
                        print 'regex match 03'
                        location = "SH-03"
                    elif(regex_sh_04.match(location)):
                        print 'regex match 04'
                        location = "SH-04"
                    self.goals_from_knowledge_base[location].append(object)
            # First for loop stores all objects location
            # Second for loop for storing objects in other locations
            for attribute in resp.attributes:
                if 'in' == attribute.attribute_name:
                    container = attribute.values[1].value
                    object = attribute.values[0].value
                    #Finding the location of the container
                    for location, objects in self.goals_from_knowledge_base.iteritems():
                        if container in objects:
                            self.goals_from_knowledge_base[location].append(object)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def update_text(self):
        #self.canvas.itemconfig(self.locations['WS-05'][5], text= "update text")
        self.query_database_facts()
        self.query_database_goals()
        for key,loc in self.locations.iteritems():
            self.canvas.itemconfig(loc[5], text=('\n'.join(self.objects_on_location[key])), anchor=tk.NW)
            self.canvas.itemconfig(loc[6], text=('\n'.join(self.goals_from_knowledge_base[key])), anchor=tk.NW, fill='red')

        self.master.after(500, self.update_text)


if __name__ == "__main__":
    wm = VisualizeWorldModel()

(define (domain general_domain)
 (:requirements :typing :action-costs)
 (:types
  	location      		; service areas, points of interest, navigation goals
  	robot         		; your amazing yet powerful robot
  	object				    ; objects to be manipulated by the robot
  	robot_platform		; platform slots for the robot to store objects
 )

 (:predicates

	; robot ?r is at location ?l
 	(at ?r - robot ?l - location)

 	; object ?o is on location ?l
 	(on ?o - object ?l - location)

 	; object ?o is stored on robot platform ?rp
 	(stored ?o - object ?rp - robot_platform)

 	; robot platform ?rp is occupied, yb has 3 free places to store objects
	(occupied ?rp - robot_platform)

	; gripper ?g is holding object ?o
	(holding ?r - robot ?o - object)

	; gripper ?g is free (does not contain object)
	(gripper_is_free ?r - robot)

	; object ?o is able to hold other objects
	(container ?o - object)

	; object ?o is inserted inside object ?h
	(in ?peg - object ?hole - object)

	; the object ?o is heavy and cannot be lifted by the robot
	(heavy ?o - object)

	; specifies if an object ?o can be inserted into another object
	(insertable ?o - object)

	; an object ?o is perceived when object recognition was triggered
	; gets lost if the robot moves the base
	(perceived ?l - location)

 )

 (:functions
  	(total-cost) - number
 )

; moves a robot ?robot from ?source - location to a ?destination - location
; NOTE : the situation in which the robot arm is in any position before moving
; is not handled at the planning level, hence we advise to always move the arm
; to a folded position, then navigate
 (:action move_base
    :parameters (?robot - robot ?source ?destination - location)
    :precondition (and (at ?robot ?source)
     					    (gripper_is_free ?robot)
     			  )
    :effect (and (not (at ?robot ?source))
     			        (at ?robot ?destination)
     			        (not (perceived ?source))
            	    	(increase (total-cost) 20)
     		 )
 )

 ; perceive an object ?object which is in a location ?location with a empty gripper ?g
 ; to find the pose of this object before it can be picked
 (:action perceive
   :parameters (?robot - robot ?location - location)
   :precondition 	(and 	(at ?robot ?location)
   							(gripper_is_free ?robot)
   							(not (perceived ?location))
   					)
   :effect 	(and 	(perceived ?location)
                    (increase (total-cost) 10)
  			)
 )

 ; pick an object ?object which is inside a location ?location with a free gripper ?g
 ; with robot ?robot that is at location ?location
 ; (:action pick
 (:action pick
     :parameters (?robot - robot ?location - location ?object - object)
     :precondition 	(and 	(on ?object ?location)
                      		(at ?robot ?location)
                      		(perceived ?location)
                      		(gripper_is_free ?robot)
                      		(not (holding ?robot ?object))
                      		(not (heavy ?object))
                   	)
     :effect (and  	(holding ?robot ?object)
                   	(not (on ?object ?location))
                   	(not (gripper_is_free ?robot))
                   	(increase (total-cost) 2)
             )
 )

 (:action place
     :parameters (?robot - robot ?location - location ?object - object)
     :precondition  (and  (at ?robot ?location)
                          (holding ?robot ?object)
                          (not (on ?object ?location))
                          (not (insertable ?object ))
                          (not (gripper_is_free ?robot))
                    )
     :effect (and   (on ?object ?location)
                    (not (holding ?robot ?object))
                    (gripper_is_free ?robot)
                    (increase (total-cost) 2)
             )
 )


 ; stage an object ?object in a robot platform ?platform which is not occupied with a gripper ?g
 ; which is holding the object ?object
 (:action stage
     :parameters (?robot - robot ?platform - robot_platform ?object - object)
     :precondition 	(and 	(holding ?robot ?object)
                      		(not (occupied ?platform))
                            (not (gripper_is_free ?robot))
                   	)
     :effect (and  	(not (holding ?robot ?object))
	 				     (gripper_is_free ?robot)
     			   	     (stored ?object ?platform)
                   	     (occupied ?platform)
                   	     (increase (total-cost) 1)
             )
 )

 ; unstage an object ?object stored on a robot platform ?platform with a free gripper ?g
 (:action unstage
     :parameters (?robot - robot ?platform - robot_platform ?object - object)
     :precondition 	(and 	(gripper_is_free ?robot)
                      		(stored ?object ?platform)
                      		(not (holding ?robot ?object))
                   	)
     :effect (and  	(not (gripper_is_free ?robot))
     			   	     (not (stored ?object ?platform))
                   	     (not (occupied ?platform))
                   	     (holding ?robot ?object)
                   	     (increase (total-cost) 1)
             )
 )

 ; inserts a object ?object which gripper ?g is holding into another object ?object at location ?location
 (:action insert
   :parameters (?robot - robot ?platform - robot_platform ?location - location ?peg ?hole - object   )
   :precondition 	(and 	(at ?robot ?location)
   							(on ?hole ?location)
   							(container ?hole)
   							;(holding ?g ?peg)
                      		(not (holding ?robot ?peg))
   							;(not (gripper_is_free ?g))
                            (gripper_is_free ?robot)
                            (stored ?peg ?platform)
   							(not (container ?peg)) ;a container cannot be inserted into a container
   							(perceived ?location)
   					)
   :effect 	(and 	(not (holding ?robot ?peg))
   					(gripper_is_free ?robot)
   					(in ?peg ?hole)
   					(on ?peg ?location)
                    (not (stored ?peg ?platform))
                    (not (occupied ?platform))
   					(heavy ?hole)
   					(heavy ?peg) ;it doesn't become heavy but it cannot be picked again
   					(increase (total-cost) 5)
   			)
 )
)

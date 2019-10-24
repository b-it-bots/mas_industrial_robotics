(define (domain cleaning_robot)

  (:requirements
    :typing
  )

  (:types
    location
    robot
  )

  (:predicates
    (at ?r - robot ?l - location) 	; robot r? is at location l?
    (clean ?l - location) 			; location ?l is clean
  ) 

  (:action move
    :parameters (?r - robot ?source ?destination - location)
    :precondition (at ?r ?source)
    :effect (and    ( not (at ?r ?source))
    				(at ?r ?destination)
    		)
  )

  (:action clean
    :parameters (?r - robot ?l - location)
    :precondition (and (at ?r ?l) (not(clean ?l)))
    :effect (clean ?l)
  )
)


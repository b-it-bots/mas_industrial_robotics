(define (problem problem)
	
  (:domain cleaning_robot)

  (:objects 
  	r--ghost - robot
  	l--locA l--locB - location
  
  )

  (:init 
  	(at r--ghost l--locA)
  ) 

  (:goal
  	(  and (at r--ghost l--locA)
      (clean l--locB)
      (clean l--locA)
    )
  )
 )
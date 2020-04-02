(define (problem minimum_required_facts)
(:domain robocup_at_work)
(:objects

    ; robots
    youbot-brsu - robot

    ; robot available platforms, to store objects inside the robot
    platform_middle platform_left platform_right - robot_platform

    ; locations
    START END - location

    ; objects
    ;o--o1 o--o2 o--o3 o--o4 o--o4 - object
)
(:init

    ; robot initial conditions : location
    (at youbot-brsu start) ; youbot is at start position

    ; status of the gripper at the beginning
    (gripper_is_free youbot-brsu)
    ;(on o1 s1)
    ;(on o2 s2)
    ;(on o3 s3)
    ;(on o4 s4)
    ;(on o5 s5)
)
(:goal
    (and
        ;(at youbot-brsu END)
        ;(holding youbot-brsu o1)
        ;(stored o1 platform_middle)
        ;(at youbot-brsu S5)
        ;(on o1 S5)
	    ;(on o2 S3)
        ;(on o3 S6)
    )
)
)

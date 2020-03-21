(define (problem minimum_required_facts)
(:domain robocup_at_work)
(:objects

    ; robots
    r--youbot-brsu - robot

    ; robot available platforms, to store objects inside the robot
    rp--platform_middle rp--platform_left rp--platform_right - robot_platform

    ; locations
    ;l--s1 l--s2 l--s3 l--s4 l--s5 l--s6 - location
    l--START - location
    l--END - location

    ; objects
    ;o--o1 o--o2 o--o3 o--o4 o--o4 - object
)
(:init

    ; robot initial conditions : location
    (at r--youbot-brsu l--start) ; youbot is at start position

    ; status of the gripper at the beginning
    (gripper_is_free r--youbot-brsu)
    ;(on o--o1 l--s1)
    ;(on o--o2 l--s2)
    ;(on o--o3 l--s3)
    ;(on o--o4 l--s4)
    ;(on o--o5 l--s5)
)
(:goal
    (and
        ;(at r--youbot-brsu l--END)
        ;(holding r--youbot-brsu o--o1)
        ;(stored o--o1 rp--platform_middle)
        ;(at r--youbot-brsu l--S5)
        ;(on o--o1 l--S5)
	    ;(on o--o2 l--S3)
        ;(on o--o3 l--S6)
    )
)
)

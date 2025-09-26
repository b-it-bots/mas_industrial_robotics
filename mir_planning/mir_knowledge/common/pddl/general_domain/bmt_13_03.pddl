;This PDDL problem definition was made automatically from a KB snapshot 
(define (problem general_domain_task) 
    (:domain general_domain) 

    (:objects 
        WS1 WS2 WS3 WS5 WS6 WS7 WS8 WS9 WS11 WS12 WS13 WS14 PETER SH1 SH2 TT1 PP1 START EXIT - location 
        ALLENKEY-00 AXIS2-00 BEARING2-00 DRILL-00 F20_20_B-00 F20_20_G-00 HOUSING-00 M20-00 M20_100-00 M30-00 MOTOR2-00 S40_40_B-00 S40_40_G-00 SCREWDRIVER-00 SPACER-00 WRENCH-00 CONTAINER_BOX_RED-00 CONTAINER_BOX_BLUE-00 PP01_CAVITY-00 - object 
        YOUBOT-BRSU - robot 
        PLATFORM_LEFT PLATFORM_MIDDLE PLATFORM_RIGHT - robot_platform 
    ) 

    (:init 
        ;Cost information starts 
        (= (total-cost) 0) 
        ;Cost information ends 

        (at YOUBOT-BRSU START) 
        (gripper_is_free YOUBOT-BRSU) 

        (on F20_20_G-00 WS2) 

        (on M20-00 WS2)

        (on M20_100-00 WS2)
    ) 

    (:goal 
        (and 
            (on F20_20_G-00 WS3) 

            (on M20-00 WS3)

            (on M20_100-00 WS3)
        ) 
    ) 

    (:metric minimize 
        (total-cost) 
    ) 

) 

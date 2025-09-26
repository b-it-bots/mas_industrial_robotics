;This PDDL problem definition was made automatically from a KB snapshot 
(define (problem general_domain_task) 
    (:domain general_domain) 

    (:objects 
        WS01 WS02 WS03 WS04 WS05 WS06 SH01 TT01 PP01 START EXIT - location 
        ALLENKEY-00 AXIS2-00 BEARING2-00 DRILL-00 F20_20_B-00 F20_20_G-00 HOUSING-00 M20-00 M20_100-00 M30-00 MOTOR2-00 S40_40_B-00 S40_40_G-00 SCREWDRIVER-00 SPACER-00 WRENCH-00 CONTAINER_BOX_RED CONTAINER_BOX_BLUE PP01_CAVITY-00 - object 
        YOUBOT-BRSU - robot 
        PLATFORM_LEFT PLATFORM_MIDDLE PLATFORM_RIGHT - robot_platform 
    ) 

    (:init 
        ;Cost information starts 
        (= (total-cost) 0) 
        ;Cost information ends 

        (at YOUBOT-BRSU START) 
        (gripper_is_free YOUBOT-BRSU) 

        (on ALLENKEY-00 WS01) 

        (on M20-00 WS02) 
        (on SCREWDRIVER-00 WS02) 

        (on M20_100-00 WS03) 
        (on WRENCH-00 WS03) 
        
        (on HOUSING-00 WS04)

        (on BEARING2-00 WS05) 
        (on F20_20_G-00 TT01) 

        (on S40_40_G-00 WS06) 
        
        (on MOTOR2-00 SH01) 
        (on S40_40_B-00 SH01) 

        (on CONTAINER_BOX_RED-00 WS06) 
        (on CONTAINER_BOX_BLUE-00 WS06) 
        (container CONTAINER_BOX_RED) 
        (heavy CONTAINER_BOX_RED) 
        (container CONTAINER_BOX_BLUE) 
        (heavy CONTAINER_BOX_BLUE) 
        (container PP01_CAVITY-00) 
        (heavy PP01_CAVITY-00) 
        (on PP01_CAVITY-00 PP01) 
        (is_large ALLENKEY-00) 
        (is_large DRILL-00) 
        (insertable F20_20_G-00)
        (insertable ALLENKEY-00)
        (insertable SCREWDRIVER-00)
        (insertable WRENCH-00)
        (insertable M20-00)
        (insertable M20_100-00)
        (is_big_enough PLATFORM_RIGHT) 
    ) 

    (:goal 
        (and 

            (on S40_40_B-00 WS01)



            (on BEARING2-00 SH01) 
            (on S40_40_G-00 SH01) 

            (in F20_20_G-00 PP01_CAVITY-00) 
            (in M20_100-00 PP01_CAVITY-00)

        ) 
    ) 

    (:metric minimize 
        (total-cost) 
    ) 

) 

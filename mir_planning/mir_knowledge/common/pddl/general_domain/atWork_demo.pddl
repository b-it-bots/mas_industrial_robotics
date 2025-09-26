;This PDDL problem definition was made automatically from a KB snapshot 
(define (problem general_domain_task) 
    (:domain general_domain) 

    (:objects 
        WS01 WS02 WS03 PP01 START EXIT - location 
        ALLENKEY BEARING2 HOUSING M20 M20_100 S40_40_G SCREWDRIVER WRENCH PP01_CAVITY-00 - object 
        YOUBOT-BRSU - robot 
        PLATFORM_LEFT PLATFORM_MIDDLE PLATFORM_RIGHT - robot_platform 
    ) 

    (:init 
        ;Cost information starts 
        (= (total-cost) 0) 
        ;Cost information ends 

        (at YOUBOT-BRSU PP01) 
        (gripper_is_free YOUBOT-BRSU) 

        
        (on M20_100 WS02)
        (container PP01_CAVITY-00) 
        (heavy PP01_CAVITY-00) 
        (on PP01_CAVITY-00 PP01) 
    ) 

    (:goal 
        (and 

            (in M20_100 PP01_CAVITY-00)

            
        ) 
    ) 

    (:metric minimize 
        (total-cost) 
    ) 

) 

(define (problem test_problem)
(:domain robocup_at_work)
(:objects

    ; locations
    WS01 WS02 - location

    ; objects
    M30 M20 - object
)
(:init

    (on M30 WS01)
    (on M20 WS01)
)
(:goal
    (and
        (on M30 WS02)
        (on M20 WS02)
        ;(at youbot-brsu END)
    )
)
)

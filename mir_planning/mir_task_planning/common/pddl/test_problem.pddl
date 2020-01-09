(define (problem test_problem)
(:domain robocup_at_work)
(:objects

    ; locations
    l--WS01 - location
    l--WS02 - location

    ; objects
    ;o--o1 o--o2 o--o3 o--o4 o--o4 - object
    o--M30 - object
    o--M20 - object
)
(:init

    (on o--M30 l--WS01)
    (on o--M20 l--WS01)
)
(:goal
    (and
        (on o--M30 l--WS02)
        (on o--M20 l--WS02)
        ;(at r--youbot-brsu l--END)
    )
)
)

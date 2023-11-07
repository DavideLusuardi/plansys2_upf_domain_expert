(define (domain simple)
(:requirements :strips :typing :adl :fluents :durative-actions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
robot
room
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

(:constants livingroom - room)

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

(robot_at ?r - robot ?ro - room)
(connected ?ro1 ?ro2 - room)
(battery_full ?r - robot)
(battery_low ?r - robot)
(charging_point_at ?ro - room)

);; end Predicates ;;;;;;;;;;;;;;;;;;;;
;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions
    (distance_travelled)
);; end Functions ;;;;;;;;;;;;;;;;;;;;

;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:action test_action
    :parameters (?r - robot ?ro - room)
    :precondition (and
        (robot_at ?r ?ro)
    )
    :effect (and
        (not (robot_at ?r ?ro))
    )
)

(:durative-action move
    :parameters (?r - robot ?r1 ?r2 - room)
    :duration ( = ?duration 5)
    :condition (and
        (at start(connected ?r1 ?r2))
        (at start(robot_at ?r ?r1))
        (over all(battery_full ?r))
        ; (at start(forall (?r - robot) (robot_at ?r ?r1)))
        ; (at start(imply (robot_at ?r ?r1) (battery_full ?r)))
        )
    :effect (and
        (at start(not(robot_at ?r ?r1)))
        (at end(robot_at ?r ?r2))
        (at end (increase (distance_travelled) 5))
    )
)

(:durative-action askcharge
    :parameters (?r - robot ?r1 ?r2 - room)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_at ?r ?r1))
        (at start(charging_point_at ?r2))
       )
    :effect (and
        (at start(not(robot_at ?r ?r1)))
        (at end(robot_at ?r ?r2))
    )
)

(:durative-action charge
    :parameters (?r - robot ?ro - room)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_at ?r ?ro))
        (at start(charging_point_at ?ro))
    )
    :effect (and
         (at end(not(battery_low ?r)))
         (at end(battery_full ?r))
    )
)

);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;

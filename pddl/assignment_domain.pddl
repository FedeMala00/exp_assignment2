(define (domain simple)
(:requirements :strips :typing :adl :fluents :durative-actions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
robot
waypoint
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

(robot_at ?r - robot ?ro - waypoint)
(patrolled ?wp - waypoint)

);; end Predicates ;;;;;;;;;;;;;;;;;;;;
;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions

);; end Functions ;;;;;;;;;;;;;;;;;;;;
;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:durative-action move
    :parameters (?r - robot ?r1 ?r2 - waypoint)
    :duration ( = ?duration 60)
    :condition (and
        (at start(robot_at ?r ?r1))
        )
    :effect (and
        (at start(not(robot_at ?r ?r1)))
        (at end(robot_at ?r ?r2))
    )
)
(:durative-action patrol
    :parameters (?r - robot ?wp - waypoint)
    :duration ( = ?duration 60)
    :condition (and
        (at start(robot_at ?r ?wp))
       )
    :effect (and
        (at end(patrolled ?wp))
    )
)

);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
(define (domain pick_place)

    (:requirements
        :typing
        :durative-actions
        )

    (:types
        waypoint
        robot
        object
        gripper
    )
    
    (:predicates
        (visited ?wp - waypoint)
        (robot-at ?v - robot ?wp - waypoint)
        (object-at ?obj - object ?wp - waypoint)
        (is_holding ?g - gripper ?obj - object)
        (free ?g - gripper)
        (priority-over ?obj1 ?obj2 - object)
        (highest-priority ?obj - object)
    )
    
    (:durative-action move
        :parameters (?v - robot ?from ?to - waypoint)
        :duration ( = ?duration 1)
        :condition (and
            (at start (robot-at ?v ?from))
        )
        :effect (and
            (at end (visited ?to))
            (at end (robot-at ?v ?to))
            (at start (not (robot-at ?v ?from)))
        )
    )

    (:durative-action pick
        :parameters (?v - robot ?wp - waypoint ?obj ?obj2 - object ?g - gripper)
        :duration (= ?duration 1)
        :condition (and
            (at start (robot-at ?v ?wp))
            (at end (robot-at ?v ?wp))
            (at start (object-at ?obj ?wp))
            (at start (free ?g))
            (at start (priority-over ?obj ?obj2))
            (at start (highest-priority ?obj))

        )
        :effect (and
            (at end (is_holding ?g ?obj))
            (at end (not (object-at ?obj ?wp)))
            (at end (not (free ?g)))
            (at start (not (free ?g)))
            (at end (not (highest-priority ?obj)))
            (at end (highest-priority ?obj2))
        )
    )

    (:durative-action place
        :parameters (?v - robot ?wp - waypoint ?obj - object ?g - gripper)
        :duration (= ?duration 1)
        :condition (and
            (at start (robot-at ?v ?wp))
            (at end (robot-at ?v ?wp))
            (at start (is_holding ?g ?obj))
        )
        :effect (and
            (at end (not (is_holding ?g ?obj)))
            (at end (object-at ?obj ?wp))
            (at end (not (object-at ?obj ?wp)))
            (at end (free ?g))
        )
    )

)

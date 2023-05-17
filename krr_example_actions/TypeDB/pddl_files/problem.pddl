(define (problem pick_place)
    (:domain pick_place)
    (:requirements :strips :typing)

    (:objects   albert - robot
                rightgrip - gripper
                
                chocolate1 - object
                
                no_priority - object
                
                wp_discount_bin - waypoint
                
                wp_table0 - waypoint
                
                wp0 - waypoint
    )
    
    (:init
        (robot-at albert wp0)
        
        (object-at chocolate1 wp_table0)
        
        
        
        (highest-priority chocolate1)
        
        
        (priority-over chocolate1 no_priority)
        
        
        (free rightgrip)
    )
    
    (:goal (and
        
        (object-at chocolate1 wp_discount_bin)
        
        )
    )

)

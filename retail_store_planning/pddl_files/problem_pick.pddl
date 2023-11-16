(define (problem waypoint_following)
    (:domain waypoint_following)
    (:requirements :strips :typing)

    (:objects   tiago - robot
                leftgrip rightgrip - gripper
		wp0 wp_table_1 - waypoint
                april_tag_cube_23 - object
    )
    (:init
	    (visited wp0)
	    (robot-at tiago wp0)
	    (object-at april_tag_cube_23 wp_table_1)
	    (free rightgrip)
    )
    
    (:goal (and
        (visited wp_table_1)
        (is_holding rightgrip april_tag_cube_23)
    ))

)


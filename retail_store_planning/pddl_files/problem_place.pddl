(define (problem waypoint_following)
    (:domain waypoint_following)
    (:requirements :strips :typing)

    (:objects   tiago - robot
                rightgrip - gripper
		wp0 wp_table_1 wp_table_2 wp_table_3 wp_cabinet_1 wp_cabinet_2 - waypoint
                april_tag_cube_23 april_tag_cube_8 - object
    )
    (:init
	    (visited wp0)
	    (robot-at tiago wp0)
	    (object-at april_tag_cube_23 wp_table_1)
	    (free rightgrip)
	    (object-to april_tag_cube_23 wp_cabinet_1)
    )

    (:goal (and

        (object-at april_tag_cube_23 wp_cabinet_1)
    ))

)


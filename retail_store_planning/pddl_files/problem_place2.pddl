(define (problem waypoint_picking) ; Defines a new planning problem with the name "waypoint_picking".
    (:domain waypoint_picking) ; Specifies that the domain of the problem is "waypoint_picking".
    (:requirements :strips :typing) ; Specifies the STRIPS and typing features are used.

    ;Defining objects for the problem
    (:objects   tiago - robot ; A robot named tiago.
                rightgrip - gripper ; A gripper named rightgrip.
                wp0 wp_table_1 wp_table_2 wp_table_3 wp_table_4 - waypoint ; Four waypoints. 
                ; april_tag_cube_23 april_tag_cube_8 object_3 object_4 - object
                ; april_tag_cube_23 april_tag_cube_8 - object ; For testing.

    )
    ; The initial state of the world before task planning. 
    (:init
        ; Initial robot state.
        (visited wp0) ; Indicates that the robot has visited waypoint "wp0".
        (robot-at tiago wp0) ; Indicates that the robot is at waypoint "wp0".
        (can-move tiago) ; Indicates that the robot "tiago" can move.
        (free rightgrip) ; Indicates that the gripper "rightgrip" is free.
        (not (not-free rightgrip)) ; Negated version of the above state.

        ; Initial product info april_tag_cube_8. 
        ; (object-at april_tag_cube_8 wp_table_1) ; Indicates that the object "april_tag_cube_8" is at the stock table. 
        ; (not-object-at april_tag_cube_8 wp_table_4) ; Indicates that the object "april_tag_cube_8" is not at the shelf table. 
        ; (is-full april_tag_cube_8) ; Indicates that the object "april_tag_cube_8" is not used and thus full. 
        ; (is-not-refrigerated april_tag_cube_8) ; Indicates that the object "april_tag_cube_8" does not need refrigeration.

        ; Initial product info april_tag_cube_23.
        ; (object-at april_tag_cube_23 wp_table_1) ; Indicates that the object "april_tag_cube_23" is at the stock table. 
        ; (not-object-at april_tag_cube_23 wp_table_3) ; Indicates that the object "april_tag_cube_8" is not at the shelf table.
        ; (is-not-full april_tag_cube_23) ; Indicates that the object "april_tag_cube_23" is used and thus full. 
        ; (is-not-refrigerated april_tag_cube_23) ; Indicates that the object "april_tag_cube_23" does not need refrigeration.


        ; Initial product info object_3.
        ; (object-at object_3 wp_table_1) ; Indicates that the object "object_3" is at the stock table. 
        ; (not-object-at object_3 wp_cabinet_2) ; Indicates that the object "object_3" is not at the refrigerated table.
        ; (is-full object_3) ; Indicates that the object "object_3" is not used and thus full. 
        ; (is-refrigerated object_3) ; Indicates that object_3 needs to be refrigerated.

        ; Initial enviroment state. 
        (stock wp_table_1) ; Indicates that the waypoint "wp_table_1" is the robot base location of the stock table.
        (refrigerator wp_table_2) ; Indicates that the waypoint "wp_table_2" is the robot base location for the refrigerated table.
        (bin wp_table_3) ; Indicates that the waypoint "wp_table_3" is the robot base location for the bin table.
        (shelf wp_table_4) ; Indicates that the waypoint "wp_table_4" is the robot base location for the shelf table.
        
        


    
    )
    
    (:goal (and ; Specifies the goal state, which is a conjunction of the following states:
        ; (stored april_tag_cube_8) ; Indicates that the object "april_tag_cube_8" is stored. 
        ; (stored april_tag_cube_23) ; Indicates that the object "april_tag_cube_23" is stored. 
        ; (stored object_3) ; Indicates that the object "object 3" is stored. 



    )

    )

)   


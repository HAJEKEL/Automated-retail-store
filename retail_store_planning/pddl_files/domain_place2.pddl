(define (domain waypoint_picking) ; Defines a new planning domain with the name "waypoint_picking".
    (:requirements ; Specifies that the domain requires several PDDL features.
        :typing
        :durative-actions
        :duration-inequalities
        :strips
        :negative-preconditions
        :continuous-effects
        :numeric-fluents
        )

    ; Defines the different types of objects that can be used in the planning problem.
    (:types
        waypoint
        robot
        object
        gripper
    )
    ; Defines predicates that can be used to describe the state of the world. 
    (:predicates
        ; Set of predicates that define conditions and properties of the world state concerning the robot. 
        (visited ?wp - waypoint) ; Predicate of arity 1 that takes a waypoint as input argument: waypoint is visited.
        (robot-at ?v - robot ?wp - waypoint) ; Predicate of arity 2 that takes a robot and a waypoint as input arguments: robot is at waypoint.
        (is_holding ?g - gripper ?obj - object) ; Predicate of arity 2 that takes a gripper and an object as input arguments: gripper is holding object.
        (free ?g - gripper) ; Predicate of arity 1 that takes a gripper as input argument: gripper is free.
        (can-move ?v - robot) ; Predicate of arity 1 that takes a robot as input argument: robot can move.
        
        ; Set of predicates that define conditions and properties of the world state concerning the products. 
        (object-at ?obj - object ?wp - waypoint) ; Predicate of arity 2 that takes an object and waypoint as input arguments: object is at a waypoint.
        (stored ?obj - object) ; Predicate of arity 1 that takes an object as input argument: object is stored.
        (is-full ?obj - object) ; Predicate of arity 1 that takes an object as input argument: object is not used and thus full.
        (is-not-full ?obj - object) ; Predicate of arity 1 that takes an object as input argument: object used and thus is not full.
        (is-refrigerated ?obj - object) ; Predicate of arity 1 that takes an object as input argument: object needs to be refrigerated.
        (is-not-refrigerated ?obj - object) ; Predicate of arity 1 that takes an object as input argument: object does not need refrigeration. 
        
        ; Set of predicates that define conditions and properties of the world state concerning the environment. 
        (bin ?wp - waypoint) ; Predicate of arity 1 that takes a waypoint as input argument: waypoint is robot base location for the bin table.
        (shelf ?wp - waypoint) ; Predicate of arity 1 that takes a waypoint as input argument: waypoint is robot base location for the shelf table.
        (refrigerator ?wp - waypoint) ; Predicate of arity 1 that takes a waypoint as input argument: waypoint is robot base location for the refrigerated table.
        (stock ?wp - waypoint) ; Predicate of arity 1 that takes a waypoint as input argument: waypoint is robot base location for the stock table.

        ; Negated versions of predicates
        (not-free ?g - gripper); Predicate of arity 1 that takes a gripper as input argument: gripper is not free.
        (not-object-at ?obj - object ?wp - waypoint) ; Predicate of arity 2 that takes an object and a waypoint as input arguments: the object is not at the waypoint.
    )


    ; Functions used.
    ; (:functions
    ;     (has-mass ?m - object)  
    ;
    ; )

    (:durative-action move ; Defines a non-instantaneous action called "move" that represents the robot moving from one robot base waypoint to another.
        :parameters (?v - robot ?from - waypoint ?to - waypoint) ; Specifies the parameters of the action, which are a robot object, a starting waypoint, and a destination waypoint.
        :duration (= ?duration 60) ; Specifies the duration of the action as a fixed value of 60 time units.
        :condition (and ; Specifies the preconditions for the action, which are a conjunction of the following conditions:

            (at start (robot-at ?v ?from)) ; The robot must be at the starting waypoint at the start of the action.
            (at start (can-move ?v)) ; The robot must be capable of moving at the start of the action.

        )

        :effect (and ; Specifies the effects of the move action, which are a conjunction of the following effects:

            (at end (visited ?to)) ; A delayed effect of the action is that the destination waypoint is marked as visited at the end of the action.
            (at end (robot-at ?v ?to)) ; A delayed effect of the action is that the robot is at the destination waypoint at the end of the action.
            (at start(not (robot-at ?v ?from))) ; An instantaneous effect of the action is that the robot is not at the starting waypoint.
            (at end (not (can-move ?v))) ; A delayed effect of the action is that the robot cannot move at the end of the action.

        )
    )

    (:durative-action pick ; Defines an action called "pick" that represents the robot picking up an object from a robot base waypoint.
        :parameters (?v - robot ?wp - waypoint ?obj - object ?g - gripper) ; Specifies the parameters of the action, which are a robot object, a waypoint where the object is located, the object to be picked up, and a gripper object.
        :duration (= ?duration 60) ; Specifies the duration of the action as a fixed value of 60 time units.
        :condition (and ; Specifies the preconditions for the action, which are a conjunction of the following conditions:

            (at start (stock ?wp)) ; The waypoint must be the stock waypoint at the start of the action.
            (at start (robot-at ?v ?wp)) ; The robot base must be at the stock waypoint at the start of the action.
            (at start (object-at ?obj ?wp)) ; The object must be at the stock waypoint at the start of the action.
            (at start (free ?g)) ; The gripper must be free at the start of the action.

        )

        :effect (and ; Specifies the effects of the pick action, which are a conjunction of the following effects:

            (at end (is_holding ?g ?obj)) ; A delayed effect of the action is that the gripper is holding an object at the end of the action.
            (at start (not (free ?g))) ; An instantaneous effect of the action is that the gripper is free. 
            (at start (not (object-at ?obj ?wp))) ; An instantaneous effect of the action is that the object is not at the waypoint anymore.
            (at end (not-free ?g)) ; A delayed effect of the action is that the gripper is not free anymore at the end of the action. 
            (at end (can-move ?v)) ; A delayed effect of the action is that the robot is allowed to perform a move action at the end of the action. 

        )

    )

    (:durative-action place ; Action to place an object on the shelf table for non-refrigerated non-used products while at the shelf waypoint.
        :parameters (?v - robot ?wp - waypoint  ?obj - object ?g - gripper) ; The parameters for the non-instantaneous action place, including the robot, the shelf table waypoint, the object being placed, and the gripper that is holding the object.
        :duration (= ?duration 60) ; The duration of the durative action set to 60 time units.
        :condition (and ; Specifies the preconditions for the place on shelf table action, which are a conjunction of the following conditions:
            
            (at start (is-full ?obj)) ; The object held by the gripper must be not used and therefore full at the start of the action.
            ; (at start (= (has-mass ?obj) 0.5)) 
            (at start (is-not-refrigerated ?obj)) ; The object held by the gripper must be one that does not need refrigeration at the start of the action.
            (at start (is_holding ?g ?obj)) ; The gripper must be holding the object at the start of the action.
            (at start (not-free ?g)) ; The gripper must not be free at the start of the action.
            (at start (shelf ?wp)) ; The robot base waypoint must be the shelf table at the start of the action.
            (at start (robot-at ?v ?wp)) ; The robot must be at the shelf table waypoint at the start of the action.

        ) 

        :effect (and ; Specifies the effects of the place on shelf table action, which are a conjunction of the following effects:

            ; (at start (not (not-object-at ?obj ?wp))) 
            (at end (object-at ?obj ?wp)) ; Delayed effect of the action is that the object is at the shelf table.
            (at start (free ?g)) ; Instantaneous effect of the action is that the gripper is free.
            (at start (not (not-free ?g))) ; An instantaneous effect of th action is the negated version of the above effect. 
            (at end (not (is_holding ?g ?obj))) ; A delayed effect of the action is that the gripper is not holding the object anymore.
            (at end (can-move ?v)) ; A delayed effect of the action is that the robot is allowed to do a move action now.
            (at end (stored ?obj)) ; A delayed effect of the action is that the object is stored at the end of the action.
        
        )

    )

    (:durative-action place1 ; Action to place not full objects on the bin table while at the bin table waypoint.
        :parameters (?v - robot ?wp - waypoint  ?obj - object ?g - gripper) ; The parameters for the non-instantaneous bin table place action, including the robot, the bin table waypoint, the object being placed, and the gripper that is holding the object.
        :duration (= ?duration 60) ; The duration of the durative action set to 60 time units.
        :condition (and ; Specifies the preconditions for the place on bin table action, which are a conjunction of the following conditions:

            (at start (is-not-full ?obj)) ; The object needs to be used and therefore not full at the start of the action.
            ; (at start (< (has-mass ?object) 0.5))
            (at start (is_holding ?g ?obj)) ; The gripper must be holding the used product at the start of the action.
            (at start (not-free ?g)) ; The gripper needs to be not free at the start of the action.
            (at start  (bin ?wp)) ; The waypoint must be the bin table waypoint at the start of the action.
            (at start (robot-at ?v ?wp)) ; The current location of the robot base must be the bin table waypoint at the start of the action.

        ) 

        :effect (and ; Specifies the effects of the place on bin table action, which are a conjunction of the following effects:

            ; (at start (not (not-object-at ?obj ?wp)))
            (at end (object-at ?obj ?wp)) ; A delayed effect of the action is that the object ends up at the bin table at the end of the action.
            (at start (free ?g)) ; An instantaneous effect of the action is that the gripper is free.
            (at start (not (not-free ?g))) ; An instantaneous effect of the action is the negated version of the above effect.
            (at end (not (is_holding ?g ?obj))) ; A delayed effect of the action is that the gripper is not holding the object anymore at the end of the action.
            (at end (can-move ?v)) ; A delayed effect of the action is that the robot is allowed to do a move action at the end of the action.
            (at end (stored ?obj)) ; A delayed effect of the action is that the object is stored at the end of the action. 
        
        )

    )

    (:durative-action place2 ; Action to place refrigerated objects on the refrigerated table while at the refrigerated table waypoint.
        :parameters (?v - robot ?wp - waypoint  ?obj - object ?g - gripper) ; The parameters for the non-instantaneous refrigerated table place action, including the robot, the refrigerated table waypoint, the object being placed, and the gripper that is holding the object.
        :duration (= ?duration 60) ; The duration of the durative action set to 60 time units.
        :condition (and ; Specifies the preconditions for the place on refrigerated table action, which are a conjunction of the following conditions:

            (at start (is-full ?obj)) ; The object needs to be noNegativet used and therefore full at the start of the action.
            ; (at start (< (has-mass ?object) 0.5))
            (at start (is-refrigerated ?obj)) ; The object needs to be an object that needs refrigeration at the start of the action.
            (at start (is_holding ?g ?obj)) ; The gripper needs to be holding the object at the start of the action.
            (at start (not-free ?g)) ; The gripper needs to be not free at the start of the action.
            (at start (refrigerator ?wp)) ; The waypoint needs to be the refrigerated table waypoint at the start of the action.
            (at start (robot-at ?v ?wp)) ; The current location of the robot base must be the refrigerated table waypoint at the start of the action.

        ) 

        :effect (and ; Specifies the effects of the place on refrigerated table action, which are a conjunction of the following effects:

            ; (at start (not (not-object-at ?obj ?wp)))
            (at end (object-at ?obj ?wp)) ; A delayed effect of the action is that the object ends up at the refrigerated table at the end of the action.
            (at start (free ?g)) ; An instantaneous effect of the action is that the gripper is free.
            (at start (not (not-free ?g))) ; An instantaneous effect of the action is the negated version of the above effect.
            (at end (not (is_holding ?g ?obj))) ; A delayed effect of the action is that the gripper is not holding the object anymore at the end of the action.
            (at end (can-move ?v)) ; A delayed effect of the action is that the robot is allowed to do a move action at the end of the action. 
            (at end (stored ?obj)) ; A delayed effect of the action is that the object is stored at the end of the action. 
        
        )

    )
)

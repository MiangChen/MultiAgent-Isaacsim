(define (domain delivery_task-domain)
 (:requirements :multi-agent :factored-privacy :typing)
 (:types item location ag - object
    robot1_type robot2_type robot3_type - ag
 )
 (:predicates
  (item_at ?i - item ?l - location)
  (a_robot_at ?agent - ag ?l - location)
  (:private
   (a_holding ?agent - ag ?i - item)))
 (:action put_down
  :parameters ( ?robot3 - robot3_type ?it - item ?loc - location)
  :precondition (and 
   (a_robot_at  ?robot3 ?loc)
   (a_holding  ?robot3 ?it)
  )
  :effect (and
 (not (a_holding  ?robot3 ?it)) (item_at ?it ?loc)))
 (:action navigate_to
  :parameters ( ?robot3 - robot3_type ?start_ - location ?goal_ - location)
  :precondition (and 
   (a_robot_at  ?robot3 ?start_)
  )
  :effect (and
 (not (a_robot_at  ?robot3 ?start_)) (a_robot_at  ?robot3 ?goal_)))
 (:action pick_up
  :parameters ( ?robot3 - robot3_type ?it - item ?loc - location)
  :precondition (and 
   (a_robot_at  ?robot3 ?loc)
   (item_at ?it ?loc)
  )
  :effect (and
 (a_holding  ?robot3 ?it)))
)

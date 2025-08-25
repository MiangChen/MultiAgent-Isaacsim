(define (problem delivery_task-problem)
 (:domain delivery_task-domain)
 (:objects
   item1 item2 - item
   place1 place2 depot - location
   robot1 - robot1_type
   robot2 - robot2_type
   robot3 - robot3_type
 )
 (:init
  (item_at item1 place1)
  (item_at item2 place2)
  (a_robot_at robot1 depot)
  (a_robot_at robot2 depot)
  (a_robot_at robot3 depot))
 (:goal (and (item_at item1 depot) (item_at item2 depot)))
)
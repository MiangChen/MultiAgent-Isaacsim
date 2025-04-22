(define (problem delivery-task)
  (:domain multi-robot-delivery)

  (:objects
    ;; 机器人
    robot1 robot2 robot3 robot4 robot5 - robot
    ;; 地点
    cityA cityB cityC depot         - location
    ;; 物品
    obj1 obj2 obj3                 - item
  )

  (:init
    ;; 机器人初始都在 depot
    (at robot1 depot)  (at robot2 depot)  (at robot3 depot)
    (at robot4 depot)  (at robot5 depot)
    ;; 物品初始分布
    (item-at obj1 cityA)
    (item-at obj2 cityB)
    (item-at obj3 cityC)
  )

  (:goal
    (and
      ;; 所有物品都回到 depot
      (item-at obj1 depot)
      (item-at obj2 depot)
      (item-at obj3 depot)
    )
  )
)

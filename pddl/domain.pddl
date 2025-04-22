(define (domain multi-robot-delivery)
  (:requirements :typing :strips)

  ;; 类型声明：不要使用 `object` 作为自定义类型
  (:types
    robot
    location
    item
  )

  ;; 谓词
  (:predicates
    (at ?r - robot ?l - location)
    (item-at ?i - item ?l - location)
    (holding ?r - robot ?i - item)
  )

  ;; navigate-to
  (:action navigate-to
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (at ?r ?from)
    :effect (and
      (not (at ?r ?from))
      (at ?r ?to)
    )
  )

  ;; pick-up
  (:action pick-up
    :parameters (?r - robot ?i - item ?l - location)
    :precondition (and
      (at ?r ?l)
      (item-at ?i ?l)
    )
    :effect (and
      (not (item-at ?i ?l))
      (holding ?r ?i)
    )
  )

  ;; put-down
  (:action put-down
    :parameters (?r - robot ?i - item ?l - location)
    :precondition (and
      (at ?r ?l)
      (holding ?r ?i)
    )
    :effect (and
      (not (holding ?r ?i))
      (item-at ?i ?l)
    )
  )
)

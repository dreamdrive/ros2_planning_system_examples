(define (domain simple)
(:requirements :strips :typing :adl :fluents :durative-actions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
robot
pose
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

(robot_at ?r - robot ?po - pose)
(connected ?po1 ?po2 - pose)
(zero_point_at ?po - pose)
(joint_at ?po - pose)
; (battery_full ?r - robot)
; (battery_low ?r - robot)
; (charging_point_at ?ro - room)

);; end Predicates ;;;;;;;;;;;;;;;;;;;;
;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions

);; end Functions ;;;;;;;;;;;;;;;;;;;;
;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:durative-action move
    :parameters (?r - robot ?po1 ?po2 - pose)
    :duration ( = ?duration 5)
    :condition (and
        (at start(connected ?po1 ?po2))
        (at start(robot_at ?r ?po1))
        ; (over all(battery_full ?r))
        )
    :effect (and
        (at start(not(robot_at ?r ?po1)))
        (at end(robot_at ?r ?po2))
    )
)

(:durative-action zero_move
    :parameters (?r - robot ?po1 ?po2 - pose)
    :duration ( = ?duration 5)
    :condition (and
        ; (at start(connected ?po1 ?po2))
        (at start(robot_at ?r ?po1))
        (at start(zero_point_at ?po2))
        ; (over all(battery_full ?r))
        )
    :effect (and
        (at start(not(robot_at ?r ?po1)))
        (at end(robot_at ?r ?po2))
    )
)

(:durative-action joint_move
    :parameters (?r - robot ?po1 ?po2 - pose)
    :duration ( = ?duration 5)
    :condition (and
        ; (at start(connected ?po1 ?po2))
        (at start(robot_at ?r ?po1))
        (at start(joint_at ?po2))
        ; (over all(battery_full ?r))
        )
    :effect (and
        (at start(not(robot_at ?r ?po1)))
        (at end(robot_at ?r ?po2))
    )
)

; (:durative-action askcharge
;     :parameters (?r - robot ?r1 ?r2 - room)
;     :duration ( = ?duration 5)
;     :condition (and
;         (at start(robot_at ?r ?r1))
;         (at start(charging_point_at ?r2))
;        )
;     :effect (and
;         (at start(not(robot_at ?r ?r1)))
;         (at end(robot_at ?r ?r2))
;     )
; )

; (:durative-action charge
;     :parameters (?r - robot ?ro - room)
;     :duration ( = ?duration 5)
;     :condition (and
;         (at start(robot_at ?r ?ro))
;         (at start(charging_point_at ?ro))
;     )
;     :effect (and
;          (at end(not(battery_low ?r)))
;          (at end(battery_full ?r))
;     )
; )

);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;

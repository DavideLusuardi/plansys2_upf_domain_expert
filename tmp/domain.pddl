;; Enrico Scala (enricos83@gmail.com) and Miquel Ramirez (miquel.ramirez@gmail.com)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain fn-counters)
    (:types
        counter - object
        counter_sub - counter
        counter_sub_sub - counter_sub
    )

    (:constants test_constant - counter)

    (:predicates
        (test_predicate ?s - counter)
    )
    (:functions
        (value ?c - counter);; - int  ;; The value shown in counter ?c
        (max_int);; -  int ;; The maximum integer we consider - a static value
    )


    ;; Increment the value in the given counter by one
    (:action test_action1
        :parameters (?c - counter)
        :precondition (and (<= (+ (value ?c) 1) (max_int)) (test_predicate ?c))
        :effect (and (increase (value ?c) 1))
    )

    ;; Decrement the value in the given counter by one
    (:action test_action2
        :parameters (?c - counter)
        :precondition (and (>= (value ?c) 1))
        :effect (and (decrease (value ?c) 1))
    )

    (:durative-action test_durative_action
        :parameters (?match - counter)
        :duration (= ?duration 5)
        :condition (and (at start (test_predicate ?match)) (over all (test_predicate ?match)) (at end (test_predicate ?match)))
        :effect (and (at start (test_predicate ?match)) (at end (not (test_predicate ?match))))
    )

)

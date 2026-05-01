(def CAN1_TX 3)
(def CAN1_RX 4)
(def CAN2_TX 7)
(def CAN2_RX 8)
(def CAN_BAUD_K 500)
(def TEST_CAN_ID 291)

(can-stop)
(can2-stop)

(can-start CAN1_TX CAN1_RX)
(can2-start CAN2_TX CAN2_RX CAN_BAUD_K)

(can-use-vesc nil)
(can2-use-vesc nil)

(def tx_cnt 0)

(defun can1_sender ()
    (loopwhile t
        (progn
            (let ((payload (list tx_cnt 170 85 1 2 3 4 5)))
                (can-send-sid TEST_CAN_ID payload)
                (print (list 'can1-tx 'id TEST_CAN_ID 'data payload)))
            (setq tx_cnt (mod (+ tx_cnt 1) 256))
            (sleep 0.2))))

(defun can2_receiver ()
    (loopwhile t
        (let ((pkt (can2-recv-sid 1.0)))
            (if pkt
                (print (list 'can2-rx pkt))
                (print 'can2-timeout)))))

(def can1_sender_cid (spawn can1_sender))
(def can2_receiver_cid (spawn can2_receiver))

(print (list 'started 'sender can1_sender_cid 'receiver can2_receiver_cid 'id TEST_CAN_ID))

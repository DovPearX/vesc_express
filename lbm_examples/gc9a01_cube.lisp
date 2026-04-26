; RST 6
; CS 5
; DC 4
; SDA 3
; SCL 2

(disp-load-gc9a01 3 2 5 6 4 40)
(disp-reset)
(ext-disp-orientation 0)

(def img (img-buffer 'indexed4 240 240))

(defun line (x0 y0 x1 y1)
    (img-line img x0 y0 x1 y1 1 '(thickness 2))
)

(def nodes '((-1 -1 -1) (-1 -1 1) (-1 1 -1) (-1 1 1) (1 -1 -1) (1 -1 1) (1 1 -1) (1 1 1)))
(def edges '((0  1) (1 3) (3 2) (2 0) (4 5) (5 7) (7 6) (6 4) (0 4) (1 5) (2 6) (3 7)))

(defun draw-edges () {
        (var scale 45.0)
        (var ofs-x (/ 120 scale))
        (var ofs-y (/ 140 scale))

        (loopforeach e edges {
                (var na (ix nodes (ix e 0)))
                (var nb (ix nodes (ix e 1)))

                (apply line (map (fn (x) (to-i (* x scale))) (list
                            (+ ofs-x (ix na 0)) (+ ofs-y (ix na 1))
                            (+ ofs-x (ix nb 0)) (+ ofs-y (ix nb 1))
                )))
        })
})

(defun rotate-cube (ax ay) {
        (var sx (sin ax))
        (var cx (cos ax))
        (var sy (sin ay))
        (var cy (cos ay))

        (loopforeach n nodes {
                (var x (ix n 0))
                (var y (ix n 1))
                (var z (ix n 2))

                (setix n 0 (- (* x cx) (* z sx)))
                (setix n 2 (+ (* z cx) (* x sx)))
                (setq z (ix n 2))
                (setix n 1 (- (* y cy) (* z sy)))
                (setix n 2 (+ (* z cy) (* y sy)))
        })
})

(def fps 0)

(loopwhile t {
        (var t-start (systime))
        (img-text img 60 35 2 0 (str-from-n fps "FPS %.1f ") '(magnify 3))
        (img-circle img 120 120 120 3 '(thickness 2))
        (draw-edges)
        (rotate-cube 0.1 0.05)
        (disp-render img 0 0 (list 0x000000 0xff0000 0x00ff00 0x0000ff))
        (img-clear img)
        (def fps (/ 1 (secs-since t-start)))
})
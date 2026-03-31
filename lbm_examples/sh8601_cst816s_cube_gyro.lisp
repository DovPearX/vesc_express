(import "font_16_26.bin" 'font)

(disp-load-sh8601 13 10 12 9 11 40)
(disp-reset)
(ext-disp-orientation 1)

(imu-start 6 47 48)

(def img (img-buffer 'rgb565 320 170))

(def bg-color 0x000000)
(def line-color 0xff0000)
(def text-color 0xffffff)

(def nodes '((-1 -1 -1) (-1 -1 1) (-1 1 -1) (-1 1 1)
             (1 -1 -1) (1 -1 1) (1 1 -1) (1 1 1)))
(def edges '((0  1) (1 3) (3 2) (2 0) (4 5) (5 7) (7 6) (6 4) (0 4) (1 5) (2 6) (3 7)))

(def fps 0)


(defun line (x0 y0 x1 y1)
    (img-line img x0 y0 x1 y1 line-color '(thickness 2))
)

(defun draw-edges () {
        (var scale 45.0)
        (var ofs-x (/ 220 scale))
        (var ofs-y (/ 85 scale))

        (loopforeach e edges {
                (var na (ix nodes (ix e 0)))
                (var nb (ix nodes (ix e 1)))

                (var p (map (fn (x) (to-i (* x scale))) (list
                            (+ ofs-x (ix na 0)) (+ ofs-y (ix na 1))
                            (+ ofs-x (ix nb 0)) (+ ofs-y (ix nb 1))
                )))
                (line (ix p 0) (ix p 1) (ix p 2) (ix p 3))
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

(loopwhile t {
    (var t-start (systime))

    (var g (get-imu-gyro))
    (var gx (ix g 0))
    (var gy (ix g 1))
    (var gz (ix g 2))
    (var dt 0.02)

    (rotate-cube (* gx 0.0008) (* gy 0.0008))

    (img-clear img bg-color)
    (img-rectangle img 0 0 319 169 line-color)
    (draw-edges)

    (img-text img 6 10 text-color 0 font (str-from-n fps "FPS %.1f"))
    (img-text img 6 30 text-color 0 font (str-from-n gx "GX %.1f"))
    (img-text img 6 50 text-color 0 font (str-from-n gy "GY %.1f"))
    (img-text img 6 70 text-color 0 font (str-from-n gz "GZ %.1f"))

    (disp-render img 0 0)

    (setq fps (/ 1 (secs-since t-start)))
    (sleep 0.01)
})

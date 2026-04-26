(def pin-clk 1)
(def pin-sd0 2)
(def pin-rst 22)
(def pin-dc 15)
(def pin-cs 14)
(def pin-bl 23)
(def spi-mhz 40)

(import "font_16_26.bin" 'font)

(gpio-configure pin-bl 'pin-mode-out)
(gpio-write pin-bl 0)

(disp-load-jd9853 pin-sd0 pin-clk pin-cs pin-rst pin-dc spi-mhz)
(disp-reset)
(ext-disp-orientation 3)

(gpio-write pin-bl 1)

(def img (img-buffer 'indexed4 320 172))

(defun line (x0 y0 x1 y1)
    (img-line img x0 y0 x1 y1 1 '(thickness 2))
)

(def nodes '((-1 -1 -1) (-1 -1 1) (-1 1 -1) (-1 1 1) (1 -1 -1) (1 -1 1) (1 1 -1) (1 1 1)))
(def edges '((0  1) (1 3) (3 2) (2 0) (4 5) (5 7) (7 6) (6 4) (0 4) (1 5) (2 6) (3 7)))

(defun draw-edges () {
        (var scale 35.0)
        (var ofs-x (/ 220 scale))
        (var ofs-y (/ 85 scale))

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
        (img-text img 10 10 3 0 font (str-from-n fps "FPS %.1f "))
        (img-rectangle img 0 0 319 169 2 '(rounded 20) '(thickness 2))
        (draw-edges)
        (rotate-cube 0.1 0.05)
        (disp-render img 0 0 (list 0x000000 0xff0000 0x00ff00 0x0000ff))
        (img-clear img)
        (def fps (/ 1 (secs-since t-start)))
})

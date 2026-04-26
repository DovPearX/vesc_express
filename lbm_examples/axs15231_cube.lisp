(import "font_16_26.bin" 'font)

(gpio-configure 1 'pin-mode-out)
(gpio-write 1 1)

(disp-load-axs15231 21 48 40 39 47 45 -1 80)
(disp-reset)
(ext-disp-orientation 0)

(def img (img-buffer 'rgb565 320 480))

(defun line (x0 y0 x1 y1)
    (img-line img x0 y0 x1 y1 0xffffff '(thickness 2))
)

(def nodes '((-1 -1 -1) (-1 -1 1) (-1 1 -1) (-1 1 1) (1 -1 -1) (1 -1 1) (1 1 -1) (1 1 1)))
(def edges '((0  1) (1 3) (3 2) (2 0) (4 5) (5 7) (7 6) (6 4) (0 4) (1 5) (2 6) (3 7)))

(defun draw-edges () {
        (var scale 68.0)
        (var ofs-x (/ 160 scale))
        (var ofs-y (/ 240 scale))

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
        (img-text img 5 5 0xffffff 0x000000 font (str-from-n fps "FPS %.1f"))
        (img-circle img 160 240 150 0xff0000 '(thickness 5))
        (img-rectangle img 0 0 319 479 0x00ff00)
        (draw-edges)
        (rotate-cube 0.1 0.05)
        (disp-render img 0 0)
        (img-clear img 0)
        (def fps (/ 1 (secs-since t-start)))
})
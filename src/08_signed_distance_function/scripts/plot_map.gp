#!/usr/bin/gnuplot --persist

set terminal wxt
unset surface
set table "../data/contour.txt"
set contour
set cntrparam bspline
set cntrparam levels discrete 0
splot "../data/result.txt" matrix every 5:5 w l
unset table
set size ratio -1
set yrange [400:800]
set xrange [250:600]
set key outside bottom center
plot "../data/contour.txt" w l lt -1 lw 1.5 t 'walls', "../data/data.txt" u 1:2 every ::2 w l lt 1 lw 2 t 'robot trajectory'

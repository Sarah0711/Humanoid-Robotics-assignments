#!/usr/bin/gnuplot --persist

set terminal wxt

set xrange [-8:8]
set yrange [-8:8]
set palette defined ( -1 "white", 0.0 "red", 0.4 "red", 0.7 "yellow", 1 "green" )
unset colorbox
set size ratio -1
set grid xtics ytics front
set arrow 1 from -8,0 to 8,0 nohead lt 1 lc rgb "black" front
set arrow 2 from 0,-8 to 0,8 nohead lt 1 lc rgb "black" front
plot "../data/irm.txt" u ($1*0.1-6.0):($2*0.1-6.0):3 matrix with image title ""

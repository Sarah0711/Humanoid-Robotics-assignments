#!/usr/bin/gnuplot -persist
set terminal wxt
set size ratio -1
set key outside below
plot "../data/calib_pose.txt" u 1:2 w l lt 1 title "ground truth", \
     "" u 1:2:(2.0*cos($3)):(2.0*sin($3)) every 15 title "" w vectors lt 1 lw 3 nohead, \
     "" u 1:2 every 15 w p pt 19 lt 1 ps 2 title "", \
     "" u 4:5 title "uncalibrated" w l lt 2, \
     "" u 4:5:(2.0*cos($6)):(2.0*sin($6)) every 15 title "" w vectors lt 2 lw 3 nohead, \
     "" u 4:5 every 15 w p pt 19 lt 2 ps 2 title "", \
     "../data/trajectory.txt" u 1:2 title "calibrated" w l lt 3, \
     "" u 1:2:(2.0*cos($3)):(2.0*sin($3)) every 15 title "" w vectors lt 3 lw 3 nohead, \
     "" u 1:2 every 15 w p pt 19 lt 3 ps 2 title ""


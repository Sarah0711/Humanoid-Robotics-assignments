#!/usr/bin/gnuplot --persist

reset
set terminal wxt size 800,350

unset colorbox
set palette defined (0 1 1 1, 1 0 0 0, 2 0.64 0.64 1)

W=""
stats "../data/pathLengthHistory.txt" u 0:(W=W.sprintf("%.1f ",$1)) nooutput
set xrange [*:*]
set yrange [*:*]
labelsCmd="set xtics ("
stats "../data/pathLengthHistory.txt" u 0:(labelsCmd=labelsCmd.sprintf("'%.1f' %d, ",$1, $0+1)) nooutput
labelsCmd=labelsCmd."'' 0)"
set xrange [*:*]
set yrange [*:*]

# get number of records and dimensions
stats "../data/pathLengthHistory.txt" u 1:2 prefix "STATSW" nooutput
stats "../data/pathLengthHistory.txt" u 1:3 prefix "STATST" nooutput
numLogs=STATSW_records

# for each record
do for [n=0:numLogs-1] {
    set multiplot
    
    # left plot
    set origin 0,0
    set size 0.4,1
    
    # plot start and goal
    set object 1 ellipse center 85,47 size 3,3 front fillstyle solid fillcolor rgbcolor "blue"
    set object 2 ellipse center 17,48 size 3,3 front fillstyle solid fillcolor rgbcolor "orange"
    set label 3 "start" at 10,53 front center textcolor rgbcolor "orange"
    set label 4 "goal"  at 90,53 front center textcolor rgbcolor "blue"

    set key outside
    set size ratio 1
    set title "w = ".word(W,n+1)
    unset xlabel
    unset ylabel
    set xrange [0:100]
    set yrange [0:100]
    unset xtics
    unset ytics
    plot sprintf("../data/GridMap_%s.txt", word(W, n+1)) matrix with image title "",\
         sprintf("../data/Path_%s.txt", word(W, n+1)) u 1:2 with line linewidth 2 lc rgbcolor 'red' title ""

    unset object 1
    unset object 2
    unset label 3
    unset label 4

    
    # right top plot
    set origin 0.4,0.5
    set size 0.6,0.5
    
    set key off
    set size ratio 0.35
    unset title
    set xlabel "w"
    set ylabel "path length"
    set xrange [0:numLogs+1]
    set yrange [0:1.2*STATSW_max_y]
    set ytics auto
    eval(labelsCmd)
    plot "../data/pathLengthHistory.txt" every ::0::n using ($0+1):2 with linespoints pt 7 lc rgbcolor 'red' title ""


    # right bottom
    set origin 0.4,0
    set size 0.6,0.5
    unset object 1
    unset object 2
    unset label 3
    unset label 4
    
    set key off
    set size ratio 0.35
    unset title
    set xlabel "w"
    set ylabel "planning time [s]"
    set xrange [0:numLogs+1]
    set yrange [0:1.2*STATST_max_y]
    set ytics auto
    eval(labelsCmd)
    plot "../data/pathLengthHistory.txt" every ::0::n using ($0+1):3 with linespoints pt 7 lc rgbcolor 'blue' title ""
    unset multiplot
    pause 1
}


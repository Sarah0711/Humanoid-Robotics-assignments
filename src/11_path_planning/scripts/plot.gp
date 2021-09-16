#!/usr/bin/gnuplot --persist

set terminal wxt
set xrange [-0.5:9.5]
set yrange [-0.5:9.5]
set terminal wxt
set size ratio -1
set key outside
unset colorbox
set palette gray negative
set xtics out nomirror 0,1,9 scale 0.01,0.01
set ytics out nomirror 0,1,9 scale 0.01,0.01
set mxtics 2
set mytics 2
set grid mxtics front linetype 1 linecolor rgbcolor "gray40"
set grid mytics front linetype 1 linecolor rgbcolor "gray40"

# plot start and goal
set object 1 ellipse center 2,7 size 0.5,0.5 front fillstyle solid fillcolor rgbcolor "cyan"
set object 2 ellipse center 4,9 size 0.5,0.5 front fillstyle solid fillcolor rgbcolor "orange"
set label 3 "start" at 4,8.3 front center textcolor rgbcolor "orange"
set label 4 "goal"  at 2,7.8 front center textcolor rgbcolor "cyan"

# get number of records
stats "../data/log.txt" nooutput
numLogs=STATS_records

# for each record
do for [n=1:numLogs-1] {
    plot "../data/map.txt" matrix with image title "", \
    -1 with line linewidth 2 lc 1 title "path", \
    "../data/log.txt" every ::0::n using (strcol(3) eq "open"  ? $1 : 1/0):2:(2.0*$0/numLogs) pointsize variable lc 2 title "opened node", \
    "../data/log.txt" every ::0::n using (strcol(3) eq "close" ? $1 : 1/0):2:(2.0*$0/numLogs) pointsize variable lc 3 title "closed node"
    pause 0.1
}

# plot the final result including the path
plot "../data/map.txt" matrix with image title "", \
    "../data/path.txt" u 1:2 with line linewidth 2 lc 1 title "path", \
    "../data/log.txt" every ::0::n using (strcol(3) eq "open"  ? $1 : 1/0):2:(2.0*$0/numLogs) pointsize variable lc 2 title "opened node", \
    "../data/log.txt" every ::0::n using (strcol(3) eq "close" ? $1 : 1/0):2:(2.0*$0/numLogs) pointsize variable lc 3 title "closed node"


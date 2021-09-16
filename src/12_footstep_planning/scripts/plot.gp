#!/usr/bin/gnuplot -persist

w = 0.08
h = 0.044

set terminal wxt size 1024,768

set xrange [0:4]
set yrange [0:4]

set size ratio -1
set key off

set object 1000 circle at 1.0,0.2 size 0.07 fill solid 0.25 fc rgb 'blue' front
set object 1001 circle at 2.0,2.0 size 0.07 fill solid 0.25 fc rgb 'blue' front
set label 1002 "start" at 0.9,0.2  front right textcolor rgbcolor 'black'
set label 1003 "goal"  at 2.0,2.15 front center textcolor rgbcolor 'black'

plot "../data/map.png" binary filetype=png dx=0.01 dy=0.01 with rgbimage title ''
pause 0.05

stats "../data/path.txt" nooutput
n=STATS_records

do for [i=0:n-1] {
    set xrange [*:*]
    set yrange [*:*]
    stats '../data/path.txt' every ::i::i u 1:(x=$1) nooutput
    stats '../data/path.txt' every ::i::i u 1:(y=$2) nooutput
    stats '../data/path.txt' every ::i::i u 1:(phi=$3) nooutput
    stats '../data/path.txt' every ::i::i u 1:(foot=strcol(4)) nooutput

    set xrange [0:4]
    set yrange [0:4]

    set object i+1 polygon from \
             x + w * cos(phi) - h * sin(phi), y - w * sin(phi) - h * cos(phi) \
        to   x + w * cos(phi) + h * sin(phi), y - w * sin(phi) + h * cos(phi) \
        to   x - w * cos(phi) + h * sin(phi), y + w * sin(phi) + h * cos(phi) \
        to   x - w * cos(phi) - h * sin(phi), y + w * sin(phi) - h * cos(phi) \
        to   x + w * cos(phi) - h * sin(phi), y - w * sin(phi) - h * cos(phi) \
        fc rgb (foot eq 'left' ? 'green' : 'red') lw 1 front 
    set object i+1 fillstyle solid
        
    if (i > 1) {
        set object (i-1) fillstyle solid 0.25
    }
    plot "../data/map.png" binary filetype=png dx=0.01 dy=0.01 with rgbimage title ''
    pause 0.05
}

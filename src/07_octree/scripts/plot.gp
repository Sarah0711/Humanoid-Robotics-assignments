#!/usr/bin/gnuplot -persist

cube(x1,y1,z1,x2,y2,z2,c) = sprintf("\
%f %f %f %s\n\
%f %f %f %s\n\
%f %f %f %s\n\
%f %f %f %s\n\
%f %f %f %s\n\
\n\
%f %f %f %s\n\
%f %f %f %s\n\
%f %f %f %s\n\
%f %f %f %s\n\
%f %f %f %s\n\
\n\
%f %f %f %s\n\
%f %f %f %s\n\
%f %f %f %s\n\
%f %f %f %s\n\
%f %f %f %s\n\
\n\
%f %f %f %s\n\
%f %f %f %s\n\
%f %f %f %s\n\
%f %f %f %s\n\
%f %f %f %s\n\
\n\n",\
x1,y1,z1,c,\
x1,y1,z2,c,\
x1,y2,z2,c,\
x1,y2,z1,c,\
x1,y1,z1,c,\
x2,y1,z1,c,\
x2,y1,z2,c,\
x2,y2,z2,c,\
x2,y2,z1,c,\
x2,y1,z1,c,\
x1,y1,z1,c,\
x2,y1,z1,c,\
x2,y2,z1,c,\
x1,y2,z1,c,\
x1,y1,z1,c,\
x1,y1,z2,c,\
x2,y1,z2,c,\
x2,y2,z2,c,\
x1,y2,z2,c,\
x1,y1,z2,c)

CMD=''
stats '../data/result.txt' u 1:(CMD=CMD.cube($1,$2,$3,$4,$5,$6,strcol(8))) nooutput
set print '../data/tmp.txt'
print CMD
unset print

set xrange [-2:14]
set yrange [-8:8]
set zrange [-5:11]
set xlabel "x"
set ylabel "y"
set zlabel "z"
set view 75,325
set view equal xyz

unset key
unset colorbox

set cbrange [1:3]
set palette defined (\
1 '#ff4c4d',\
2 '#ce4c7d',\
3 '#ae559e')

set style line  1 lc rgb '#aa0505' lt 1 lw 0.5
set style line  2 lc rgb '#111188' lt 1 lw 0.5
set style line  3 lc rgb '#8b0b74' lt 1 lw 0.5

set pm3d depthorder hidden3d 1
set pm3d explicit
set ticslevel 0

splot '../data/tmp.txt' u (strcol(4) eq 'OCCUPIED' ? $1 : 1/0):2:3 with pm3d linestyle 1, \
      '../data/tmp.txt' u (strcol(4) eq 'FREE'     ? $1 : 1/0):2:3 with line linestyle 2

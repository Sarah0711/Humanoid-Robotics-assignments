#!/usr/bin/gnuplot --persist

reset
set terminal wxt size 1024,512

radius=0.008               # radius for the small moment indicators
radiusmcom=0.05            # radius for the moment indicator of the CoM
momentscale=1.             # scales the moment arrows
forcescale=0.1             # scales the external force vector
eps=0.3                    # epsilon for calculating moment arrow tips

set macros

# define line and point styles
mgitype_a="head size 0.005,15 filled"            # gi moment arrow heads
mctype_a=" head size 0.005,15 filled"            # contact moment arrow heads
mcomtype_a="head size 0.02,15 filled"            # CoM moment arrow heads
mgitype=" lc rgbcolor 'red' lw 2"                # gi moment arcs
mctype="  lc rgbcolor 'blue' lw 2"               # contact moment arcs
mcomtype="lc rgbcolor '#AA5500' lw 2"            # CoM moment arcs
zmpstyle="lc rgbcolor '#00AA00' pt 3 ps 2 lw 3"  # zero moment point
robotstyle="lc rgbcolor 'black' lw 2 lt 1"       # robot silhouette
pressurestyle="pt 7 lc rgbcolor 'magenta'"       # ground pressure
comstyle="lw 3 pt 2 lc rgbcolor '#AA5500'"       # center of mass
extforcestyle="lw 3 lc rgbcolor '#00AAAA' lt 1"  # external force
extforce_a="head size screen 0.02,15 filled"     # external force arrow tip
groundstyle="lw 1 lc rgbcolor '#AAAAAA' lt 1"    # ground

unset ytics
unset xtics
unset border
unset title

# extract time steps from log file
T=''
stats '../data/log.txt' u 1:(T=T.sprintf(" %4.2f", $1)) name "LOG" nooutput

# for each time step
do for [i=1:LOG_records] {
    t=word(T,i)      # current time

    # get center of mass, external force, and attack point 
    set xrange [*:*]
    set yrange [*:*]
    stats '../data/log.txt' u 0:(abs($1-t)<1e-3 ? cx=$5 : 1/0) nooutput
    stats '../data/log.txt' u 0:(abs($1-t)<1e-3 ? cy=$6 : 1/0) nooutput    
    stats '../data/log.txt' u 0:(abs($1-t)<1e-3 ? cz=$7 : 1/0) nooutput
    stats '../data/log.txt' u 0:(abs($1-t)<1e-3 ? ex=$8 : 1/0) nooutput
    stats '../data/log.txt' u 0:(abs($1-t)<1e-3 ? ey=$9 : 1/0) nooutput    
    stats '../data/log.txt' u 0:(abs($1-t)<1e-3 ? ez=$10 : 1/0) nooutput
    stats '../data/log.txt' u 0:(abs($1-t)<1e-3 ? px=$11 : 1/0) nooutput
    stats '../data/log.txt' u 0:(abs($1-t)<1e-3 ? py=$12 : 1/0) nooutput    
    stats '../data/log.txt' u 0:(abs($1-t)<1e-3 ? pz=$13 : 1/0) nooutput

    
    # get the tipping moment acting on the center of mass
    stats '../data/log.txt' u 0:(abs($1-t)<1e-3 ? mcom=$18 : 1/0) nooutput

    # moment arcs
    W1=''
    file=sprintf('../data/moments-%s.txt',t)
    set xrange [*:*]
    set yrange [*:*]
    stats file u 1:(abs($2)<1e-5?W1=W1.sprintf("\
        plot [t=%f:%f] (%f+radius*cos(t)),(radius*sin(t)) title '' @mgitype;\
        plot [t=%f:%f] (%f+radius*cos(t)),(radius*sin(t)) title '' @mctype;",\
        $4>0 ? pi :  0, $4 > 0 ? pi-momentscale*$4 :   -momentscale*$4, $1, \
        $8>0 ? 0  : pi, $8 > 0 ?   -momentscale*$8 : pi-momentscale*$8, $1):(1)) nooutput
    if (abs(mcom) > 1e-3) {   
      W1=W1.sprintf("plot [t=%f:%f] (%f + radiusmcom*cos(t)),(%f + radiusmcom*sin(t)) title '' @mcomtype;",\
        mcom > 0 ? pi : 0, mcom > 0 ? pi-momentscale*mcom : -momentscale*mcom, cx, cz)      
    }        

    # moment arrowheads
    W2=''
    stats file u 1:((abs($2)<1e-5 && abs($4)>1e-5)?W2=W2.sprintf("set arrow %d from %f,%f to %f,%f @mgitype @mgitype_a; ", \
    $0+1000,$1+radius*cos($4>0 ? pi-momentscale*$4+eps/2 : -momentscale*$4-eps/2),   radius*sin($4>0 ? pi-momentscale*$4+eps/2 : -momentscale*$4-eps/2),\
            $1+radius*cos($4>0 ? pi-momentscale*$4-eps/2 : -momentscale*$4+eps/2),   radius*sin($4>0 ? pi-momentscale*$4-eps/2 : -momentscale*$4+eps/2)):(1)) nooutput

    stats file u 1:((abs($2)<1e-5 && abs($8)>1e-5)?W2=W2.sprintf("set arrow %d from %f,%f to %f,%f @mctype @mctype_a; ", \
    $0+2000,$1+radius*cos($8>0 ?   -momentscale*$8+eps/2 : pi-momentscale*$8-eps/2), radius*sin($8>0 ?   -momentscale*$8+eps/2 : pi-momentscale*$8-eps/2),\
            $1+radius*cos($8>0 ?   -momentscale*$8-eps/2 : pi-momentscale*$8+eps/2), radius*sin($8>0 ?   -momentscale*$8-eps/2 : pi-momentscale*$8+eps/2)):(1)) nooutput

    if (abs(mcom) > 1e-3) { 
        W2=W2.sprintf("set arrow %d from %f,%f to %f,%f @mcomtype @mcomtype_a; ", \
            4, cx+radiusmcom*cos(mcom>0 ? pi-momentscale*mcom+eps/2 : -momentscale*mcom-eps/2), cz+radiusmcom*sin(mcom>0 ? pi-momentscale*mcom+eps/2 : -momentscale*mcom-eps/2),\
               cx+radiusmcom*cos(mcom>0 ? pi-momentscale*mcom-eps/2 : -momentscale*mcom+eps/2), cz+radiusmcom*sin(mcom>0 ? pi-momentscale*mcom-eps/2 : -momentscale*mcom+eps/2))
    }
    
    # delete arrowheads after plotting
    W3=''
    stats file u 1:((abs($2)<1e-5 && abs($4)>1e-5) ? W3=W3.sprintf("unset arrow %d;", $0+1000):(1)) nooutput
    stats file u 1:((abs($2)<1e-5 && abs($8)>1e-5) ? W3=W3.sprintf("unset arrow %d;", $0+2000):(1)) nooutput    
    if (abs(mcom) > 1e-3) { 
        W3=W3."unset arrow 4"
    }
    
    # begin plotting
    set multiplot
    
    # left window: full robot
    set size 0.3,1
    set origin 0,0
    set size ratio -1
    set key off
    set xrange [-0.2:0.11]
    set yrange [-0.05:0.75]
    
    # robot silhouette and ground
    plot '+' u 1:(0) w l @groundstyle title "", "../data/robot.txt" u (cx+$1):(cz+$2) w l @robotstyle title ""

    # moment arcs
    set parametric    
    eval W1 
    unset parametric
    
    # moment arrow heads (will be plotted with next plot command)
    eval W2
    
    if (abs(mcom) > 1e-3) {
        set label 5 "robot tips over" center at cx, cz - 0.7*radiusmcom
    }
     
    # ZMP
    plot '../data/log.txt' u (abs($1-t)<1e-3 ? $2 : 1/0):(0) @zmpstyle title ""
    
    # delete arrow heads so that they are plotted only once
    eval W3
    
    if (abs(mcom) > 1e-3) {
        unset label 5
    }
    
    # Ground pressure
    plot file u 1:(abs($2)<1e-3 ? -0.04 : 1/0):(2.*$11) with points pointsize variable @pressurestyle title ""

    # External force and COM
    set arrow 3 from px-forcescale*ex,pz-forcescale*ez to px,pz @extforcestyle @extforce_a
    plot '+' u (cx):(cz) w p @comstyle title ""
    unset arrow 3    

    # right top plot: legend
    set size 0.7,0.3
    set origin 0.3,0.7
    set key outside top right spacing 1.1
    set xrange [0:1]
    set yrange [0:1]
    plot 1/0 w p @comstyle title "center of mass", \
         1/0 w vectors @extforcestyle title "external force", \
         1/0 w p @zmpstyle title "zero moment point", \
         1/0 w p ps 1.5 @pressurestyle title "ground pressure distribution"


    # right bottom plot: close up of the robot's foot with moments and pressure
    set size 0.7,0.7
    set origin 0.3,0
    set size ratio -1
    set key off
    set xrange [-0.2:0.11]
    set yrange [-0.05:0.1]

    # robot silhouette and ground
    plot '+' u 1:(0) w l @groundstyle title "", "../data/robot.txt" every ::0::8 u (cx+$1):(cz+$2) w l @robotstyle title ""
        
    # moment arcs
    set parametric
    eval W1
    unset parametric
    
    # moment arrow heads (will be plotted with next plot command)
    eval W2
        
    # ZMP
    plot '../data/log.txt' u (abs($1-t)<1e-3 ? $2 : 1/0):(0) @zmpstyle title ""
    
    # delete arrow heads so that they are plotted only once    
    eval W3

    
    # Ground pressure
    plot file u 1:(abs($2)<1e-3 ? -0.02 : 1/0):(2.*$11) with points pointsize variable @pressurestyle title ""

    # done plotting
    unset multiplot
    
    # pause the frame for the animation
    pause 0.2
}

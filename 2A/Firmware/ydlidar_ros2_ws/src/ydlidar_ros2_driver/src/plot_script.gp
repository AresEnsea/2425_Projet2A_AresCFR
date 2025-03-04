set title "Sine Wave"
set xlabel "x"
set ylabel "sin(x)"
plot "data.dat" using 1:2 with lines title "Sine Curve"
pause -1

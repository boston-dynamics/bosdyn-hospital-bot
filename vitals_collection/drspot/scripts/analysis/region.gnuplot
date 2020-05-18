set terminal x11 size 2000,1000
set datafile separator ','

set multiplot layout 4, 1 title "Region stats" font ",14"
set tmargin 2

set title "Raw color channels"
plot raw using 1:2 with lines, '' using 1:3 with lines, '' using 1:4 with lines, '' using 1:5 with lines
#
set title "Normalized color channels"
plot normalized using 1:2 with lines, '' using 1:3 with lines, '' using 1:4 with lines, '' using 1:5 with lines
#
set title "Pulse signals for each candidate SpO2 level"
plot pulse using 1:2 with lines, '' using 1:3 with lines, '' using 1:4 with lines, '' using 1:5 with lines, '' using 1:6 with lines, '' using 1:7 with lines, '' using 1:8 with lines, '' using 1:9 with lines
#
set title "Frequency domain of pulse signal for each candidate SpO2 level"
plot freq using 1:2 with lines, '' using 1:3 with lines, '' using 1:4 with lines, '' using 1:5 with lines, '' using 1:6 with lines, '' using 1:7 with lines, '' using 1:8 with lines, '' using 1:9 with lines
#

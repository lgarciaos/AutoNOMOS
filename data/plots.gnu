#stats "data_31-07-18_195547.txt" u 6:(stringcolumn(2) eq "SST"?$11:1/0)

print ARG1


set key left
set terminal postscript enhanced eps color

set output sprintf("%s/comp_time_%s.eps", ARG1, ARG1)
set title "Computational time"
set xlabel 'Iterations'
set ylabel 'time (s)'
set logscale x 2
plot ARG2 using 6:(stringcolumn(2) eq "SST"?$4:1/0) with\
  points pt 6 title "SST","" using 6:(stringcolumn(2) eq "RRT"?$4:1/0) title "RRT"
#reset

set output sprintf("%s/nodes_%s.eps", ARG1, ARG1)
set title "Total nodes"
set xlabel 'Iterations'
set ylabel 'nodes'
set logscale x 2
plot ARG2  using 6:(stringcolumn(2) eq "SST"?$8:1/0) with\
  points pt 6 title "SST","" using 6:(stringcolumn(2) eq "RRT"?$8:1/0) title "RRT"
#reset

set output sprintf("%s/sln_q_%s.eps", ARG1, ARG1)
set title "Solution Quality"
set xlabel 'Iterations' left
set ylabel 'Quality' left
set yrange [-1:*]
set logscale x 2


plot ARG2 using 6:(stringcolumn(2) eq "SST"?$11:1/0) with\
  points pt 6 title "SST","" using 6:(stringcolumn(2) eq "RRT"?$11:1/0):\
  (stringcolumn(2) eq "SST"?($11 - $13):1/0) with points title "RRT"

#reset
set output sprintf("%s/valid_sln%s.eps", ARG1, ARG1)
set title "Goal Reached"
set xlabel 'Iterations' left
#set ylabel '% ' left
set format y '%2.0f%%'
set yrange [0:100]
set logscale x 2

#awk_script = sprintf("<awk '{if ($11 == 0){print $1" "$2" "$5" "$6" "$9$10" "$11" "$12" "$13 }}'
#    %s | uniq -c", ARG2)
awk_script_random = sprintf("awk '{print $1\" \"$2\" \"$5\" \"$6\" \"$12\" \"$13; \
  if ($11 == 0){print $1\" \"$2\" \"$5\" \"$6\" \"$12\" \"$13 }}' \
  %s | uniq -c", ARG2)
awk_script_bangbang = sprintf("awk '{print $1\" \"$2\" \"$5\" \"$6\" \"$12\" \"$13; \
  if ($11 == 0){print $1\" \"$2\" \"$5\" \"$6\" \"$12\" \"$13 }}' \
  %s | uniq -c", ARG3)
#print awk_script

plot "< ".awk_script_random." | grep SST " using 5:((20 - $1)*10) title \
        "SST Random" with linespoints pt 4, \
     "< ".awk_script_random." | grep RRT " using 5:((20 - $1)*10) title \
        "RRT Random" with linespoints pt 4
     #for[i=1:11] "< ".awk_script_sst." | grep SST | grep ".(1000000/(2**(12 -i))) \
    #    using 1 title "SST - Random: Sol"
  #"< ".awk_script." | grep RRT" using 5:1 title "SST - Random"

reset

set output sprintf("%s/multiplot%s.eps", ARG1, ARG1)
set key left font ", 8"

title_ = sprintf("Car (-10, -7.5, 0) to %s", ARG4 )
random_ctrl_str = "< cat ".ARG2." | grep RANDOM_CTRL"
bangbang_ctrl_str = "< cat ".ARG3." | grep BANG_BANG"
set multiplot layout 2,2 title title_ font ",14"
set xtics rotate
set bmargin 5
###############################################################################
set title "Computational time"
set xlabel 'Iterations'
set ylabel 'Time (s)'
set logscale x 2
set xtics (488, 976, 1953, 3906, 7812, 15625, 31250, 62500, 125000, 250000, 500000, 1000000)
plot random_ctrl_str using 6:(stringcolumn(2) eq "SST"?$4:1/0) with points pt 6\
  title "SST Random", random_ctrl_str using 6:(stringcolumn(2) eq \
  "RRT"?$4:1/0) title "RRT Random",\
  bangbang_ctrl_str using 6:(stringcolumn(2) eq "SST"?$4:1/0) with points pt 6 \
  title "SST Bangbang", bangbang_ctrl_str using 6:(stringcolumn(2) eq \
  "RRT"?$4:1/0) title "RRT Bangbang"
###############################################################################
set title "Total Nodes"
set xlabel 'Iterations'
set ylabel 'Nodes'
set logscale x 2
plot random_ctrl_str  using 6:(stringcolumn(2) eq "SST"?$8:1/0) with points \
  pt 6 title "SST Random", random_ctrl_str using 6:(stringcolumn(2) eq \
  "RRT"?$8:1/0) title "RRT Random", \
  bangbang_ctrl_str  using 6:(stringcolumn(2) eq "SST"?$8:1/0) with points \
  pt 6 title "SST Bangbang", bangbang_ctrl_str using 6:(stringcolumn(2) eq \
  "RRT"?$8:1/0) title "RRT Bangbang"
###############################################################################
set title "Solution Quality"
set xlabel 'Iterations' left
set ylabel 'Quality' left
set yrange [-1:*]
set logscale x 2
plot random_ctrl_str using 6:(stringcolumn(2) eq "SST"?$11:1/0) with\
  points pt 6 title "SST Random", random_ctrl_str using 6:(stringcolumn(2) eq \
  "RRT"?$11:1/0):(stringcolumn(2) eq "SST"?($11 - $13):1/0) with points title \
  "RRT Random", \
  bangbang_ctrl_str using 6:(stringcolumn(2) eq "SST"?$11:1/0) with points pt 6\
  title "SST Bangbang", bangbang_ctrl_str using 6:(stringcolumn(2) eq \
  "RRT"?$11:1/0):(stringcolumn(2) eq "SST"?($11 - $13):1/0) with points title \
  "RRT Bangbang"
###############################################################################
set title "Goal Reached"
set xlabel 'Iterations' left
unset ylabel
#set ylabel '% Solved' left
set format y '%2.0f%%'
set yrange [0:100]
set logscale x 2
plot "< ".awk_script_random." | grep SST | grep RANDOM_CTRL" using 5:((20 - $1)*10) title \
        "SST Random" with linespoints pt 4, \
     "< ".awk_script_random." | grep RRT | grep RANDOM_CTRL" using 5:((20 - $1)*10) title \
        "RRT Random" with linespoints pt 4, \
     "< ".awk_script_bangbang." | grep SST | grep BANG_BANG" using 5:((20 - $1)*10) title \
        "SST Bangbang" with linespoints pt 4, \
     "< ".awk_script_bangbang." | grep RRT | grep BANG_BANG" using 5:((20 - $1)*10) title \
        "RRT Bangbang" with linespoints pt 4
unset multiplot
























#

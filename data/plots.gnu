# TO RUN THIS SCRIPT:
# gnuplot -c plots.gnu dir_to_save 
# sst-rrt_rand_file sst-rrt_bang_file dirt_rand_file dirt_bang_file "(x_g, y_g, th_g)"

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
awk_script_dirt_rctrl = sprintf("awk '{print $1\" \"$2\" \"$5\" \"$6\" \"$12\" \"$13; \
  if ($11 == 0){print $1\" \"$2\" \"$5\" \"$6\" \"$12\" \"$13 }}' \
  %s | uniq -c", ARG4)
awk_script_dirt_bang = sprintf("awk '{print $1\" \"$2\" \"$5\" \"$6\" \"$12\" \"$13; \
  if ($11 == 0){print $1\" \"$2\" \"$5\" \"$6\" \"$12\" \"$13 }}' \
  %s | uniq -c", ARG5)
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

title_ = sprintf("Car (-10, -7.5, 0) to %s", ARG6)
random_ctrl_str = "< cat ".ARG2." | grep RANDOM_CTRL"
bangbang_ctrl_str = "< cat ".ARG3." | grep BANG_BANG"
dirt_rctrl_str = "< cat ".ARG4." | grep RANDOM_CTRL"
dirt_bang_str = "< cat ".ARG5." | grep BANG_BANG"

cl_sst_rand = "#ff00b6"
cl_rrt_rand = "#02ff20"
cl_dir_rand = "#ff0000"
cl_sst_bang = "#56003d"
cl_rrt_bang = "#004c09"
cl_dir_bang = "#ff5c05"

set multiplot layout 3,2 margins 0.1,.9,0.01,0.9 spacing 0.1, 0.075 title title_ font ",14"
# set offset 0, 0.5
# set offset 0,.5,graph 0.05, graph 0.05
# set bmargin 0
# set tmargin 0
set format y "%0.1s%c"
###############################################################################
set size 0.5,0.5
# set offset 0.2, 0
set title "Computational time"
# set xlabel 'Iterations'
unset xlabel
set ylabel 'Time (s)' offset 2
set logscale x 2
set logscale y 2
unset key
set format x ''
set xtics (488, 976, 1953, 3906, 7812, 15625, 31250, 62500, 125000, 250000, 500000, 1000000) 
# unset xtics 
plot random_ctrl_str using 6:(stringcolumn(2) eq "SST"?$4:1/0) with points pt 6 lt rgb cl_sst_rand title "SST Random"  ,\
     random_ctrl_str using 6:(stringcolumn(2) eq "RRT"?$4:1/0) with points pt 6 lt rgb cl_rrt_rand title "RRT Random",\
     bangbang_ctrl_str using 6:(stringcolumn(2) eq "SST"?$4:1/0) with points pt 6 lt rgb cl_sst_bang title "SST Bangbang",\
  bangbang_ctrl_str using 6:(stringcolumn(2) eq "RRT"?$4:1/0) with points pt 6 lt rgb cl_rrt_bang title "RRT Bangbang",\
  dirt_rctrl_str using 6:(stringcolumn(2) eq "DIRT"?$4:1/0) with points pt 6 lt rgb cl_dir_rand title "DIRT Random",\
  dirt_bang_str using 6:(stringcolumn(2) eq "DIRT"?$4:1/0) with points pt 6 lt rgb cl_dir_bang title "DIRT Bangbang"
###############################################################################
# set size 0.5,0.3
set title "Total Nodes"
set xlabel 
set ylabel 'Nodes' offset 2
set logscale x 2
set logscale y 2
plot random_ctrl_str  using 6:(stringcolumn(2) eq "SST"?$8:1/0) with points pt 6 lt rgb cl_sst_rand title "SST Random",\
  random_ctrl_str using 6:(stringcolumn(2) eq "RRT"?$8:1/0) lt rgb cl_rrt_rand title "RRT Random",\
  bangbang_ctrl_str  using 6:(stringcolumn(2) eq "SST"?$8:1/0) with points pt 6 lt rgb cl_sst_bang title "SST Bangbang",\
  bangbang_ctrl_str using 6:(stringcolumn(2) eq "RRT"?$8:1/0) lt rgb cl_rrt_bang title "RRT Bangbang",\
  dirt_rctrl_str using 6:(stringcolumn(2) eq "DIRT"?$8:1/0) with points pt 6 lt rgb cl_dir_rand title "DIRT Random",\
  dirt_bang_str using 6:(stringcolumn(2) eq "DIRT"?$8:1/0) with points pt 6 lt rgb cl_dir_bang title "DIRT Bangbang"
###############################################################################
# set tmargin 5

# set size 0.5,0.4
set format x 
set xtics (488, 976, 1953, 3906, 7812, 15625, 31250, 62500, 125000, 250000, 500000, 1000000)
set xtics rotate
set title "Solution Quality"
set xlabel 'Iterations' left
set ylabel 'Quality (Duration)' left
set yrange [-1:*]
# set format y "%T"
unset format y
unset logscale y 
set logscale x 2
plot random_ctrl_str using 6:(stringcolumn(2) eq "SST"?$11:1/0) with points pt 6 lt rgb cl_sst_rand title "SST Random",\
     random_ctrl_str using 6:(stringcolumn(2) eq "RRT"?$11:1/0):(stringcolumn(2) eq "SST"?($11 - $13):1/0) \
        with points lt rgb cl_rrt_rand title "RRT Random", \
     bangbang_ctrl_str using 6:(stringcolumn(2) eq "SST"?$11:1/0) with points pt 6 lt rgb cl_sst_bang title "SST Bangbang",\
     bangbang_ctrl_str using 6:(stringcolumn(2) eq "RRT"?$11:1/0):(stringcolumn(2) eq "SST"?($11 - $13):1/0) \
        with points lt rgb cl_rrt_bang title "RRT Bangbang",\
     dirt_rctrl_str using 6:(stringcolumn(2) eq "DIRT"?$11:1/0) with points pt 6 lt rgb cl_dir_rand title "DIRT Random",\
     dirt_bang_str using 6:(stringcolumn(2) eq "DIRT"?$11:1/0) with points pt 6 lt rgb cl_dir_bang title "DIRT Bangbang"
###############################################################################
set size 0.5,0.8
set title "Goal Reached"
set xlabel 'Iterations' left
unset ylabel
#set ylabel '% Solved' left
set format y '%2.0f%%'
set yrange [0:100]
set logscale x 2
plot "< ".awk_script_random." | grep SST | grep RANDOM_CTRL" using 5:((20 - $1)*10) title \
        "SST Random" with linespoints pt 4 lt rgb cl_sst_rand, \
     "< ".awk_script_random." | grep RRT | grep RANDOM_CTRL" using 5:((20 - $1)*10) title \
        "RRT Random" with linespoints pt 4 lt rgb cl_rrt_rand , \
     "< ".awk_script_bangbang." | grep SST | grep BANG_BANG" using 5:((20 - $1)*10) title \
        "SST Bangbang" with linespoints pt 4 lt rgb cl_sst_bang , \
     "< ".awk_script_bangbang." | grep RRT | grep BANG_BANG" using 5:((20 - $1)*10) title \
        "RRT Bangbang" with linespoints pt 4 lt rgb cl_rrt_bang ,\
     "< ".awk_script_dirt_rctrl." | grep DIRT | grep RANDOM_CTRL" using 5:((20 - $1)*10) title \
        "DIRT Random" with linespoints pt 4 lt rgb cl_dir_rand ,\
     "< ".awk_script_dirt_bang." | grep DIRT | grep BANG_BANG" using 5:((20 - $1)*10) title \
        "DIRT Bangbang" with linespoints pt 4 lt rgb cl_dir_bang 
###############################################################################
reset
set size 1,0.1
# set bmargin 0
# set tmargin 0
set key center center font ",10"
set border 0
unset tics
unset xlabel
unset ylabel
# set key font ",20"
set yrange [0:1]
plot 1.5 t 'SST Random'    lt rgb cl_sst_rand ,\
     1.5 t 'RRT Random'    lt rgb cl_rrt_rand ,\
     1.5 t 'DIRT Random'   lt rgb cl_dir_rand ,\
     1.5 t 'SST Bangbang'  lt rgb cl_sst_bang ,\
     1.5 t 'RRT Bangbang'  lt rgb cl_rrt_bang ,\
     1.5 t 'DIRT Bangbang' lt rgb cl_dir_bang
unset multiplot






















#

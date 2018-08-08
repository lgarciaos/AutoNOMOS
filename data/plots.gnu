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
reset

set output sprintf("%s/nodes_%s.eps", ARG1, ARG1)
set title "Total nodes"
set xlabel 'Iterations'
set ylabel 'nodes'
set logscale x 2
plot ARG2  using 6:(stringcolumn(2) eq "SST"?$8:1/0) with\
  points pt 6 title "SST","" using 6:(stringcolumn(2) eq "RRT"?$8:1/0) title "RRT"
reset

set output sprintf("%s/sln_q_%s.eps", ARG1, ARG1)
set title "Solution Quality"
set xlabel 'Iterations' left
set ylabel 'Quality' left
set yrange [-1:*]
set logscale x 2


plot ARG2 using 6:(stringcolumn(2) eq "SST"?$11:1/0) with\
  points pt 6 title "SST","" using 6:(stringcolumn(2) eq "RRT"?$11:1/0):\
  (stringcolumn(2) eq "SST"?($11 - $13):1/0) with points title "RRT"

reset
set output sprintf("%s/valid_sln%s.eps", ARG1, ARG1)
set style data histograms
set style histogram rowstacked
#set style histogram columnstacked
set title "Solution Quality"
set xlabel 'Iterations' left
set ylabel 'Quality' left
set yrange [0:*]
#set logscale x 2

#awk_script = sprintf("<awk '{if ($11 == 0){print $1" "$2" "$5" "$6" "$9$10" "$11" "$12" "$13 }}'
#    %s | uniq -c", ARG2)
awk_script_sst = sprintf("awk '{if ($11 == 0 && $2 == \"SST\"){print $1\" \"$2\" \"$5\" \
  \"$6\" \"$9$10\" \"$11\" \"$12\" \"$13 }}' %s | uniq -c", ARG2)
awk_script_rrt = sprintf("awk '{if ($11 == 0 && $2 == \"RRT\"){print $1\" \"$2\" \"$5\" \
  \"$6\" \"$9$10\" \"$11\" \"$12\" \"$13 }}' %s | uniq -c", ARG2)
#print  system(awk_script)
#print awk_script
#plot for [i=1:1000] 'data'.i.'.txt' using 1:2 title 'Flow '.i
plot for[i=1:11] "< ".awk_script." | grep SST | grep ".(1000000/(2**(12 -i))) using 1:xticlabels(i) title "SST - Random: No Sol",\
     for[i=1:11] "< ".awk_script." | grep SST | grep ".(1000000/(2**(12 -i))) using 1:xticlabels(i) title "SST - Random: Sol"
  #"< ".awk_script." | grep RRT" using 5:1 title "SST - Random"

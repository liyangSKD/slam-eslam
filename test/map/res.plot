# 1 i << " "
# 2 forward[i] << " "
# 3 height[i].mean() << " "
# 4 height[i].stdev() << " "
# 5 sqrt(z_variance[i]) << " "
# 6 map_z[i].mean() << " "
# 7 map_z[i].stdev() << " "
# 8 map_stdev[i] << " "
# 9 height[i].min() << " "
# 10 height[i].max() << " "

set xlabel "distance (m)"
set ylabel "z position (m)"
set term pdf
set output "res.pdf"

plot \
    'res.out' using 2:3 t "mean_z meas", \
    'res.out' using 2:4 t "sigma_z meas", \
    'res.out' using 2:5 w l t "sigma_z estimate", \
    'res.out' using 2:6 t "mean_map meas", \
    'res.out' using 2:7 t "sigma_map meas", \
    'res.out' using 2:8 w l t "sigma_map estimate"


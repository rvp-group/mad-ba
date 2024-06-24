### TODO
-math easy - no averaging before optims above and without robustifier and faster, lm - output1
-math easy - no averaging before optims above and without robustifier and faster, gn 1e1 - output2
-math easy - no averaging before optims above and without robustififfier, split with solver lm - output3


# Done 
- math_easy - commented averraginsurfels after optimization and before optimizing the poses, robustifier 0.1, lm - the RMSE is decreasing and lower than originally, but after 25 iteration MAX error increases
- math_easy - commented averraginsurfels after optimization and before optimizing the poses, robustifier 0.1, gn 1e3 - output - the RMSE is decreasing and lower than originally, but after 25 iteration MAX error increases 
- math_easy - commented averraginsurfels after optimization and before optimizing the poses - output - looks like that helps
- math_easy alternating with gn damping 1e4, loam, factor robustifier 0.1 - output
- math_easy alternating with gn damping 1e3, loam, factor robustifier 0.1, only 1/4 trajectory  - output - same issue
- math_easy alternating with gn damping 1e3, loam, factor robustifier 0.1, only half trajectory  - output2 - the same issue, max error for the first pose is increasing
- math_easy alternating with gn damping 1e3, loam, factor robustifier 0.1, poses first  - output - the same issue, max error for the first pose is increasing
- math_easy alternating with gn damping 1e4, loam, factor robustifier 0.2  - output2 - error is smoothly increasing
- math_easy alternating with gn damping 1e4, loam, factor robustifier 0.1  - output1 - error is smoothly increasing
- math_easy alternating with gn damping 1e4, loam, factor robustifier  - output1 -- looks like the error is decreasing so looks good, but need more iterations
- math_easy solver only ln, loam, no robustifier no damping, decimating surfels 1/10 - output2 - a bit wavy, but still results are OK
- math_easy solver only ln, loam, no robustifier no damping, decimating surfels 1/5 - outpu2 - a bit wavy plot, but results are similar to all surfels
- math_easy alternating with gn damping 1e5, loam, factor robustifier  - output1 - better, but too smoth i think
- math easy - robustifier, gn, damping 1e3, loam, robustifier 0.1, output1 - dim_variables= 12954; dim_factors= 46665940; num_inliers= 46170894; chi_inliers= 166977.734375; num_outliers= 495046;
- math easy - robustifier, gn, damping 1e3, loam , robustifier 0.01, output2 - dim_variables= 12954; dim_factors= 45501101; num_inliers= 34752495; chi_inliers= 53099.8320312; num_outliers= 10748606;
- math easy - gauss newton damping = 1e-3, alternating, loam.  output2 - nothing changed
- math easy - gauss newton damping = 1e3, alternatin, loam, - the poses are damped and smoothed, but still error increases
-math_easy with loam - alternating but 1 inner loop and damping lambda - change inital value of surfels to average mean and normal - output(1) - troche inaczej ale blad rms i max rosna
-math_easy with loam - alternating but 1 inner loop and damping lambda and m-estiator for surfels- change inital value of surfels to average mean and normal - output2 - j.w
-math_easy with loam - alternating - change inital value of surfels to average mean and normal - not good, the first pose max error increases, and rmse also
-math_easy with loam - solver only - change inital value of surfels to average mean and normal - looks GOOD, the max error is not increasing, the error is probably even smaller
-quad_easy again to check if max error increases - with the LOAM as initial trajectory, the max error increases slightly at 1st iteration but then drops and stays constant
-math_easy with solver only and fixed first pose with mad_icp as initial - the same problem, max error is increasing
-math_easy with solver only and fixed first pose but skipped first one (0th) - the same problem, error is increasing
-math_easy with solver only and fixed first pose without skipping (baseline) - yes the max error is increasing
-math_easy with alternating solver with damping to surfel optim - output2
-math_easy with alternating solver with unfixed first pose - output
-math_easy with solver only and unfixed first pose - the results are OK, the maximum error is not increasing
-math_easy with alternating optimization with changed parameters - smaller parameters for matchich gives similar results
-math_easy with solver only optimization and many iterations to see if it will converge eventually - the maximum error increases but only because of the first pose

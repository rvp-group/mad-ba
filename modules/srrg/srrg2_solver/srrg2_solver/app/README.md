## generate a simulated graph 

first try to understand parsing arguments using 100% of your brain by typing 
`rosrun srrg2_solver solver_app_plgo_simulator -h`

if it is unclear read below



you can generate 3 types of graph `se3/se3_ba/sim3` where `se3_ba` includes projective info (for bundle adjustement problems for instance) and and `sim3` stands for similiarity

#### example create a sphere se3 pose-graph (noise free)

`rosrun srrg2_solver solver_app_plgo_simulator -gt se3 -mt sphere -mp -mo -o output_graph.boss`

other then `sphere` types you can generate `manhattan` and `torus` worlds (type of motion), in order to see what options are available for each type of world and how to customize them, type (in between `<>` only one of them)

`rosrun srrg2_solver solver_app_plgo_simulator -gt se3 -mt <torus/manhattan/sphere>`

this will return a list of arguments opened for each world. For instance in `sphere` we can change `radius, azimuth steps, elevation steps`, in order to modify these params type their numerical value respecting the output order that you see in the terminal using `-ma`, like

`rosrun srrg2_solver solver_app_plgo_simulator -gt se3 -mt sphere -ma 20 50 50 -o output_graph.boss`

#### example create a torus se3 pose-landmark graph (noise free)

`rosrun srrg2_solver solver_app_plgo_simulator -gt se3 -mt torus -ml -mp -mo -o output_graph.boss`

now you can play with args like `-lpp` `-lsr` in order to make your system less or more dense, you can do the same with pose args

## view a created graph

make sure you have `srrg2_qgl_viewport` linked to your workspace :)

`rosrun srrg2_solver_gui test_graph_viewer output_graph.boss`

if it is cold outside warm your hands close to your computer fan

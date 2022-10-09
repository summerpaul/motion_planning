project(global_planner)

add_library(global_planner 
astar_search_grid_map.cpp 
kinodynamic_astar_grid_map.cpp
min_collision_risk_planner.cpp
)
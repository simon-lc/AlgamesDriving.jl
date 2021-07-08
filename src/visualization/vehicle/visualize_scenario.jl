################################################################################
# set_scenario!
################################################################################

function set_scenario!(vis::Visualizer, sce::Scenario{T};
	roadway_vis_opts::RoadwayVisualizationOptions=RoadwayVisualizationOptions{T}(),
    player_vis_opts::PlayerVisualizationOptions=PlayerVisualizationOptions{T}(),
	color::Vector{Symbol}=Vector{Symbol}()) where {T}
    clean!(vis)
	set_roadway!(vis, sce.roadway.opts, vis_opts=roadway_vis_opts)
	set_player!(vis, sce.player, vis_opts=player_vis_opts, color=color)
    return nothing
end

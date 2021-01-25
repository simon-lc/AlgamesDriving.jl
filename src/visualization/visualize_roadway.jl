################################################################################
# VisualizationOptions
################################################################################

@with_kw mutable struct RoadwayVisualizationOptions{T}
    # Options
	"Thickness of the object."
	ϵ::T=0.05

	"Line width."
	line_width::T=0.007

	"Bound width."
	bound_width::T=0.02

	"Bound height."
	bound_height::T=0.05

	"Road color."
	road_color::RGB=RGB(27/255, 50/255, 81/255)

	"Line color."
	line_color::RGB=RGB(255/255, 206/255, 68/255)

	"Road color."
	bound_color::RGB=RGB(75/255, 129/255, 194/255)
end

################################################################################
# set_roadway!
################################################################################

function set_roadway!(vis::Visualizer, opts::HighwayRoadwayOptions{T};
    vis_opts::RoadwayVisualizationOptions=RoadwayVisualizationOptions()) where {T}
    clean!(vis)
    ll = opts.lane_length
    lw = opts.lane_width

	ϵ = vis_opts.ϵ
	liw = vis_opts.line_width
	bw = vis_opts.bound_width
	bh = vis_opts.bound_height

    road = Rect3D(Vec(0.0, -lw, -ϵ), Vec(ll, 2lw, ϵ))
    line = Rect3D(Vec(0.0, -liw/2, 0.0), Vec(ll, liw, liw))
	left_bound = Rect3D(Vec(0.0, lw, -ϵ), Vec(ll, bw, ϵ+bh))
    right_bound = Rect3D(Vec(0.0, -lw-bw, -ϵ), Vec(ll, bw, ϵ+bh))

    setobject!(vis["env/roadway"]["road"], road, MeshPhongMaterial(color=vis_opts.road_color))
	setobject!(vis["env/roadway"]["line"], line, MeshPhongMaterial(color=colorant"yellow"))
	setobject!(vis["env/roadway"]["left_bound"], left_bound, MeshPhongMaterial(color=vis_opts.bound_color))
	setobject!(vis["env/roadway"]["right_bound"], right_bound, MeshPhongMaterial(color=vis_opts.bound_color))
    return nothing
end

function set_roadway!(vis::Visualizer, opts::MergingRoadwayOptions111{T};
    vis_opts::RoadwayVisualizationOptions=RoadwayVisualizationOptions()) where {T}
    clean!(vis)
    ll = opts.lane_length
	lw = opts.lane_width
	mp = opts.merging_point
	θ = opts.merging_angle

	ϵ = vis_opts.ϵ
	liw = vis_opts.line_width
	bw = vis_opts.bound_width
	bh = vis_opts.bound_height

	road = Rect3D(Vec(0.0, -lw, -ϵ), Vec(ll, 2lw, ϵ))
	line = Rect3D(Vec(0.0, -liw/2, 0.0), Vec(ll, liw, liw))
	left_bound = Rect3D(Vec(0.0, lw, -ϵ), Vec(ll, bw, ϵ+bh))
	right_bound = Rect3D(Vec(mp+lw*tan(θ), -lw-bw, -ϵ), Vec(ll-mp-lw*tan(θ), bw, ϵ+bh))
	merging_road = Rect3D(Vec(0., 0., -ϵ), Vec(mp, lw/cos(θ), ϵ))
	merging_right_bound = Rect3D(Vec(0., 0., -ϵ), Vec(mp, bw, ϵ+bh))

    setobject!(vis["env/roadway"]["road"], road, MeshPhongMaterial(color=vis_opts.road_color))
	setobject!(vis["env/roadway"]["line"], line, MeshPhongMaterial(color=colorant"yellow"))
	setobject!(vis["env/roadway"]["left_bound"], left_bound, MeshPhongMaterial(color=vis_opts.bound_color))
	setobject!(vis["env/roadway"]["right_bound"], right_bound, MeshPhongMaterial(color=vis_opts.bound_color))
	setobject!(vis["env/roadway"]["merging_road"], merging_road, MeshPhongMaterial(color=vis_opts.road_color))
	setobject!(vis["env/roadway"]["merging_right_bound"], merging_right_bound, MeshPhongMaterial(color=vis_opts.bound_color))

	tr_road = compose(Translation(mp, 0., 0.), LinearMap(AngleAxis(θ-pi, 0, 0, 1)))
	tr_bound = compose(Translation(mp+lw*tan(θ), -lw, 0.), LinearMap(AngleAxis(θ-pi, 0, 0, 1)))
	settransform!(vis["env/roadway"]["merging_road"], tr_road)
	settransform!(vis["env/roadway"]["merging_right_bound"], tr_bound)
	return nothing
end

# vis = Visualizer()
# open(vis)

roadway_opts = MergingRoadwayOptions111()
roadway = build_roadway(roadway_opts)

set_roadway!(vis, roadway_opts)


################################################################################
# AutoProblem
################################################################################

# mutable struct AutoProblem{T}
# 	prob::GameProblem
# 	sce::Scenario{T}
# end
#
# function AutoProblem(prob::GameProblem, sce::Scenario{T}) where {T}
# 	return AutoProblem{T}(prob, sce)
# end
#
# function AutoProblem(sce::Scenario{T}) where {T}
# 	prob = GameProblem(scenario)
# 	return AutoProblem{T}(prob, sce)
# end
#
#
# function visualize!(vis::Visualizer, sce::Scenario{T}, traj::Traj) where {T}
# 	# Visualize a trajectory
#
# 	return nothing
# end
#
# function visualize!(vis::Visualizer, sce::Scenario{T}, x::SVx) where {Svx}
# 	# Visualize a state
#
# 	return nothing
# end

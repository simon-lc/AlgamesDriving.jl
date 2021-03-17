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
    vis_opts::RoadwayVisualizationOptions=RoadwayVisualizationOptions(), k) where {T}
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

    setobject!(vis["env/roadway$k"]["road"], road, MeshPhongMaterial(color=vis_opts.road_color))
	setobject!(vis["env/roadway$k"]["line"], line, MeshPhongMaterial(color=colorant"yellow"))
	setobject!(vis["env/roadway$k"]["left_bound"], left_bound, MeshPhongMaterial(color=vis_opts.bound_color))
	setobject!(vis["env/roadway$k"]["right_bound"], right_bound, MeshPhongMaterial(color=vis_opts.bound_color))
    return nothing
end

function set_roadway!(vis::Visualizer, opts::MergingRoadwayOptions{T};
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

function set_roadway!(vis::Visualizer, opts::FourIntersectionRoadwayOptions{T};
	vis_opts::RoadwayVisualizationOptions=RoadwayVisualizationOptions()) where {T}
	clean!(vis)
	ll = opts.lane_length
    lw = opts.lane_width

	ϵ = vis_opts.ϵ
	liw = vis_opts.line_width
	bw = vis_opts.bound_width
	bh = vis_opts.bound_height

	road_horizontal = Rect3D(Vec(-ll/2, -lw, -ϵ), Vec(ll, 2lw, ϵ))
	line_horizontal_1 = Rect3D(Vec(-ll/2, -liw/2, 0.0), Vec(ll/2 - lw, liw, liw))
	line_horizontal_2 = Rect3D(Vec(lw, -liw/2, 0.0), Vec(ll/2 - lw, liw, liw))

	road_vertical = Rect3D(Vec(-lw, -ll/2, -ϵ), Vec(2lw, ll, ϵ))
	line_vertical_1 = Rect3D(Vec(-liw/2, -ll/2, 0.0), Vec(liw, ll/2 - lw, liw))
	line_vertical_2 = Rect3D(Vec(-liw/2, lw, 0.0), Vec(liw, ll/2 - lw, liw))

	horizontal_left_bound_w = Rect3D(Vec(-ll/2, lw, -ϵ), Vec(ll/2 - lw, bw, ϵ+bh))
    horizontal_right_bound_w = Rect3D(Vec(-ll/2, -lw-bw, -ϵ), Vec(ll/2 - lw, bw, ϵ+bh))

	horizontal_left_bound_e = Rect3D(Vec(lw, lw, -ϵ), Vec(ll/2 - lw, bw, ϵ+bh))
    horizontal_right_bound_e = Rect3D(Vec(lw, -lw-bw, -ϵ), Vec(ll/2 - lw, bw, ϵ+bh))

	# horizontal_left_bound = Rect3D(Vec(-ll/2, lw, -ϵ), Vec(ll, bw, ϵ+bh))
    # horizontal_right_bound = Rect3D(Vec(-ll/2, -lw-bw, -ϵ), Vec(ll, bw, ϵ+bh))

	vert_left_bound_n = Rect3D(Vec(lw, -ll/2, -ϵ), Vec(bw, ll/2 -lw, ϵ+bh))
	vert_right_bound_n = Rect3D(Vec(-lw-bw, -ll/2, -ϵ), Vec(bw, ll/2 - lw, ϵ+bh))

	vert_left_bound_s = Rect3D(Vec(lw, lw, -ϵ), Vec(bw, ll/2 - lw, ϵ+bh))
	vert_right_bound_s = Rect3D(Vec(-lw-bw, lw, -ϵ), Vec(bw, ll/2 - lw, ϵ+bh))

	stop_line_west = Rect3D(Vec(-lw, -lw, 0.0), Vec(liw, lw, liw))
	stop_line_east = Rect3D(Vec(lw, 0, 0.0), Vec(liw, lw, liw))
	stop_line_north = Rect3D(Vec(0, -lw, 0.0), Vec(lw, liw, liw))
	stop_line_south = Rect3D(Vec(-lw, lw, 0.0), Vec(lw, liw, liw))

    setobject!(vis["env/roadway"]["road_horizontal"], road_horizontal, MeshPhongMaterial(color=vis_opts.road_color))
	setobject!(vis["env/roadway"]["line_horizontal_1"], line_horizontal_1, MeshPhongMaterial(color=colorant"yellow"))
	setobject!(vis["env/roadway"]["line_horizontal_2"], line_horizontal_2, MeshPhongMaterial(color=colorant"yellow"))
	setobject!(vis["env/roadway"]["horizontal_left_bound_w"], horizontal_left_bound_w, MeshPhongMaterial(color=vis_opts.bound_color))
	setobject!(vis["env/roadway"]["horizontal_right_bound_w"], horizontal_right_bound_w, MeshPhongMaterial(color=vis_opts.bound_color))
	setobject!(vis["env/roadway"]["horizontal_left_bound_e"], horizontal_left_bound_e, MeshPhongMaterial(color=vis_opts.bound_color))
	setobject!(vis["env/roadway"]["horizontal_right_bound_e"], horizontal_right_bound_e, MeshPhongMaterial(color=vis_opts.bound_color))

	setobject!(vis["env/roadway"]["road_vertical"], road_vertical, MeshPhongMaterial(color=vis_opts.road_color))
	setobject!(vis["env/roadway"]["line_vertical_1"], line_vertical_1, MeshPhongMaterial(color=colorant"yellow"))
	setobject!(vis["env/roadway"]["line_vertical_2"], line_vertical_2, MeshPhongMaterial(color=colorant"yellow"))
	setobject!(vis["env/roadway"]["vertical_left_bound_n"], vert_left_bound_n, MeshPhongMaterial(color=vis_opts.bound_color))
	setobject!(vis["env/roadway"]["vertical_right_bound_n"], vert_right_bound_n, MeshPhongMaterial(color=vis_opts.bound_color))

	setobject!(vis["env/roadway"]["vertical_left_bound_s"], vert_left_bound_s, MeshPhongMaterial(color=vis_opts.bound_color))
	setobject!(vis["env/roadway"]["vertical_right_bound_s"], vert_right_bound_s, MeshPhongMaterial(color=vis_opts.bound_color))

	setobject!(vis["env/roadway"]["stop_line_west"], stop_line_west, MeshPhongMaterial(color=colorant"white"))
	setobject!(vis["env/roadway"]["stop_line_east"], stop_line_east, MeshPhongMaterial(color=colorant"white"))
	setobject!(vis["env/roadway"]["stop_line_north"], stop_line_north, MeshPhongMaterial(color=colorant"white"))
	setobject!(vis["env/roadway"]["stop_line_south"], stop_line_south, MeshPhongMaterial(color=colorant"white"))
    return nothing
end

################################################################################
# VisualizationOptions
################################################################################

@with_kw mutable struct PlayerVisualizationOptions{T}
    # Options
	"Path to the material files."
	material_path::String=joinpath(module_dir(), "resources", "material")

	"Path to the object files."
	object_path::String=joinpath(module_dir(), "resources", "object", "car_geometry.obj")

	"Scaling of the car 3D mesh."
	car_scale::T=1.0

	"Car offset that puts the car is the correct orientation/position."
	car_offset::AbstractVector=[
			LinearMap(AngleAxis(-0.012+pi/200, 0, 1, 0)),
			LinearMap(AngleAxis(pi/2, 0, 0, 1)),
			LinearMap(AngleAxis(pi/2, 1, 0, 0))]

	"Height of the collision avoidance cylinder."
	cylinder_height::T=0.02

	"Fading of the cylinder"
	α::T=0.9

	"Colors."
	colors::Dict{Symbol,RGB}=Dict(
		:cornflowerblue => RGB(100/255, 149/255, 237/255),
		:orange         => RGB(255/255, 125/255,   0/255),
		:forestgreen    => RGB( 34/255, 139/255,  34/255),
		:red            => RGB(255/255,   0/255,   0/255),
		:yellow         => RGB(255/255, 255/255,   0/255),
		:gray           => RGB(127/255, 127/255, 127/255),
		:lime           => RGB(  0/255, 255/255,   0/255),
		)
end


@with_kw mutable struct WaypointVisualizationOptions{T}
    # Options
	"Height of the waypoint cylinder."
	waypoint_height::T=0.015

	"Radius of the waypoint cylinder."
	waypoint_radius::T=0.013

	"Fading of the cylinder"
	α::T=0.9

	"Colors."
	colors::Dict{Symbol,RGB}=Dict(
		:cornflowerblue => RGB(100/255, 149/255, 237/255),
		:orange         => RGB(255/255, 125/255,   0/255),
		:forestgreen    => RGB( 34/255, 139/255,  34/255),
		:red            => RGB(255/255,   0/255,   0/255),
		:yellow         => RGB(255/255, 255/255,   0/255),
		:gray           => RGB(127/255, 127/255, 127/255),
		:lime           => RGB(  0/255, 255/255,   0/255),
		)
end


@with_kw mutable struct LineVisualizationOptions{T}
    # Options

	"Line width."
	line_width::T=8.0

	"Fading of the line"
	α::T=0.9

	"Colors."
	colors::Dict{Symbol,RGB}=Dict(
		:cornflowerblue => RGB(100/255, 149/255, 237/255),
		:orange         => RGB(255/255, 125/255,   0/255),
		:forestgreen    => RGB( 34/255, 139/255,  34/255),
		:red            => RGB(255/255,   0/255,   0/255),
		:yellow         => RGB(255/255, 255/255,   0/255),
		:gray           => RGB(127/255, 127/255, 127/255),
		:lime           => RGB(  0/255, 255/255,   0/255),
		)
end

function get_car_offset(vis_opts::PlayerVisualizationOptions)\
	car_scale = vis_opts.car_scale
	car_offset = vis_opts.car_offset
	offset = compose(compose(compose(
			Translation(0., 0., 0.03*car_scale),
			car_offset[1]),
			car_offset[2]),
			car_offset[3])
	return offset
end

################################################################################
# set_player!
################################################################################

function set_player!(vis::Visualizer, player::Player{T};
    vis_opts::PlayerVisualizationOptions=PlayerVisualizationOptions{T}(),
	color=:orange, key::Int=0) where {T}
    clean!(vis)
	mtlpath = joinpath(vis_opts.material_path , "$(string(color)).mtl")
	objpath = vis_opts.object_path
	col = vis_opts.colors[color]
	α = vis_opts.α
	ch = vis_opts.cylinder_height
	player_path = "env/player_key$key/player$(player.id)"
	offset = get_car_offset(vis_opts)

	# Add car object
	car = ModifiedMeshFileObject(objpath, mtlpath, scale=0.010*vis_opts.car_scale)
	setobject!(vis[player_path*"/car"], car)
	settransform!(vis[player_path*"/car"], offset)
	# Add cylinder
	cyl = Cylinder(Point(0.,0,0), Point(0,0,ch), player.r_col)
	mat = MeshPhongMaterial(color=RGBA(col, α))
	setobject!(vis[player_path*"/cyl"], cyl, mat)
    return nothing
end

function set_player!(vis::Visualizer, player::Vector{Player{T}};
    vis_opts::PlayerVisualizationOptions=PlayerVisualizationOptions{T}(),
	color::Vector{Symbol}=Vector{Symbol}(), key::Int=0) where {T}
	length(color) == 0 ? color = collect(keys(vis_opts.colors)) : nothing
	p = length(player)
	c = length(color)
	for i = 1:p
		set_player!(vis, player[i], vis_opts=vis_opts, color=color[1+(i-1)%c], key=key)
	end
	return nothing
end


################################################################################
# build_waypoint!
################################################################################

function build_waypoint!(vis::Visualizer, player::Player{T}, N::Int;
    vis_opts::WaypointVisualizationOptions=WaypointVisualizationOptions{T}(),
	color=:orange, key::Int=0) where {T}
    clean!(vis)
	col = vis_opts.colors[color]
	α = vis_opts.α
	wh = vis_opts.waypoint_height
	wr = vis_opts.waypoint_radius
	waypoint_path = "env/waypoint_key$key/player$(player.id)"

	# Add waypoint
	for k = 1:N
		wpt = Cylinder(Point(0.,0,0), Point(0,0,wh), wr)
		mat = MeshPhongMaterial(color=RGBA(col, α))
		setobject!(vis[waypoint_path*"/$k"], wpt, mat)
	end
    return nothing
end

function build_waypoint!(vis::Visualizer, player::Vector{Player{T}}, N::Int;
    vis_opts::WaypointVisualizationOptions=WaypointVisualizationOptions{T}(),
	color::Vector{Symbol}=Vector{Symbol}(), key::Int=0) where {T}
	length(color) == 0 ? color = collect(keys(vis_opts.colors)) : nothing
	p = length(player)
	c = length(color)
	for i = 1:p
		build_waypoint!(vis, player[i], N, vis_opts=vis_opts, color=color[1+(i-1)%c], key=key)
	end
	return nothing
end

################################################################################
# set_line_traj!
################################################################################

function set_line_traj!(vis::Visualizer, model::AbstractGameModel, player::Vector{Player{T}}, traj;
    vis_opts::LineVisualizationOptions=LineVisualizationOptions{T}(),
	color::Vector{Symbol}=Vector{Symbol}(), key::Int=0) where {T}
	length(color) == 0 ? color = collect(keys(vis_opts.colors)) : nothing
	p = length(player)
	c = length(color)
	for i = 1:p
		set_line_traj!(vis, model, player[i], traj, vis_opts=vis_opts,
			color=color[1+(i-1)%c], key=key)
	end
	return nothing
end

function set_line_traj!(vis::Visualizer, model::AbstractGameModel, player::Player{T}, traj;
    vis_opts::LineVisualizationOptions=LineVisualizationOptions{T}(),
	color=:orange, key::Int=0) where {T}
    clean!(vis)
	col = vis_opts.colors[color]
	α = vis_opts.α
	lw = vis_opts.line_width
	line_path = "env/line_key$key/player$(player.id)"

	# Add line
	mat = LineBasicMaterial(color=color=RGBA(col, α), linewidth=lw)
	N = length(traj)
	vs = [standardize(model, Algames.state(traj[k]))[player.id] for k=1:N]
	points = [[vs[k].x; vs[k].y; 0.01] for k=1:N]
	for k = 1:N
		setobject!(vis[line_path], Object(PointCloud(points), mat, "Line"))
	end
    return nothing
end

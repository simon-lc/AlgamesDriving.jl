################################################################################
# VisualizationOptions
################################################################################

@with_kw mutable struct PlayerVisualizationOptions{T}
    # Options
	# Options
	"Path to the material files."
	material_path::String=joinpath( Base.@__DIR__, "../../resources/material/")

	"Path to the object files."
	object_path::String=joinpath( Base.@__DIR__, "../../resources/object/car_geometry.obj")

	"Car offset that puts the car is the correct orientation/position."
	car_offset::AffineMap=compose(compose(compose(
			Translation(0., 0., 0.03),
			LinearMap(AngleAxis(-0.012+pi/200, 0, 1, 0))),
			LinearMap(AngleAxis(pi/2, 0, 0, 1))),
			LinearMap(AngleAxis(pi/2, 1, 0, 0)))

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

################################################################################
# set_player!
################################################################################

function set_player!(vis::Visualizer, player::Player{T};
    vis_opts::PlayerVisualizationOptions=PlayerVisualizationOptions{T}(),
	color=:orange) where {T}
    clean!(vis)
	mtlpath = joinpath(vis_opts.material_path , "$(string(color)).mtl")
	objpath = vis_opts.object_path
	col = vis_opts.colors[color]
	α = vis_opts.α
	ch = vis_opts.cylinder_height
	player_path = "env/player/player$(player.id)"
	offset = vis_opts.car_offset

	# Add car object
	car = ModifiedMeshFileObject(objpath, mtlpath, scale=0.010)
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
	color::Vector{Symbol}=Vector{Symbol}()) where {T}
	length(color) == 0 ? color = collect(keys(vis_opts.colors)) : nothing
	p = length(player)
	c = length(color)
	for i = 1:p
		set_player!(vis, player[i], vis_opts=vis_opts, color=color[i%c])
	end
	return nothing
end

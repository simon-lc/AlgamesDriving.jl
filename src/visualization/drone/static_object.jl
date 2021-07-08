################################################################################
# Static Object
################################################################################

function build_traj!(vis::Visualizer, model::AbstractGameModel, traj::Algames.Traj;
		name::Symbol=:Traj, α=0.8, r=0.02)

	default_background!(vis)
	r = convert(Float32, r)
    p = model.p
    pz = model.pz
	d = Algames.dim(model)
	orange_mat, blue_mat, black_mat = get_material(;α=α)

    for t in eachindex(traj)
        s = Algames.state(traj[t])
        for i = 1:p
            x = [s[pz[i][1:d]]; zeros(3-d)]
			# setobject!(vis[name]["player$i"]["t$t"], Sphere(Point3f0(0.0), r), blue_mat)
			setobject!(vis[name]["player$i"]["t$t"], Sphere(Point3f0(0.0), r), orange_mat)
            settransform!(vis[name]["player$i"]["t$t"], MeshCat.Translation(x...))
        end
    end
    return nothing
end

function build_xf!(vis::Visualizer, model::AbstractGameModel, xf::AbstractVector; name::Symbol=:Xf, α=0.8)
	default_background!(vis)
    p = model.p
    pz = model.pz
	d = Algames.dim(model)
	orange_mat, blue_mat, black_mat = get_material(;α=α)

    for i = 1:p
        x = [xf[i][1:d]; zeros(3-d)]
        setobject!(vis[name]["player$i"], Sphere(Point3f0(0.0), 0.05), blue_mat)
        settransform!(vis[name]["player$i"], MeshCat.Translation(x...))
    end
    return nothing
end

function build_door_frame!(vis::Visualizer; door_size=0.10, radius=0.08, room_size=2.0,
	 	name::Symbol=:DoorFrame, size=10, α=1.0)

	orange_mat, blue_mat, black_mat = get_line_material(size, α=α)

	# Point Traj
	ds = door_size/2 + radius
	rs = room_size/2
	door_frame = [[0, ds, ds], [0, -ds, ds], [0, -ds, -ds], [0, ds, -ds], [0, ds, ds]]
	door_frame = [Point(p...) for p in door_frame]
	pierced_wall = [
		[0, ds, ds], [0, rs, rs], [0, ds, ds],
		[0, -ds, ds], [0, -rs, rs], [0, -ds, ds],
		[0, -ds, -ds], [0, -rs, -rs], [0, -ds, -ds],
		[0, ds, -ds], [0, rs, -rs], [0, ds, -ds],
		]
	pierced_wall = [Point(p...) for p in pierced_wall]
	setobject!(vis[name][:lines][:door_frame], MeshCat.Line(door_frame, black_mat))
	setobject!(vis[name][:lines][:pierced_wall], MeshCat.Line(pierced_wall, black_mat))
	return nothing
end

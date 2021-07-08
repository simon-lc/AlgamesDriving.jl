function build_drone!(vis::Visualizer, model::UnicycleGame; name::Symbol=:Robot, r_col=0.08, r_cost=0.5, α=1.0, r=0.15)
	r = convert(Float32, r)
    p = model.p

	obj_path = joinpath(module_dir(), "resources", "object", "drone.obj")
	orange_mat, blue_mat, black_mat_col = get_material(;α=0.3)
	orange_mat, blue_mat, black_mat_cost = get_material(;α=0.1)
	obj = MeshFileObject(obj_path)
    for i = 1:p
		setobject!(vis[name]["player$i"]["body"], obj)
		setobject!(vis[name]["player$i"]["collision"], GeometryBasics.Sphere(Point3f0(0.0), r_col), black_mat_col)
		setobject!(vis[name]["player$i"]["cost"], GeometryBasics.Sphere(Point3f0(0.0), r_cost), black_mat_cost)
		settransform!(vis[name]["player$i"]["body"], MeshCat.LinearMap(r*RotX(pi/2)))
    end
    return nothing
end

function set_drone!(vis::Visualizer, model::UnicycleGame, s::AbstractVector; name::Symbol=:Robot)
    p = model.p
    pz = model.pz
	d = Algames.dim(model)
    for i = 1:p
		x = [s[pz[i][1:d]]; zeros(3-d)]
		θ = s[pz[i][3]]
        settransform!(vis[name]["player$i"], MeshCat.compose(MeshCat.Translation(x...), MeshCat.LinearMap(RotZ(θ))))
    end
    return nothing
end

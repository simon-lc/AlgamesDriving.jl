function set_env!(vis, model::AbstractGameModel, i::Int, x::SVx) where {SVx}
    x = standardize(model, x[model.pz[i]])
    set_env!(vis, x)
    return nothing
end

function set_env!(vis, x::VehicleState)
    settransform!(vis["env"], Translation(-x.x, -x.y, 0.))
    return nothing
end

function set_camera!(vis)
    settransform!(vis["/Cameras/default"], Translation(-0.70, 0.0, 0.04))
end

function set_camera_birdseye!(vis; height::T=7.0) where {T}
    t = Translation(0.0, 0.0, height)
    r = compose(
        LinearMap(AngleAxis(pi/2, 0, 0, 1)),
        LinearMap(AngleAxis(1.0*pi, 0, 1, 0)))
    tr = compose(t,r)
    settransform!(vis["/Cameras/default"], tr)
    setprop!(vis["/Cameras/default/rotated/<object>"], "zoom", 5.0)
    return nothing
end

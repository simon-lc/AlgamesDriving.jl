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

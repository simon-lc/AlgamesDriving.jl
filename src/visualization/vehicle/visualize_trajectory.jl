function set_state!(vis::Visualizer, model::AbstractGameModel, sce::Scenario{T},
    x::SVx; key::Int=0) where {T,SVx}
    p = model.p
    for i = 1:p
        tr = transRot(model, x, i)
        player_path = "env/player_key$key/player$(sce.player[i].id)"
        settransform!(vis[player_path], tr)
    end
    return nothing
end

function Algames.set_traj!(vis::Visualizer, model::AbstractGameModel, sce::Scenario{T},
    traj; key::Int=0) where {T}
    N = length(traj)
    anim = MeshCat.Animation(10)
    for k = 1:N
        MeshCat.atframe(anim, k) do
            set_state!(vis, model, sce, Algames.state(traj[k]), key=key)
        end
    end
    MeshCat.setanimation!(vis, anim)
    return nothing
end

function set_waypoint!(vis::Visualizer, model::AbstractGameModel, sce::Scenario{T},
    x::SVx, k::Int; key::Int=0) where {T,SVx}
    p = model.p
    for i = 1:p
        tr = transRot(model, x, i)
        waypoint_path = "env/waypoint_key$key/player$(sce.player[i].id)/$k"
        settransform!(vis[waypoint_path], tr)
    end
    return nothing
end

function set_waypoint_traj!(vis::Visualizer, model::AbstractGameModel, sce::Scenario{T},
    traj; key::Int=0) where {T}
    N = length(traj)
    for k = 1:N
        set_waypoint!(vis, model, sce, Algames.state(traj[k]), k, key=key)
    end
    return nothing
end

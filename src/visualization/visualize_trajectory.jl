

function set_state!(vis::Visualizer, model::AbstractGameModel, sce::Scenario{T},
    x::SVx) where {T,SVx}
    p = model.p
    for i = 1:p
        tr = transRot(model, x, i)
        player_path = "env/player/player$(sce.player[i].id)"
        settransform!(vis[player_path], tr)
    end
    return nothing
end

function Algames.set_traj!(vis::Visualizer, model::AbstractGameModel, sce::Scenario{T},
    traj) where {T}
    p = model.p
    N = length(traj)
    anim = MeshCat.Animation(10)
    for k = 1:N
        MeshCat.atframe(anim, k) do
            set_state!(vis, model, sce, Algames.state(traj[k]))
        end
    end
    MeshCat.setanimation!(vis, anim)
    return nothing
end

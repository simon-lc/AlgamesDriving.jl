@with_kw mutable struct MPCOptions209{T}
    "Maximum horizon of the MPC"
    h::T=3.0

    "Maximum number of MPC steps"
    M::Int=300
end

mutable struct MPCStatistics209{T}
    t::T
    dt::Vector{T}
    traj::Vector{Algames.KnotPoint}
end

function MPCStatistics209()
    t = 0.0
    dt = zeros(0)
    traj = Vector{Algames.KnotPoint}()
    return MPCStatistics209(t,dt,traj)
end

function Algames.reset!(stats::MPCStatistics209)
    stats.t = 0.0
    stats.dt = zeros(0)
    stats.traj = Vector{Algames.KnotPoint}()
    return nothing
end

function Algames.record!(stats::MPCStatistics209, z::Algames.KnotPoint)
    stats.t += z.dt
    push!(stats.dt, z.dt)
    push!(stats.traj, z)
    return nothing
end

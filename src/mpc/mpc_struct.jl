@with_kw mutable struct MPCOptions{T}
    "Maximum horizon of the MPC"
    h::T=3.0

    "Maximum number of MPC steps"
    M::Int=300
end

mutable struct MPCStatistics{T}
    iter::Int
    t::T
    dt::Vector{T}
    traj::Vector{Algames.KnotPoint}
end

function MPCStatistics()
    iter = 0
    t = 0.0
    dt = zeros(0)
    traj = Vector{Algames.KnotPoint}()
    return MPCStatistics(iter,t,dt,traj)
end

function Algames.reset!(stats::MPCStatistics)
    stats.iter = 0
    stats.t = 0.0
    stats.dt = zeros(0)
    stats.traj = Vector{Algames.KnotPoint}()
    return nothing
end

function Algames.record!(stats::MPCStatistics, z::Algames.KnotPoint)
    stats.iter += 1
    stats.t += z.dt
    push!(stats.dt, z.dt)
    push!(stats.traj, z)
    return nothing
end

mutable struct TrajStatistics{T}
    iter::Int
    probsize::ProblemSize
    t::T
    dt::Vector{T}
    traj::Vector{Vector{Algames.Traj}}
end

function TrajStatistics(probsize)
    iter = 0
    t = 0.0
    dt = zeros(0)
    traj = Vector{Vector{Algames.Traj}}()
    return TrajStatistics(iter, probsize,t,dt,traj)
end

function Algames.reset!(stats::TrajStatistics)
    stats.iter = 0
    stats.t = 0.0
    stats.dt = zeros(0)
    stats.traj = Vector{Vector{Algames.Traj}}()
    return nothing
end

function Algames.record!(stats::TrajStatistics, dt::T,
    trajs::Vector{<:Algames.Traj}) where {T}
    @assert length(trajs) == stats.probsize.p
    stats.iter += 1
    stats.t += dt
    push!(stats.dt, dt)
    push!(stats.traj, trajs)
    return nothing
end

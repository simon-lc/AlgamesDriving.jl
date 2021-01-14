mutable struct Scenario
    p::Int
    roadway::Roadway
    player::Vector{Player}
end

mutable struct Roadway
    l::Int
    lane::Vector{Lane}
end

mutable struct Player{SMQ,SMR,SVx,SVu,T}
    x0::SVx        # Initial state
    lane_id::Int   # ID of the lane the vehicle drives in
    Q::SMQ         # LQR Cost Q
    R::SMR         # LQR Cost R
    xf::SVx        # LQR Cost xf
    uf::SVu        # LQR Cost uf
    u_min::SVu     # Bound constraint on the controls
    u_max::SVu     # Bound constraint on the controls
    r_col::T       # Collision radius of the car
    r_cost::T      # Radius of the collision avoidance cost
    μ::T           # Amplitude of the collision avoidance cost
end

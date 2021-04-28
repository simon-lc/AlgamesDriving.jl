mutable struct Scenario{T}
    p::Int
    model::AbstractGameModel
    roadway::Roadway{T}
    player::Vector{Player{T}}
end

function Scenario(model::AbstractGameModel, roadway::Roadway{T}, players::Vector{<:Player}) where {T}
    p = model.p
    @assert p == length(players)
    sort!(players, by = x -> x.id)
    if [players[i].id for i=1:p] != (1:p)
        # @show "Reindexing the players"
        for i = 1:p
            players[i].id = i
        end
    end
    sce = Scenario{T}(p,model,roadway,players)
    return sce
end

################################################################################
# Getters
################################################################################

function get_state(sce::Scenario{T}) where {T}
    model = sce.model
    n = model.n
    p = model.p
    pz = model.pz
    x0 = zeros(n)
    for i = 1:p
        x0[pz[i]] = sce.player[i].x0
    end
    return SVector{n,T}(x0)
end

function get_control_bounds(sce::Scenario{T}) where {T}
    model = sce.model
    m = model.m
    p = model.p
    pu = model.pu
    u_min = zeros(m)
    u_max = zeros(m)
    for i = 1:p
        u_min[pu[i]] = sce.player[i].u_min
        u_max[pu[i]] = sce.player[i].u_max
    end
    return SVector{m,T}(u_min), SVector{m,T}(u_max)
end

function Algames.GameObjective(N::Int, sce::Scenario{T}) where {T}
    model = sce.model
    n = model.n
    p = model.p

    Q = [sce.player[i].Q for i=1:p]
    R = [sce.player[i].R for i=1:p]
    xf = [sce.player[i].xf for i=1:p]
    uf = [sce.player[i].uf for i=1:p]
    game_obj = Algames.GameObjective(Q, R, xf, uf, N, model)

    radius = [sce.player[i].r_cost for i=1:p]
    μ = [sce.player[i].μ for i=1:p]
    add_collision_cost!(game_obj, radius, μ)
    return game_obj
end

function Algames.GameConstraintValues(N::Int, sce::Scenario{T}) where {T}
    model = sce.model
    roadway = sce.roadway
    p = model.p
    probsize = ProblemSize(N,model)
    game_con = Algames.GameConstraintValues(probsize)

    radius = [sce.player[i].r_col for i=1:p]
    add_collision_avoidance!(game_con, radius)

    u_min, u_max = get_control_bounds(sce)
    if u_min != -Inf*ones(model.m) || u_max != Inf*ones(model.m) # Avoid adding void constraints
        add_control_bound!(game_con, u_max, u_min)
    end

    for i = 1:p
        player = sce.player[i]
        v_min = player.v_min
        v_max = player.v_max
        if v_min != -Inf || v_max != Inf # Avoid adding void constraints
            add_velocity_bound!(model, game_con, i, v_max, v_min)
        end
    end

    for i = 1:p
        player = sce.player[i]
        lane = roadway.lane[player.lane_id]
        if length(lane.circle) > 0 # Avoid adding void constraints
            ri = radius[i]
            xc = [c.x for c in lane.circle]
            yc = [c.y for c in lane.circle]
            rc = [c.r for c in lane.circle]
            add_circle_constraint!(game_con, i, xc, yc, rc .+ ri)
        end
    end

    for i = 1:p
        player = sce.player[i]
        lane = roadway.lane[player.lane_id]
        walls = deepcopy(lane.wall)
        ri = radius[i]
        for j = 1:length(lane.wall)
            walls[j].p1 -= ri*walls[j].v
            walls[j].p2 -= ri*walls[j].v
        end
        add_wall_constraint!(game_con, i, walls)
    end

    return game_con
end


function Algames.GameProblem(N::Int, dt::T, sce::Scenario{T}, opts::Options{T}) where {T}
    x0 = get_state(sce)
    game_obj = GameObjective(N, sce)
    game_con = GameConstraintValues(N, sce)
    prob = Algames.GameProblem(N, dt, x0, sce.model, opts, game_obj, game_con)
    return prob
end

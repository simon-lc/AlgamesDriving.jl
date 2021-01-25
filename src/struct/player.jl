################################################################################
# Player
################################################################################

mutable struct Player{T,SMQ,SMR,SVx,SVu}
	x0::SVx        # Initial state
	id::Int # ID of the player
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

function Player(x0::SVx, id::Int, lane_id::Int, Q::SMQ, R::SMR, xf::SVx, uf::SVu,
    u_min::SVu, u_max::SVu, r_col::T, r_cost::T, μ::T) where {SMQ,SMR,SVx,SVu,T}
    return Player{T,SMQ,SMR,SVx,SVu}(x0, id, lane_id, Q, R, xf, uf, u_min, u_max, r_col, r_cost, μ)
end

################################################################################
# PlayerOptions
################################################################################
abstract type PlayerOptions
end

@with_kw mutable struct HighwayPlayerOptions{ni,mi,T} <: PlayerOptions
    # Options
	"Lane id."
	lane_id::Int=1

	"LQR Cost uf."
	uf::SVector{mi,T}=zeros(SVector{mi,T})

	"Bound constraint on the controls."
	u_min::SVector{mi,T}=-Inf*ones(SVector{mi,T})
	u_max::SVector{mi,T}=Inf*ones(SVector{mi,T})

	"Collision avoidance constraint radius."
	r_col::T=0.08

	"Cost avoidance cost radius."
	r_cost::T=0.25
end

function Player(model::AbstractGameModel, lane::Lane{T};
				   id::Int=1,
				   x0::VehicleState=lane.start.x_nom,
				   xf::VehicleState=lane.start.x_max,
				   Q::Diagonal{T,SVector{ni,T}}=Diagonal(1.0*ones(SVector{model.ni[1],T})),
				   R::Diagonal{T,SVector{mi,T}}=Diagonal(0.1*ones(SVector{model.mi[1],T})),
				   μ::T=20.0,
				   opts::HighwayPlayerOptions{ni,mi,T}=HighwayPlayerOptions{model.ni[1],model.mi[1],T}(),
		) where {ni,mi,T}

	# @show x0
	x0_ = specialize(model, x0)
	xf_ = specialize(model, xf)

	player = Player(
		x0_,
		id,
		lane.id,
		Q,
		R,
		xf_,
		opts.uf,
		opts.u_min,
		opts.u_max,
		opts.r_col,
		opts.r_cost,
		μ,
		)
	return player
end



# # Test HighwayRoadway
# T = Float64
# p = 3
# model = UnicycleGame(p=p)
# n = model.n
# m = model.m
# ni = model.ni
# mi = model.mi
# player_opts = HighwayPlayerOptions{n,m,ni[1],mi[1],T}()
# build_player(model, player_opts)
#
# T = Float64
# Diagonal(ones(SVector{4,T}))
# Diagonal{T,SVector{4,T}}

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
	v_min::T       # Bound constraint on the velocity
	v_max::T       # Bound constraint on the velocity
    r_col::T       # Collision radius of the car
    r_cost::T      # Radius of the collision avoidance cost
    μ::T           # Amplitude of the collision avoidance cost
end

function Player(model::AbstractGameModel, lane::Lane{T};
				   id::Int=1,
				   x0::VehicleState=lane.start.x_nom,
				   xf::VehicleState=lane.start.x_max,
				   uf::SVector{mi,T}=zeros(SVector{model.mi[1],T}),
				   Q::Diagonal{T,SVector{ni,T}}=Diagonal(1.0*ones(SVector{model.ni[1],T})),
				   R::Diagonal{T,SVector{mi,T}}=Diagonal(0.1*ones(SVector{model.mi[1],T})),
				   r_col::T=0.08,
				   r_cost::T=0.22,
				   μ::T=20.0,
				   u_min::SVector{mi,T}=-Inf*ones(SVector{model.mi[1],T}),
				   u_max::SVector{mi,T}=Inf*ones(SVector{model.mi[1],T}),
				   v_min::T=-Inf,
				   v_max::T=Inf,
		) where {ni,mi,T}

	x0_ = specialize(model, x0)
	xf_ = specialize(model, xf)

	player = Player(
		x0_,
		id,
		lane.id,
		Q,
		R,
		xf_,
		uf,
		u_min,
		u_max,
		v_min,
		v_max,
		r_col,
		r_cost,
		μ,
		)
	return player
end
